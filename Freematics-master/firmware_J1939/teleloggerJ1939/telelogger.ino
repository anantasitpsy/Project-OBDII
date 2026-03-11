/******************************************************************************
Arduino sketch of a vehicle data logger and telemeter for Freematics Hub
Modified for J1939 passive listening with comprehensive error handling
Works with Freematics ONE+ Model A and Model B
Based on original work by Stanley Huang stanley@freematics.com.au
Modified for J1939 protocol support
Distributed under BSD license
Visit https://freematics.com/products for hardware information
Visit https://hub.freematics.com to view live and history telemetry data
*
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
******************************************************************************/

#include <FreematicsPlus.h>
#include <httpd.h>
#include "config.h"
#include "telestore.h"
#include "teleclient.h"
#if BOARD_HAS_PSRAM
#include "esp32/himem.h"
#endif
#include "driver/adc.h"
#include "nvs_flash.h"
#include "nvs.h"
#if ENABLE_OLED
#include "FreematicsOLED.h"
#endif

// states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_GPS_ONLINE 0x20
#define STATE_CELL_CONNECTED 0x40
#define STATE_WIFI_CONNECTED 0x80
#define STATE_WORKING 0x100
#define STATE_STANDBY 0x200

// J1939 Error codes
#define ERR_J1939_NO_DATA        0x01
#define ERR_J1939_TIMEOUT        0x02
#define ERR_J1939_INVALID_DATA   0x03
#define ERR_J1939_BUS_OFF        0x04
#define ERR_J1939_INIT_FAIL      0x05

// Error handling thresholds
#define J1939_MIN_PACKET_SIZE    3
#define J1939_MAX_PACKET_SIZE    128
#define MAX_CONSECUTIVE_J1939_ERR 50
#define WATCHDOG_TIMEOUT         30000  // 30 seconds
#define DATA_STALE_TIMEOUT       5000   // 5 seconds
#define MAX_RECOVERY_ATTEMPTS    3
#define MAX_NETWORK_RETRIES      3
#define NETWORK_RETRY_DELAY      5000
#define MAX_CONSECUTIVE_NET_FAIL 10
#define STORAGE_CHECK_INTERVAL   60000  // 1 minute

// Data validation ranges (ปรับตาม J1939 Simulator)
#define RPM_MIN        0
#define RPM_MAX        8500      // Simulator max: 8012, เผื่อไว้หน่อย
#define SPEED_MIN      0
#define SPEED_MAX      255       // Simulator max: 250
#define TEMP_MIN       -40       // Simulator min: -40
#define TEMP_MAX       220       // Simulator max: 215
#define FUEL_MIN       0
#define FUEL_MAX       100       // Simulator max: 100
#define THROTTLE_MIN   0
#define THROTTLE_MAX   100       // Simulator max: 100
#define LOAD_MIN       0
#define LOAD_MAX       255       // Simulator max: 255
#define MAF_MIN        0
#define MAF_MAX        3500      // Simulator max: 3205
#define MANIFOLD_MIN   0
#define MANIFOLD_MAX   6500      // Simulator max: 6405
#define TIMING_MIN     -210      // Simulator min: -200
#define TIMING_MAX     320       // Simulator max: 301
#define TURBO_MIN      0
#define TURBO_MAX      500       // Simulator: 127 Kpa


// J1939 PGN definitions
#define PGN_RPM          61444
#define PGN_SPEED        65265
#define PGN_COOLANT      65262
#define PGN_FUEL_LEVEL   65276
#define PGN_THROTTLE     65266
#define PGN_LOAD         61443
#define PGN_INTAKE_AIR   65269
#define PGN_MAF          61450
#define PGN_MANIFOLD     65194
#define PGN_BATTERY      65271
#define PGN_TIMING       65159

uint8_t currentPGNIndex = 0; 
uint32_t lastReportTime = 0;

// รายชื่อ PGN ที่ต้องการวนหาตามลำดับ
const uint32_t targetPGNs[11] = {
    PGN_SPEED,        // Index 0
    PGN_RPM,          // Index 1
    PGN_COOLANT,      // Index 2
    PGN_INTAKE_AIR,   // Index 3
    PGN_MAF,          // Index 4
    PGN_MANIFOLD,     // Index 5
    PGN_THROTTLE,     // Index 6
    PGN_TIMING,       // Index 7
    PGN_LOAD,         // Index 8
    PGN_FUEL_LEVEL,   // Index 9
    PGN_BATTERY,      // Index 10
};

const uint8_t NUM_PGNS = sizeof(targetPGNs) / sizeof(targetPGNs[0]);  
const uint32_t LOOP_INTERVAL = 0;
const uint32_t DATA_COLLECT_INTERVAL = 800;
const uint32_t DATA_REPORT_INTERVAL = DATA_COLLECT_INTERVAL * NUM_PGNS;

/*******************************************************************************
  Error Tracking Structures
*******************************************************************************/
typedef struct {
    uint32_t consecutiveErrors;
    uint32_t totalErrors;
    uint32_t lastErrorTime;
    uint8_t lastErrorCode;
    bool busHealthy;
} J1939ErrorState;

typedef struct {
    uint32_t failureCount;
    uint32_t consecutiveFailures;
    uint32_t lastFailureTime;
    bool connected;
} NetworkErrorState;

typedef struct {
    bool available;
    uint32_t writeErrors;
    uint32_t lastCheckTime;
    uint32_t lastGoodWrite;
} StorageErrorState;

typedef struct {
    uint32_t lastUpdate;
    bool valid;
    int value;
} DataPoint;

typedef struct {
    DataPoint rpm;
    DataPoint speed;
    DataPoint coolantTemp;
    DataPoint fuelLevel;
    DataPoint throttle;
    DataPoint load;
    DataPoint intakeTemp;
    DataPoint batteryVolt;
    DataPoint maf;
    DataPoint manifoldPressure;
    DataPoint timing;
} VehicleDataCache;

typedef struct {
    uint32_t uptimeSeconds;
    uint32_t totalPacketsReceived;
    uint32_t totalPacketsProcessed;
    uint32_t totalPacketsDropped;
    float successRate;
} SystemHealth;

// Global error state tracking
J1939ErrorState j1939State = {0, 0, 0, 0, true};
NetworkErrorState netState = {0, 0, 0, false};
StorageErrorState storageState = {false, 0, 0, 0};
VehicleDataCache dataCache = {
    {0, false, 0},  // rpm
    {0, false, 0},  // speed
    {0, false, 0},  // coolantTemp
    {0, false, 0},  // fuelLevel
    {0, false, 0},  // throttle
    {0, false, 0},  // load
    {0, false, 0},  // intakeTemp
    {0, false, 0}   // batteryVolt
};
SystemHealth health = {0, 0, 0, 0, 0.0};

uint32_t lastDataReceived = 0;
uint8_t recoveryAttempts = 0;

/*******************************************************************************
  Original Data Structures
*******************************************************************************/
typedef struct {
  byte pid;
  byte tier;
  int value;
  uint32_t ts;
} PID_POLLING_INFO;

// Keep for compatibility with HTTP API
PID_POLLING_INFO obdData[]= {
  {PID_SPEED, 1},
  {PID_RPM, 1},
  {PID_THROTTLE, 1},
  {PID_ENGINE_LOAD, 1},
  {PID_FUEL_LEVEL, 1},
  {PID_TIMING_ADVANCE, 1},
  {PID_COOLANT_TEMP, 1},
  {PID_INTAKE_TEMP, 1},
  {PID_MAF_FLOW, 1},
  {PID_INTAKE_MAP, 1},
};

CBufferManager bufman;
Task subtask;

#if ENABLE_MEMS
float accBias[3] = {0}; // calibrated reference accelerometer data
float accSum[3] = {0};
float acc[3] = {0};
float gyr[3] = {0};
float mag[3] = {0};
uint8_t accCount = 0;
#endif
int deviceTemp = 0;

// config data
char apn[32];
#if ENABLE_WIFI
char wifiSSID[32] = WIFI_SSID;
char wifiPassword[32] = WIFI_PASSWORD;
#endif
nvs_handle_t nvs;

// live data
String netop;
String ip;
int16_t rssi = 0;
int16_t rssiLast = 0;
char vin[18] = {0};
uint16_t dtc[6] = {0};
float batteryVoltage = 0;
GPS_DATA* gd = 0;

char devid[12] = {0};
char isoTime[32] = {0};

// stats data
uint32_t lastMotionTime = 0;
uint32_t timeoutsOBD = 0;
uint32_t timeoutsNet = 0;
uint32_t lastStatsTime = 0;

int32_t syncInterval = SERVER_SYNC_INTERVAL * 1000;
int32_t dataInterval = 1000;

#if STORAGE != STORAGE_NONE
int fileid = 0;
uint16_t lastSizeKB = 0;
#endif

byte ledMode = 0;

bool serverSetup(IPAddress& ip);
void serverProcess(int timeout);
void processMEMS(CBuffer* buffer);
bool processGPS(CBuffer* buffer);
void processBLE(int timeout);

class State {
public:
  bool check(uint16_t flags) { return (m_state & flags) == flags; }
  void set(uint16_t flags) { m_state |= flags; }
  void clear(uint16_t flags) { m_state &= ~flags; }
  uint16_t m_state = 0;
};

FreematicsESP32 sys;

class OBD : public COBD
{
protected:
  void idleTasks()
  {
    // do some quick tasks while waiting for OBD response
#if ENABLE_MEMS
    processMEMS(0);
#endif
    processBLE(0);
  }
};

OBD obd;

MEMS_I2C* mems = 0;

#if STORAGE == STORAGE_SPIFFS
SPIFFSLogger logger;
#elif STORAGE == STORAGE_SD
SDLogger logger;
#endif

#if SERVER_PROTOCOL == PROTOCOL_UDP
TeleClientUDP teleClient;
#else
TeleClientHTTP teleClient;
#endif

#if ENABLE_OLED
OLED_SH1106 oled;
#endif

State state;

/*******************************************************************************
  Error Handling and Validation Functions
*******************************************************************************/
void updateDataPoint(DataPoint* dp, int value) {
    dp->value = value;
    dp->lastUpdate = millis();
    dp->valid = true;
}

bool isDataStale(uint32_t lastUpdate) {
    return (millis() - lastUpdate) > DATA_STALE_TIMEOUT;
}

void checkDataFreshness() {
    if (isDataStale(dataCache.rpm.lastUpdate) && dataCache.rpm.valid) {
        Serial.println("[WARN] RPM data is stale");
        dataCache.rpm.valid = false;
    }
    if (isDataStale(dataCache.speed.lastUpdate) && dataCache.speed.valid) {
        Serial.println("[WARN] Speed data is stale");
        dataCache.speed.valid = false;
    }
    if (isDataStale(dataCache.coolantTemp.lastUpdate) && dataCache.coolantTemp.valid) {
        Serial.println("[WARN] Coolant temp data is stale");
        dataCache.coolantTemp.valid = false;
    }
}

bool validateJ1939Packet(byte* buf, int bytes) {
    // Check packet size
    if (bytes < J1939_MIN_PACKET_SIZE || bytes > J1939_MAX_PACKET_SIZE) {
        Serial.printf("[ERR] Invalid packet size: %d\n", bytes);
        j1939State.lastErrorCode = ERR_J1939_INVALID_DATA;
        return false;
    }

    // Check for all 0xFF (common error pattern)
    bool allFF = true;
    for (int i = 0; i < bytes; i++) {
        if (buf[i] != 0xFF) {
            allFF = false;
            break;
        }
    }
    if (allFF) {
        Serial.println("[ERR] Packet is all 0xFF");
        return false;
    }

    return true;
}

bool validateRPM(float rpm) {
    if (rpm < RPM_MIN || rpm > RPM_MAX) {
        Serial.printf("[ERR] RPM out of range: %.0f\n", rpm);
        return false;
    }
    return true;
}

bool validateSpeed(float speed) {
    if (speed < SPEED_MIN || speed > SPEED_MAX) {
        Serial.printf("[ERR] Speed out of range: %.1f\n", speed);
        return false;
    }
    return true;
}

bool validateTemperature(int temp) {
    if (temp < TEMP_MIN || temp > TEMP_MAX) {
        Serial.printf("[ERR] Temperature out of range: %d\n", temp);
        return false;
    }
    return true;
}

bool validateFuel(float fuel) {
    if (fuel < FUEL_MIN || fuel > FUEL_MAX) {
        Serial.printf("[ERR] Fuel level out of range: %.1f\n", fuel);
        return false;
    }
    return true;
}

bool validateThrottle(float throttle) {
    if (throttle < THROTTLE_MIN || throttle > THROTTLE_MAX) {
        Serial.printf("[ERR] Throttle out of range: %.1f\n", throttle);
        return false;
    }
    return true;
}

bool validateLoad(int load) {
    if (load < LOAD_MIN || load > LOAD_MAX) {
        Serial.printf("[ERR] Load out of range: %d\n", load);
        return false;
    }
    return true;
}

bool validateMAF(float maf) {
    if (maf < MAF_MIN || maf > MAF_MAX) {
        Serial.printf("[ERR] MAF out of range: %.0f\n", maf);
        return false;
    }
    return true;
}

bool validateManifold(int pressure) {
    if (pressure < MANIFOLD_MIN || pressure > MANIFOLD_MAX) {
        Serial.printf("[ERR] Manifold pressure out of range: %d\n", pressure);
        return false;
    }
    return true;
}

bool validateTiming(float timing) {
    if (timing < TIMING_MIN || timing > TIMING_MAX) {
        Serial.printf("[ERR] Timing out of range: %.1f\n", timing);
        return false;
    }
    return true;
}

bool validateTurbo(int boost) {
    if (boost < TURBO_MIN || boost > TURBO_MAX) {
        Serial.printf("[ERR] Turbo out of range: %d\n", boost);
        return false;
    }
    return true;
}

void updateHealthMetrics() {
    health.uptimeSeconds = millis() / 1000;
    health.totalPacketsReceived = j1939State.totalErrors + health.totalPacketsProcessed;

    if (health.totalPacketsReceived > 0) {
        health.successRate = (float)health.totalPacketsProcessed / health.totalPacketsReceived * 100.0;
    }

    health.totalPacketsDropped = j1939State.totalErrors;
}

void printHealthStatus() {
    updateHealthMetrics();

    Serial.println("\n=== SYSTEM HEALTH ===");
    Serial.printf("Uptime: %d seconds\n", health.uptimeSeconds);
    Serial.printf("Packets Received: %d\n", health.totalPacketsReceived);
    Serial.printf("Packets Processed: %d\n", health.totalPacketsProcessed);
    Serial.printf("Packets Dropped: %d\n", health.totalPacketsDropped);
    Serial.printf("Success Rate: %.1f%%\n", health.successRate);
    Serial.printf("J1939 Bus: %s\n", j1939State.busHealthy ? "HEALTHY" : "UNHEALTHY");
    Serial.printf("Network: %s\n", netState.connected ? "CONNECTED" : "DISCONNECTED");
    Serial.printf("Storage: %s\n", storageState.available ? "AVAILABLE" : "UNAVAILABLE");
    Serial.println("====================\n");
}

bool attemptJ1939Recovery() {
    Serial.println("[RECOVERY] Reinitializing J1939...");

    // Reset OBD interface (no direct end(), use link reset instead)
    sys.resetLink();
    delay(1000);

    obd.begin(sys.link);
    if (obd.init(PROTO_J1939)) {
        Serial.println("[RECOVERY] J1939 reinitialized successfully");
        j1939State.consecutiveErrors = 0;
        lastDataReceived = millis();
        recoveryAttempts = 0;
        return true;
    }

    Serial.println("[RECOVERY] Failed to reinitialize J1939");
    return false;
}

void watchdogCheck() {
    uint32_t timeSinceData = millis() - lastDataReceived;

    if (timeSinceData > WATCHDOG_TIMEOUT) {
        Serial.printf("[WATCHDOG] No data for %d seconds\n", timeSinceData / 1000);

        if (recoveryAttempts < MAX_RECOVERY_ATTEMPTS) {
            Serial.println("[WATCHDOG] Attempting recovery...");
            attemptJ1939Recovery();
            recoveryAttempts++;
        } else {
            Serial.println("[WATCHDOG] Max recovery attempts reached, system reset needed");
            // Log critical error
            beep(2000);  // Alert user
            delay(5000);
            ESP.restart();
        }
    }
}

bool checkStorageHealth() {
    if (millis() - storageState.lastCheckTime < STORAGE_CHECK_INTERVAL) {
        return storageState.available;
    }

    storageState.lastCheckTime = millis();

    #if STORAGE != STORAGE_NONE
    // Simple availability check
    storageState.available = state.check(STATE_STORAGE_READY);
    #endif

    return storageState.available;
}

bool writeToStorageWithErrorHandling(CBuffer* buffer) {
    #if STORAGE != STORAGE_NONE
    if (!checkStorageHealth()) {
        Serial.println("[STORAGE] Storage not available, skipping write");
        return false;
    }

    // serialize returns void in this library version, so assume success
    // (No direct way to check failure; in production, monitor logger state if available)
    buffer->serialize(logger);
    storageState.lastGoodWrite = millis();

    // Note: Write error handling removed due to void return; assume success for compilation
    // If logger has an error flag, check it here in future updates
    return true;
    #else
    return false;
    #endif
}

bool transmitWithRetry(const char* data, int length) {
    for (int attempt = 0; attempt < MAX_NETWORK_RETRIES; attempt++) {
        if (teleClient.transmit(data, length)) {
            // Success - reset consecutive failures
            netState.consecutiveFailures = 0;
            netState.connected = true;
            return true;
        }

        // Failed - log and retry
        Serial.printf("[NET] Transmission failed, attempt %d/%d\n", 
                     attempt + 1, MAX_NETWORK_RETRIES);

        if (attempt < MAX_NETWORK_RETRIES - 1) {
            delay(NETWORK_RETRY_DELAY);
        }
    }

    // All retries failed
    netState.failureCount++;
    netState.consecutiveFailures++;
    netState.lastFailureTime = millis();

    // Check if we need to reconnect
    if (netState.consecutiveFailures >= MAX_CONSECUTIVE_NET_FAIL) {
        Serial.println("[NET] Too many failures, forcing reconnect");
        netState.connected = false;
        return false;
    }

    return false;
}

void printTimeoutStats()
{
  Serial.print("Timeouts: J1939:");
  Serial.print(j1939State.totalErrors);
  Serial.print(" Network:");
  Serial.println(timeoutsNet);
}

void beep(int duration)
{
    // turn on buzzer at 2000Hz frequency 
    sys.buzzer(2000);
    delay(duration);
    // turn off buzzer
    sys.buzzer(0);
}

#if LOG_EXT_SENSORS
void processExtInputs(CBuffer* buffer)
{
#if LOG_EXT_SENSORS == 1
  uint8_t levels[2] = {(uint8_t)digitalRead(PIN_SENSOR1), (uint8_t)digitalRead(PIN_SENSOR2)};
  buffer->add(PID_EXT_SENSORS, ELEMENT_UINT8, levels, sizeof(levels), 2);
#elif LOG_EXT_SENSORS == 2
  uint16_t reading[] = {adc1_get_raw(ADC1_CHANNEL_0), adc1_get_raw(ADC1_CHANNEL_1)};
  Serial.print("GPIO0:");
  Serial.print((float)reading[0] * 3.15 / 4095 - 0.01);
  Serial.print(" GPIO1:");
  Serial.println((float)reading[1] * 3.15 / 4095 - 0.01);
  buffer->add(PID_EXT_SENSORS, ELEMENT_UINT16, reading, sizeof(reading), 2);
#endif
}
#endif

/*******************************************************************************
  HTTP API
*******************************************************************************/
#if ENABLE_HTTPD
int handlerLiveData(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    int n = snprintf(buf, bufsize, "{\"obd\":{\"vin\":\"%s\",\"battery\":%.1f,\"pid\":[", vin, batteryVoltage);
    uint32_t t = millis();

    // Use cached J1939 data
    if (dataCache.rpm.valid) {
        n += snprintf(buf + n, bufsize - n, "{\"pid\":%u,\"value\":%d,\"age\":%u},",
            0x100 | PID_RPM, dataCache.rpm.value, (unsigned int)(t - dataCache.rpm.lastUpdate));
    }
    if (dataCache.speed.valid) {
        n += snprintf(buf + n, bufsize - n, "{\"pid\":%u,\"value\":%d,\"age\":%u},",
            0x100 | PID_SPEED, dataCache.speed.value, (unsigned int)(t - dataCache.speed.lastUpdate));
    }
    if (dataCache.throttle.valid) {
        n += snprintf(buf + n, bufsize - n, "{\"pid\":%u,\"value\":%d,\"age\":%u},",
            0x100 | PID_THROTTLE, dataCache.throttle.value, (unsigned int)(t - dataCache.throttle.lastUpdate));
    }
    if (dataCache.load.valid) {
        n += snprintf(buf + n, bufsize - n, "{\"pid\":%u,\"value\":%d,\"age\":%u},",
            0x100 | PID_ENGINE_LOAD, dataCache.load.value, (unsigned int)(t - dataCache.load.lastUpdate));
    }
    if (dataCache.coolantTemp.valid) {
        n += snprintf(buf + n, bufsize - n, "{\"pid\":%u,\"value\":%d,\"age\":%u},",
            0x100 | PID_COOLANT_TEMP, dataCache.coolantTemp.value, (unsigned int)(t - dataCache.coolantTemp.lastUpdate));
    }
    if (dataCache.intakeTemp.valid) {
        n += snprintf(buf + n, bufsize - n, "{\"pid\":%u,\"value\":%d,\"age\":%u},",
            0x100 | PID_INTAKE_TEMP, dataCache.intakeTemp.value, (unsigned int)(t - dataCache.intakeTemp.lastUpdate));
    }

    if (n > 0 && buf[n-1] == ',') n--;
    n += snprintf(buf + n, bufsize - n, "]}");
#if ENABLE_MEMS
    if (accCount) {
      n += snprintf(buf + n, bufsize - n, ",\"mems\":{\"acc\":[%d,%d,%d],\"stationary\":%u}",
          (int)((accSum[0] / accCount - accBias[0]) * 100), (int)((accSum[1] / accCount - accBias[1]) * 100), (int)((accSum[2] / accCount - accBias[2]) * 100),
          (unsigned int)(millis() - lastMotionTime));
    }
#endif
    if (gd && gd->ts) {
      n += snprintf(buf + n, bufsize - n, ",\"gps\":{\"utc\":\"%s\",\"lat\":%f,\"lng\":%f,\"alt\":%f,\"speed\":%f,\"sat\":%d,\"age\":%u}",
          isoTime, gd->lat, gd->lng, gd->alt, gd->speed, (int)gd->sat, (unsigned int)(millis() - gd->ts));
    }
    buf[n++] = '}';
    param->contentLength = n;
    param->contentType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
}
#endif
/*******************************************************************************
  J1939 Data Display Function (Freematics Format)
*******************************************************************************/
void printJ1939DataPacket(uint32_t pgn, const char* name, int value) {
    static uint32_t packetCount = 0;
    uint32_t ts = millis();
    
    // Format: [DAT] DEVICEID#PACKET:TIMESTAMP,PGN:VALUE
    Serial.printf("[DAT] %s#%lu:%lu,%lu:%d\n", devid, packetCount++, ts, pgn, value);
}

void printJ1939DataPacketFloat(uint32_t pgn, const char* name, float value) {
    static uint32_t packetCount = 0;
    uint32_t ts = millis();
    
    Serial.printf("[DAT] %s#%lu:%lu,%lu:%.2f\n", devid, packetCount++, ts, pgn, value);
}

/*******************************************************************************
  J1939 Data Processing - Round Robin Single PGN Capture
*******************************************************************************/
#if ENABLE_OBD
void processJ1939() 
{
    byte buf[128];
    int bytes;
    
    // ดึง PGN เป้าหมายจาก Global Index
    uint32_t currentTargetPGN = targetPGNs[currentPGNIndex];
    bool foundTarget = false;
    
    // อ่านข้อมูลออกจาก Buffer ให้หมด
    while ((bytes = obd.receiveData(buf, sizeof(buf))) > 0) {
        
        // ตรวจสอบความถูกต้องพื้นฐาน
        if (!validateJ1939Packet(buf, bytes)) {
            j1939State.totalErrors++;
            continue;
        }

        uint32_t pgn = (uint32_t)buf[0] << 8 | buf[1];
        uint32_t ts = millis();
        
        lastDataReceived = ts;
        j1939State.busHealthy = true;

        // ====================================================
        // หัวใจสำคัญ: ตรวจสอบว่าเป็น PGN ที่เรากำลังล่าอยู่หรือไม่?
        // ====================================================
        if (pgn == currentTargetPGN) {
            
            // ถ้าใช่ ให้ทำการ Decode ตามชนิดของ Index นั้น
            switch(currentPGNIndex) {
                case 0: // Speed
                    if (bytes >= 5) {
                        float speed = (buf[3] + (buf[4] * 256)) / 256.0;
                        if (validateSpeed(speed)) {
                            updateDataPoint(&dataCache.speed, (int)speed);
                            //Serial.printf("✓ [FOUND-Index:%d] SPEED: %d\n", currentPGNIndex, (int)speed);
                        }
                    }
                    break;
                
                case 1: // RPM
                    if (bytes >= 7) {
                        float rpm = (buf[5] + (buf[6] * 256)) * 0.125;
                        if (validateRPM(rpm)) {
                            updateDataPoint(&dataCache.rpm, (int)rpm);
                            //Serial.printf("✓ [FOUND-Index:%d] RPM: %d\n", currentPGNIndex, (int)rpm);
                        }
                    }
                    break;
                
                case 2: // Coolant
                    if (bytes >= 3) {
                        int temp = buf[2] - 40;
                        updateDataPoint(&dataCache.coolantTemp, temp);
                        //Serial.printf("✓ [FOUND-Index:%d] COOLANT: %d\n", currentPGNIndex, temp);
                    }
                    break;

                case 3: // intake Air
                    if (bytes >= 8) {
                        int intake = buf[7] - 40;
                        updateDataPoint(&dataCache.intakeTemp, intake);
                        //Serial.printf("✓ [FOUND-Index:%d] INTAKE: %d\n", currentPGNIndex, intake);
                    }
                    break;

                case 4: // MAF
                    if (bytes >= 6) {
                        float maf = (buf[4] + (buf[5] * 256)) * 0.05;
                        updateDataPoint(&dataCache.maf, (int)maf);
                        //Serial.printf("✓ [FOUND-Index:%d] MAF: %d\n", currentPGNIndex, (int)maf);
                    }
                    break;

                case 5: // Manifold Pressure
                    if (bytes >= 5) {
                        long manifold = buf[3] + (buf[4] * 256);
                        manifold = int(manifold * 0.1);
                        updateDataPoint(&dataCache.manifoldPressure, manifold);
                        //Serial.printf("✓ [FOUND-Index:%d] MANIFOLD: %d\n", currentPGNIndex, manifold);
                    }
                    break;

                case 6: // throttle
                    if (bytes >= 9) {
                        float throttle = buf[8] * 0.4;
                        updateDataPoint(&dataCache.throttle, (int)throttle);
                        //Serial.printf("✓ [FOUND-Index:%d] THROTTLE: %d\n", currentPGNIndex, (int)throttle);
                     }
                     break;
                
                case 7: // timing
                    if (bytes >= 9) {
                        int16_t rawValue = (int16_t)((buf[8] << 8) | buf[7]);
                        float timing = 112.5 + (rawValue / 127.0);
                        if (timing > 305.0) {
                          timing -= 516.0;
                        }
                        updateDataPoint(&dataCache.timing, (int)timing);
                        //Serial.printf("✓ [FOUND-Index:%d] TIMING: %d\n", currentPGNIndex, (int)timing);
                    }
                    break;

                case 8: // Load
                    if (bytes >= 5) {
                        updateDataPoint(&dataCache.load, buf[4]);
                        //Serial.printf("✓ [FOUND-Index:%d] LOAD: %d\n", currentPGNIndex, buf[4]);
                    }
                    break;

                case 9: // fual Level
                    if (bytes >= 4) {
                        float fuel = buf[3] * 0.4;
                        updateDataPoint(&dataCache.fuelLevel, (int)fuel);
                        //Serial.printf("✓ [FOUND-Index:%d] FUEL: %d\n", currentPGNIndex, (int)fuel);
                    }
                    break;
                case 10: // battery voltage
                    if (bytes >= 8) {
                        float volt = (buf[6] + (buf[7] * 256)) * 0.05;
                        updateDataPoint(&dataCache.batteryVolt, (int)(volt*10));
                        //Serial.printf("✓ [FOUND-Index:%d] BATT: %.1f\n", currentPGNIndex, volt);
                    }
                    break;
            }
            foundTarget = true;
        } 
        else {
        }
    }
}
#endif

bool initGPS()
{
  // start GNSS receiver
  if (sys.gpsBeginExt()) {
    Serial.println("GNSS:OK(E)");
  } else if (sys.gpsBegin()) {
    Serial.println("GNSS:OK(I)");
  } else {
    Serial.println("GNSS:NO");
    return false;
  }
  return true;
}

bool processGPS(CBuffer* buffer)
{
  static uint32_t lastGPStime = 0;
  static float lastGPSLat = 0;
  static float lastGPSLng = 0;
  static uint32_t lastPrintTime = 0;  // ← เพิ่มตัวแปรสำหรับควบคุมการพิมพ์

  if (!gd) {
    lastGPStime = 0;
    lastGPSLat = 0;
    lastGPSLng = 0;
  }
#if GNSS == GNSS_STANDALONE
  if (state.check(STATE_GPS_READY)) {
    if (!sys.gpsGetData(&gd)) {
      return false;  // ← ไม่มีข้อมูล GPS ใหม่
    }
  }
#else
    if (!teleClient.cell.getLocation(&gd)) {
      return false;
    }
#endif
  if (!gd || lastGPStime == gd->time) return false;
  if (gd->date) {
    char *p = isoTime + sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u",
        (unsigned int)(gd->date % 100) + 2000, (unsigned int)(gd->date / 100) % 100, (unsigned int)(gd->date / 10000),
        (unsigned int)(gd->time / 1000000), (unsigned int)(gd->time % 1000000) / 10000, (unsigned int)(gd->time % 10000) / 100);
    unsigned char tenth = (gd->time % 100) / 10;
    if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
    *p = 'Z';
    *(p + 1) = 0;
  }
  if (gd->lng == 0 && gd->lat == 0) {
    if (millis() - lastPrintTime >= DATA_REPORT_INTERVAL) {
        if (gd->date) {
            Serial.print("[GNSS] ");
            Serial.println(isoTime);
        } else {
             // เผื่อกรณียังไม่ได้ทั้งตำแหน่งและเวลา
             Serial.println("[GNSS] Searching..."); 
        }
        
        // อัปเดตเวลาล่าสุดที่ปริ้น เพื่อให้รอกระรอบถัดไป
        lastPrintTime = millis(); 
    }
    
    return false; // ยังคง return false เพื่อไม่ให้เก็บค่า 0,0 ลง Buffer
  }
  if ((lastGPSLat || lastGPSLng) && (abs(gd->lat - lastGPSLat) > 0.001 || abs(gd->lng - lastGPSLng) > 0.001)) {
    lastGPSLat = 0;
    lastGPSLng = 0;
    return false;
  }
  lastGPSLat = gd->lat;
  lastGPSLng = gd->lng;

  float kph = gd->speed * 1.852f;
  if (kph >= 2) lastMotionTime = millis();

  // ← ส่วนนี้ทำงานทุกครั้งที่เรียก processGPS() (ทุก 800ms)
  if (buffer) {
    buffer->add(PID_GPS_TIME, ELEMENT_UINT32, &gd->time, sizeof(uint32_t));
    buffer->add(PID_GPS_LATITUDE, ELEMENT_FLOAT, &gd->lat, sizeof(float));
    buffer->add(PID_GPS_LONGITUDE, ELEMENT_FLOAT, &gd->lng, sizeof(float));
    buffer->add(PID_GPS_ALTITUDE, ELEMENT_FLOAT_D1, &gd->alt, sizeof(float));
    buffer->add(PID_GPS_SPEED, ELEMENT_FLOAT_D1, &kph, sizeof(kph));
    buffer->add(PID_GPS_HEADING, ELEMENT_UINT16, &gd->heading, sizeof(uint16_t));
    if (gd->sat) buffer->add(PID_GPS_SAT_COUNT, ELEMENT_UINT8, &gd->sat, sizeof(uint8_t));
    if (gd->hdop) buffer->add(PID_GPS_HDOP, ELEMENT_UINT8, &gd->hdop, sizeof(uint8_t));
  }
  
  if (millis() - lastPrintTime >= DATA_REPORT_INTERVAL) {
    Serial.print("[GNSS] ");
    Serial.print(gd->lat, 6);
    Serial.print(' ');
    Serial.print(gd->lng, 6);
    Serial.print(' ');
    Serial.print((int)kph);
    Serial.print("km/h SAT:");
    Serial.print(gd->sat);
    Serial.print(" HDOP:");
    Serial.println(gd->hdop);
    lastPrintTime = millis();
  }
  
  lastGPStime = gd->time;
  return true;  // ← return true = เก็บข้อมูลสำเร็จ
}

bool waitMotionGPS(int timeout)
{
  unsigned long t = millis();
  lastMotionTime = 0;
  do {
      serverProcess(100);
    if (!processGPS(0)) continue;
    if (lastMotionTime) return true;
  } while (millis() - t < timeout);
  return false;
}

#if ENABLE_MEMS
void processMEMS(CBuffer* buffer)
{
  if (!state.check(STATE_MEMS_READY)) return;

  float temp;
#if ENABLE_ORIENTATION
  ORIENTATION ori;
  if (!mems->read(acc, gyr, mag, &temp, &ori)) return;
#else
  if (!mems->read(acc, gyr, mag, &temp)) return;
#endif
  deviceTemp = (int)temp;

  accSum[0] += acc[0];
  accSum[1] += acc[1];
  accSum[2] += acc[2];
  accCount++;

  if (buffer) {
    if (accCount) {
      float value[3];
      value[0] = accSum[0] / accCount - accBias[0];
      value[1] = accSum[1] / accCount - accBias[1];
      value[2] = accSum[2] / accCount - accBias[2];
      buffer->add(PID_ACC, ELEMENT_FLOAT_D2, value, sizeof(value), 3);
#if ENABLE_ORIENTATION
      value[0] = ori.yaw;
      value[1] = ori.pitch;
      value[2] = ori.roll;
      buffer->add(PID_ORIENTATION, ELEMENT_FLOAT_D2, value, sizeof(value), 3);
#endif
    }
    accSum[0] = 0;
    accSum[1] = 0;
    accSum[2] = 0;
    accCount = 0;
  }
}

void calibrateMEMS()
{
  if (state.check(STATE_MEMS_READY)) {
    accBias[0] = 0;
    accBias[1] = 0;
    accBias[2] = 0;
    int n;
    unsigned long t = millis();
    for (n = 0; millis() - t < 1000; n++) {
      float acc[3];
      if (!mems->read(acc)) continue;
      accBias[0] += acc[0];
      accBias[1] += acc[1];
      accBias[2] += acc[2];
      delay(10);
    }
    accBias[0] /= n;
    accBias[1] /= n;
    accBias[2] /= n;
    Serial.print("ACC BIAS:");
    Serial.print(accBias[0]);
    Serial.print('/');
    Serial.print(accBias[1]);
    Serial.print('/');
    Serial.println(accBias[2]);
  }
}
#endif

void printTime()
{
  time_t utc;
  time(&utc);
  struct tm *btm = gmtime(&utc);
  if (btm->tm_year > 100) {
    char buf[64];
    sprintf(buf, "%04u-%02u-%02u %02u:%02u:%02u",
      1900 + btm->tm_year, btm->tm_mon + 1, btm->tm_mday, btm->tm_hour, btm->tm_min, btm->tm_sec);
    Serial.print("UTC:");
    Serial.println(buf);
  }
}

/*******************************************************************************
  Initializing all data logging components
*******************************************************************************/
void initialize()
{
  bufman.purge();

#if ENABLE_MEMS
  if (state.check(STATE_MEMS_READY)) {
    calibrateMEMS();
  }
#endif

#if GNSS == GNSS_STANDALONE
  if (!state.check(STATE_GPS_READY)) {
    if (initGPS()) {
      state.set(STATE_GPS_READY);
    }
  }
#endif

#if ENABLE_OBD
  if (!state.check(STATE_OBD_READY)) {
    timeoutsOBD = 0;
    Serial.println("J1939: Initializing...");
    if (obd.init(PROTO_J1939)) {
      Serial.println("J1939:OK");
      state.set(STATE_OBD_READY);
      j1939State.busHealthy = true;
      lastDataReceived = millis();
#if ENABLE_OLED
      oled.println("J1939 OK");
#endif
    } else {
      Serial.println("J1939:NO");
      j1939State.busHealthy = false;
    }
  }
#endif

#if STORAGE != STORAGE_NONE
  if (!state.check(STATE_STORAGE_READY)) {
    if (logger.init()) {
      state.set(STATE_STORAGE_READY);
      storageState.available = true;
    }
  }
  if (state.check(STATE_STORAGE_READY)) {
    fileid = logger.begin();
  }
#endif

  printTime();

  lastMotionTime = millis();
  state.set(STATE_WORKING);

#if ENABLE_OLED
  delay(1000);
  oled.clear();
  oled.print("DEVICE ID: ");
  oled.println(devid);
  oled.setCursor(0, 7);
  oled.print("Packets");
  oled.setCursor(80, 7);
  oled.print("KB Sent");
  oled.setFontSize(FONT_SIZE_MEDIUM);
#endif
}

void showStats()
{
  uint32_t t = millis() - teleClient.startTime;
  char buf[32];
  sprintf(buf, "%02u:%02u.%c ", t / 60000, (t % 60000) / 1000, (t % 1000) / 100 + '0');
  Serial.print("[NET] ");
  Serial.print(buf);
  Serial.print("| Packet #");
  Serial.print(teleClient.txCount);
  Serial.print(" | Out: ");
  Serial.print(teleClient.txBytes >> 10);
  Serial.print(" KB | In: ");
  Serial.print(teleClient.rxBytes);
  Serial.print(" bytes | ");
  Serial.print((unsigned int)((uint64_t)(teleClient.txBytes + teleClient.rxBytes) * 3600 / (millis() - teleClient.startTime)));
  Serial.println(" KB/h");

#if ENABLE_OLED
  oled.setCursor(0, 2);
  oled.setCursor(0, 5);
  oled.printInt(teleClient.txCount, 2);
  oled.setCursor(80, 5);
  oled.printInt(teleClient.txBytes >> 10, 3);
#endif
}

bool waitMotion(long timeout)
{
#if ENABLE_MEMS
  unsigned long t = millis();
  if (state.check(STATE_MEMS_READY)) {
    do {
      float motion = 0;
      float acc[3];
      if (!mems->read(acc)) continue;
      if (accCount == 10) {
        accCount = 0;
        accSum[0] = 0;
        accSum[1] = 0;
        accSum[2] = 0;
      }
      accSum[0] += acc[0];
      accSum[1] += acc[1];
      accSum[2] += acc[2];
      accCount++;
      for (byte i = 0; i < 3; i++) {
        float m = (acc[i] - accBias[i]);
        motion += m * m;
      }
#if ENABLE_HTTPD
      serverProcess(100);
#endif
      processBLE(100);
      if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
        Serial.println(motion);
        return true;
      }
    } while (state.check(STATE_STANDBY) && ((long)(millis() - t) < timeout || timeout == -1));
    return false;
  }
#endif
  serverProcess(timeout);
  return false;
}

void printJ1939Summary() {
    uint32_t ts = millis();
    char datBuf[256];
    int n = 0;
    
    n += snprintf(datBuf + n, sizeof(datBuf) - n, "[DAT] %s#%lu:", devid, ts);
    
    if (dataCache.rpm.valid) {
        n += snprintf(datBuf + n, sizeof(datBuf) - n, "10C:%d,", dataCache.rpm.value);
    }
    if (dataCache.speed.valid) {
        n += snprintf(datBuf + n, sizeof(datBuf) - n, "10D:%d,", dataCache.speed.value);
    }
    if (dataCache.coolantTemp.valid) {
        n += snprintf(datBuf + n, sizeof(datBuf) - n, "105:%d,", dataCache.coolantTemp.value);
    }
    if (dataCache.throttle.valid) {
        n += snprintf(datBuf + n, sizeof(datBuf) - n, "111:%d,", dataCache.throttle.value);
    }
    if (dataCache.load.valid) {
        n += snprintf(datBuf + n, sizeof(datBuf) - n, "104:%d,", dataCache.load.value);
    }
    if (dataCache.fuelLevel.valid) {
        n += snprintf(datBuf + n, sizeof(datBuf) - n, "12F:%d,", dataCache.fuelLevel.value);
    }
    if (batteryVoltage > 0) {
        n += snprintf(datBuf + n, sizeof(datBuf) - n, "24:%.1f,", batteryVoltage);
    }
    if (dataCache.intakeTemp.valid) {
        n += snprintf(datBuf + n, sizeof(datBuf) - n, "10F:%d,", dataCache.intakeTemp.value);
    }
    if (dataCache.maf.valid) { 
        n += snprintf(datBuf + n, sizeof(datBuf) - n, "110:%d,", dataCache.maf.value);
    }
    if (dataCache.timing.valid) {
        n += snprintf(datBuf + n, sizeof(datBuf) - n, "10E:%d,", dataCache.timing.value);
    }
    if (dataCache.manifoldPressure.valid) {
        n += snprintf(datBuf + n, sizeof(datBuf) - n, "10B:%d,", dataCache.manifoldPressure.value);
    }
    
    // Add MEMS data if available
#if ENABLE_MEMS
    if (accCount > 0) {
        float ax = accSum[0] / accCount - accBias[0];
        float ay = accSum[1] / accCount - accBias[1];
        float az = accSum[2] / accCount - accBias[2];
        n += snprintf(datBuf + n, sizeof(datBuf) - n, "20:%.2f;%.2f;%.2f,", ax, ay, az);
    }
#endif
    // --- GNSS/GPS Data ---
    if (gd && gd->ts) {
        float kph = gd->speed * 1.852f; 
        n += snprintf(datBuf + n, sizeof(datBuf) - n, "A:%.6f,B:%.6f,C:%d,D:%d,F:%d,", 
            gd->lat, 
            gd->lng, 
            (int)gd->alt, 
            (int)kph, 
            gd->sat
        );
    }

    // Add device temp
    n += snprintf(datBuf + n, sizeof(datBuf) - n, "82:%d", deviceTemp);
    
    // Calculate simple checksum
    uint8_t checksum = 0;
    for (int i = 0; i < n; i++) {
        checksum ^= datBuf[i];
    }
    snprintf(datBuf + n, sizeof(datBuf) - n, "*%02X", checksum);
    
    Serial.println(datBuf);
}

/*******************************************************************************
  Collecting and processing data - Modified for Round Robin
*******************************************************************************/
void process()
{
    static uint32_t lastCollectTime = 0;
    static uint32_t lastReportTime = 0;
    static CBuffer* collectBuffer = nullptr;  // เก็บ buffer สำหรับรอบนี้
    
    uint32_t startTime = millis();
    uint32_t currentTime = millis();

    // ============================================================
    // STEP 1: เก็บข้อมูล J1939 ทุกครั้ง
    // ============================================================
#if ENABLE_OBD
    if (state.check(STATE_OBD_READY)) {
        processJ1939();
        
        if (!j1939State.busHealthy && !attemptJ1939Recovery()) {
            state.clear(STATE_OBD_READY | STATE_WORKING);
            return;
        }
    } else {
        if (obd.init(PROTO_J1939)) {
            state.set(STATE_OBD_READY);
            j1939State.busHealthy = true;
        }
    }
#endif

    // ============================================================
    // STEP 2: ทุก 800ms → สร้าง buffer ใหม่, เก็บ GPS และเปลี่ยน PGN Target
    // ============================================================
    if (currentTime - lastCollectTime >= DATA_COLLECT_INTERVAL) {
        // สร้าง Buffer ใหม่
        collectBuffer = bufman.getFree();
        collectBuffer->state = BUFFER_STATE_FILLING;
        collectBuffer->timestamp = millis();
        
        // อัพเดท GPS เข้า Buffer (จะ update global gd และเพิ่มลง buffer)
        processGPS(collectBuffer);
        
        // เปลี่ยน PGN เป้าหมาย (Round Robin)
        currentPGNIndex++;
        if (currentPGNIndex >= NUM_PGNS) {
            currentPGNIndex = 0;
        }
        
        lastCollectTime = currentTime;
    }

    // ============================================================
    // STEP 3: ทุก 8000ms → ส่งข้อมูลที่เก็บไว้
    // ============================================================
    if (currentTime - lastReportTime >= DATA_REPORT_INTERVAL && collectBuffer != nullptr) {
        
        // อัพเดท device temp
        if (!state.check(STATE_MEMS_READY)) deviceTemp = readChipTemperature();
        collectBuffer->add(PID_DEVICE_TEMP, ELEMENT_INT32, &deviceTemp, sizeof(deviceTemp));
        
        // MEMS data
#if ENABLE_MEMS
        processMEMS(collectBuffer);
#endif
        
        // พิมพ์ DAT Summary (gd ถูก update แล้วจาก STEP 2)
        printJ1939Summary();
        
        // เพิ่มข้อมูล J1939 ทั้งหมดลง Buffer
        if (dataCache.rpm.valid) 
            collectBuffer->add(PID_RPM | 0x100, ELEMENT_INT32, &dataCache.rpm.value, sizeof(int));
        if (dataCache.speed.valid) 
            collectBuffer->add(PID_SPEED | 0x100, ELEMENT_INT32, &dataCache.speed.value, sizeof(int));
        if (dataCache.coolantTemp.valid) 
            collectBuffer->add(PID_COOLANT_TEMP | 0x100, ELEMENT_INT32, &dataCache.coolantTemp.value, sizeof(int));
        if (dataCache.throttle.valid) 
            collectBuffer->add(PID_THROTTLE | 0x100, ELEMENT_INT32, &dataCache.throttle.value, sizeof(int));
        if (dataCache.load.valid) 
            collectBuffer->add(PID_ENGINE_LOAD | 0x100, ELEMENT_INT32, &dataCache.load.value, sizeof(int));
        if (dataCache.fuelLevel.valid) 
            collectBuffer->add(PID_FUEL_LEVEL | 0x100, ELEMENT_INT32, &dataCache.fuelLevel.value, sizeof(int));
        if (batteryVoltage > 0) 
            collectBuffer->add(PID_BATTERY_VOLTAGE, ELEMENT_FLOAT_D1, &batteryVoltage, sizeof(float));
        if (dataCache.intakeTemp.valid) 
            collectBuffer->add(PID_INTAKE_TEMP | 0x100, ELEMENT_INT32, &dataCache.intakeTemp.value, sizeof(int));
        if (dataCache.maf.valid) 
            collectBuffer->add(PID_MAF_FLOW | 0x100, ELEMENT_INT32, &dataCache.maf.value, sizeof(int));
        if (dataCache.timing.valid) 
            collectBuffer->add(PID_TIMING_ADVANCE | 0x100, ELEMENT_INT32, &dataCache.timing.value, sizeof(int));
        if (dataCache.manifoldPressure.valid) 
            collectBuffer->add(PID_INTAKE_MAP | 0x100, ELEMENT_INT32, &dataCache.manifoldPressure.value, sizeof(int));

        collectBuffer->state = BUFFER_STATE_FILLED;

        // บันทึก/ส่ง
#if STORAGE != STORAGE_NONE
        if (state.check(STATE_STORAGE_READY)) {
            writeToStorageWithErrorHandling(collectBuffer);
        }
#endif

        // ล้าง buffer pointer
        collectBuffer = nullptr;
        lastReportTime = currentTime;
    }

    // ============================================================
    // STEP 4: Watchdog และ BLE processing
    // ============================================================
    watchdogCheck();
    long t = LOOP_INTERVAL - (millis() - startTime);
    if (t > 0) {
        processBLE(t);
    } else {
        processBLE(0);
    }
}

bool initCell(bool quick = false)
{
  Serial.println("[CELL] Activating...");
  if (!teleClient.cell.begin(&sys)) {
    Serial.println("[CELL] No supported module");
#if ENABLE_OLED
    oled.println("No Cell Module");
#endif
    return false;
  }
  if (quick) return true;
#if ENABLE_OLED
    oled.print(teleClient.cell.deviceName());
    oled.println(" OK\r");
    oled.print("IMEI:");
    oled.println(teleClient.cell.IMEI);
#endif
  Serial.print("CELL:");
  Serial.println(teleClient.cell.deviceName());
  if (!teleClient.cell.checkSIM(SIM_CARD_PIN)) {
    Serial.println("NO SIM CARD");
  }
  Serial.print("IMEI:");
  Serial.println(teleClient.cell.IMEI);
  Serial.println("[CELL] Searching...");
  if (*apn) {
    Serial.print("APN:");
    Serial.println(apn);
  }
  if (teleClient.cell.setup(apn, APN_USERNAME, APN_PASSWORD)) {
    netop = teleClient.cell.getOperatorName();
    if (netop.length()) {
      Serial.print("Operator:");
      Serial.println(netop);
#if ENABLE_OLED
      oled.println(netop);
#endif
    }

#if GNSS == GNSS_CELLULAR
    if (teleClient.cell.setGPS(true)) {
      Serial.println("CELL GNSS:OK");
    }
#endif

    ip = teleClient.cell.getIP();
    if (ip.length()) {
      Serial.print("[CELL] IP:");
      Serial.println(ip);
#if ENABLE_OLED
      oled.print("IP:");
      oled.println(ip);
#endif
    }
    state.set(STATE_CELL_CONNECTED);
  } else {
    char *p = strstr(teleClient.cell.getBuffer(), "+CPSI:");
    if (p) {
      char *q = strchr(p, '\r');
      if (q) *q = 0;
      Serial.print("[CELL] ");
      Serial.println(p + 7);
#if ENABLE_OLED
      oled.println(p + 7);
#endif
    } else {
      Serial.print(teleClient.cell.getBuffer());
    }
  }
  timeoutsNet = 0;
  return state.check(STATE_CELL_CONNECTED);
}

/*******************************************************************************
  Initializing network, maintaining connection and doing transmissions
*******************************************************************************/
void telemetry(void* inst)
{
  uint32_t lastRssiTime = 0;
  uint8_t connErrors = 0;
  CStorageRAM store;
  store.init(
#if BOARD_HAS_PSRAM
    (char*)heap_caps_malloc(SERIALIZE_BUFFER_SIZE, MALLOC_CAP_SPIRAM),
#else
    (char*)malloc(SERIALIZE_BUFFER_SIZE),
#endif
    SERIALIZE_BUFFER_SIZE
  );
  teleClient.reset();

  for (;;) {
    if (state.check(STATE_STANDBY)) {
      if (state.check(STATE_CELL_CONNECTED) || state.check(STATE_WIFI_CONNECTED)) {
        teleClient.shutdown();
        netop = "";
        ip = "";
        rssi = 0;
      }
      state.clear(STATE_NET_READY | STATE_CELL_CONNECTED | STATE_WIFI_CONNECTED);
      teleClient.reset();
      bufman.purge();

      uint32_t t = millis();
      do {
        delay(1000);
      } while (state.check(STATE_STANDBY) && millis() - t < 1000L * PING_BACK_INTERVAL);
      if (state.check(STATE_STANDBY)) {
#if ENABLE_WIFI
        if (wifiSSID[0]) { 
          Serial.print("[WIFI] Joining SSID:");
          Serial.println(wifiSSID);
          teleClient.wifi.begin(wifiSSID, wifiPassword);
        }
        if (teleClient.wifi.setup()) {
          Serial.println("[WIFI] Ping...");
          teleClient.ping();
        }
        else
#endif
        {
          if (initCell()) {
            Serial.println("[CELL] Ping...");
            teleClient.ping();
          }
        }
        teleClient.shutdown();
        state.clear(STATE_CELL_CONNECTED | STATE_WIFI_CONNECTED);
      }
      continue;
    }

#if ENABLE_WIFI
    if (wifiSSID[0] && !state.check(STATE_WIFI_CONNECTED)) {
      Serial.print("[WIFI] Joining SSID:");
      Serial.println(wifiSSID);
      teleClient.wifi.begin(wifiSSID, wifiPassword);
      teleClient.wifi.setup();
    }
#endif

    while (state.check(STATE_WORKING)) {
#if ENABLE_WIFI
      if (wifiSSID[0]) {
        if (!state.check(STATE_WIFI_CONNECTED) && teleClient.wifi.connected()) {
          ip = teleClient.wifi.getIP();
          if (ip.length()) {
            Serial.print("[WIFI] IP:");
            Serial.println(ip);
          }
          connErrors = 0;
          if (teleClient.connect()) {
            state.set(STATE_WIFI_CONNECTED | STATE_NET_READY);
            netState.connected = true;
            beep(50);
            if (state.check(STATE_CELL_CONNECTED)) {
              teleClient.cell.end();
              state.clear(STATE_CELL_CONNECTED);
              Serial.println("[CELL] Deactivated");
            }
          }
        } else if (state.check(STATE_WIFI_CONNECTED) && !teleClient.wifi.connected()) {
          Serial.println("[WIFI] Disconnected");
          state.clear(STATE_WIFI_CONNECTED);
          netState.connected = false;
        }
      }
#endif
      if (!state.check(STATE_WIFI_CONNECTED) && !state.check(STATE_CELL_CONNECTED)) {
        connErrors = 0;
        if (!initCell() || !teleClient.connect()) {
          teleClient.cell.end();
          state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
          netState.connected = false;
          Serial.println("[CELL] Deactivated");
          delay(60000 * 3);
          break;
        }
        Serial.println("[CELL] In service");
        state.set(STATE_NET_READY);
        netState.connected = true;
        beep(50);
      }

      if (millis() - lastRssiTime > SIGNAL_CHECK_INTERVAL * 1000) {
#if ENABLE_WIFI
        if (state.check(STATE_WIFI_CONNECTED))
        {
          rssi = teleClient.wifi.RSSI();
        }
        else
#endif
        {
          rssi = teleClient.cell.RSSI();
        }
        if (rssi) {
          Serial.print("RSSI:");
          Serial.print(rssi);
          Serial.println("dBm");
        }
        lastRssiTime = millis();

#if ENABLE_WIFI
        if (wifiSSID[0] && !state.check(STATE_WIFI_CONNECTED)) {
          teleClient.wifi.begin(wifiSSID, wifiPassword);
        }
#endif
      }

      CBuffer* buffer = bufman.getNewest();
      if (!buffer) {
        delay(50);
        continue;
      }
#if SERVER_PROTOCOL == PROTOCOL_UDP
      store.header(devid);
#endif
      store.timestamp(buffer->timestamp);
      buffer->serialize(store);
      bufman.free(buffer);
      store.tailer();
      // Serial.print("[DAT] ");
      // Serial.println(store.buffer());

#ifdef PIN_LED
      if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
#endif

      if (transmitWithRetry(store.buffer(), store.length())) {
        connErrors = 0;
        showStats();
      } else {
        timeoutsNet++;
        connErrors++;
        printTimeoutStats();
        if (connErrors < MAX_CONN_ERRORS_RECONNECT) {
          teleClient.connect(true);
        }
      }
#ifdef PIN_LED
      if (ledMode == 0) digitalWrite(PIN_LED, LOW);
#endif
      store.purge();

      teleClient.inbound();

      if (state.check(STATE_CELL_CONNECTED) && !teleClient.cell.check(1000)) {
        Serial.println("[CELL] Not in service");
        state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
        netState.connected = false;
        break;
      }

      if (syncInterval > 10000 && millis() - teleClient.lastSyncTime > syncInterval) {
        Serial.println("[NET] Poor connection");
        timeoutsNet++;
        if (!teleClient.connect()) {
          connErrors++;
        }
      }

      if (connErrors >= MAX_CONN_ERRORS_RECONNECT) {
#if ENABLE_WIFI
        if (state.check(STATE_WIFI_CONNECTED)) {
          teleClient.wifi.end();
          state.clear(STATE_NET_READY | STATE_WIFI_CONNECTED);
          netState.connected = false;
          break;
        }
#endif
        if (state.check(STATE_CELL_CONNECTED)) {
          teleClient.cell.end();
          state.clear(STATE_NET_READY | STATE_CELL_CONNECTED);
          netState.connected = false;
          break;
        }
      }

      if (deviceTemp >= COOLING_DOWN_TEMP) {
        Serial.print("HIGH DEVICE TEMP: ");
        Serial.println(deviceTemp);
        bufman.purge();
      }
    }
  }
}

/*******************************************************************************
  Implementing stand-by mode
*******************************************************************************/
void standby()
{
  state.set(STATE_STANDBY);
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    logger.end();
  }
#endif

#if !GNSS_ALWAYS_ON && GNSS == GNSS_STANDALONE
  if (state.check(STATE_GPS_READY)) {
    Serial.println("[GNSS] OFF");
    sys.gpsEnd(true);
    state.clear(STATE_GPS_READY | STATE_GPS_ONLINE);
    gd = 0;
  }
#endif

  state.clear(STATE_WORKING | STATE_OBD_READY | STATE_STORAGE_READY);
#if ENABLE_OLED
  oled.print("STANDBY");
  delay(1000);
  oled.clear();
#endif
  Serial.println("STANDBY");
  obd.enterLowPowerMode();
#if ENABLE_MEMS
  calibrateMEMS();
  waitMotion(-1);
#elif ENABLE_OBD
  do {
    delay(5000);
  } while (obd.getVoltage() < JUMPSTART_VOLTAGE);
#else
  delay(5000);
#endif
  Serial.println("WAKEUP");
  sys.resetLink();
#if RESET_AFTER_WAKEUP
#if ENABLE_MEMS
  if (mems) mems->end();  
#endif
  ESP.restart();
#endif  
  state.clear(STATE_STANDBY);
}

void genDeviceID(char* buf)
{
    uint64_t seed = ESP.getEfuseMac() >> 8;
    for (int i = 0; i < 8; i++, seed >>= 5) {
      byte x = (byte)seed & 0x1f;
      if (x >= 10) {
        x = x - 10 + 'A';
        switch (x) {
          case 'B': x = 'W'; break;
          case 'D': x = 'X'; break;
          case 'I': x = 'Y'; break;
          case 'O': x = 'Z'; break;
        }
      } else {
        x += '0';
      }
      buf[i] = x;
    }
    buf[8] = 0;
}

void showSysInfo()
{
  Serial.print("CPU:");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.print("MHz FLASH:");
  Serial.print(ESP.getFlashChipSize() >> 20);
  Serial.println("MB");
  Serial.print("IRAM:");
  Serial.print(ESP.getHeapSize() >> 10);
  Serial.print("KB");
#if BOARD_HAS_PSRAM
  if (psramInit()) {
    Serial.print(" PSRAM:");
    Serial.print(esp_spiram_get_size() >> 20);
    Serial.print("MB");
  }
#endif
  Serial.println();

  int rtc = rtc_clk_slow_freq_get();
  if (rtc) {
    Serial.print("RTC:");
    Serial.println(rtc);
  }

#if ENABLE_OLED
  oled.clear();
  oled.print("CPU:");
  oled.print(ESP.getCpuFreqMHz());
  oled.print("Mhz ");
  oled.print(getFlashSize() >> 10);
  oled.println("MB Flash");
#endif

  Serial.print("DEVICE ID:");
  Serial.println(devid);
#if ENABLE_OLED
  oled.print("DEVICE ID:");
  oled.println(devid);
#endif
}

void loadConfig()
{
  size_t len;
  len = sizeof(apn);
  apn[0] = 0;
  nvs_get_str(nvs, "CELL_APN", apn, &len);
  if (!apn[0]) {
    strcpy(apn, CELL_APN);
  }

#if ENABLE_WIFI
  len = sizeof(wifiSSID);
  nvs_get_str(nvs, "WIFI_SSID", wifiSSID, &len);
  len = sizeof(wifiPassword);
  nvs_get_str(nvs, "WIFI_PWD", wifiPassword, &len);
#endif
}

void processBLE(int timeout)
{
#if ENABLE_BLE
  static byte echo = 0;
  char* cmd;
  if (!(cmd = ble_recv_command(timeout))) {
    return;
  }

  char *p = strchr(cmd, '\r');
  if (p) *p = 0;
  char buf[48];
  int bufsize = sizeof(buf);
  int n = 0;
  if (echo) n += snprintf(buf + n, bufsize - n, "%s\r", cmd);
  Serial.print("[BLE] ");
  Serial.print(cmd);
  if (!strcmp(cmd, "UPTIME") || !strcmp(cmd, "TICK")) {
    n += snprintf(buf + n, bufsize - n, "%lu", millis());
  } else if (!strcmp(cmd, "HEALTH")) {
    updateHealthMetrics();
    n += snprintf(buf + n, bufsize - n, "%.1f%%", health.successRate);
  } else if (!strcmp(cmd, "BATT")) {
    n += snprintf(buf + n, bufsize - n, "%.2f", batteryVoltage);
  } else if (!strcmp(cmd, "RESET")) {
#if STORAGE
    logger.end();
#endif
    ESP.restart();
  } else if (!strcmp(cmd, "OFF")) {
    state.set(STATE_STANDBY);
    state.clear(STATE_WORKING);
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "ON")) {
    state.clear(STATE_STANDBY);
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "ON?")) {
    n += snprintf(buf + n, bufsize - n, "%u", state.check(STATE_STANDBY) ? 0 : 1);
  } else if (!strcmp(cmd, "APN?")) {
    n += snprintf(buf + n, bufsize - n, "%s", *apn ? apn : "DEFAULT");
  } else if (!strncmp(cmd, "APN=", 4)) {
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "CELL_APN", strcmp(cmd + 4, "DEFAULT") ? cmd + 4 : "") == ESP_OK ? "OK" : "ERR");
    loadConfig();
  } else if (!strcmp(cmd, "NET_OP")) {
    if (state.check(STATE_WIFI_CONNECTED)) {
#if ENABLE_WIFI
      n += snprintf(buf + n, bufsize - n, "%s", wifiSSID[0] ? wifiSSID : "-");
#endif
    } else {
      snprintf(buf + n, bufsize - n, "%s", netop.length() ? netop.c_str() : "-");
      char *p = strchr(buf + n, ' ');
      if (p) *p = 0;
      n += strlen(buf + n);
    }
  } else if (!strcmp(cmd, "NET_IP")) {
    n += snprintf(buf + n, bufsize - n, "%s", ip.length() ? ip.c_str() : "-");
  } else if (!strcmp(cmd, "NET_PACKET")) {
      n += snprintf(buf + n, bufsize - n, "%u", teleClient.txCount);
  } else if (!strcmp(cmd, "NET_DATA")) {
      n += snprintf(buf + n, bufsize - n, "%u", teleClient.txBytes);
  } else if (!strcmp(cmd, "NET_RATE")) {
      n += snprintf(buf + n, bufsize - n, "%u", teleClient.startTime ? (unsigned int)((uint64_t)(teleClient.txBytes + teleClient.rxBytes) * 3600 / (millis() - teleClient.startTime)) : 0);
  } else if (!strcmp(cmd, "RSSI")) {
    n += snprintf(buf + n, bufsize - n, "%d", rssi);
#if ENABLE_WIFI
  } else if (!strcmp(cmd, "SSID?")) {
    n += snprintf(buf + n, bufsize - n, "%s", wifiSSID[0] ? wifiSSID : "-");
  } else if (!strncmp(cmd, "SSID=", 5)) {
    const char* p = cmd + 5;
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "WIFI_SSID", strcmp(p, "-") ? p : "") == ESP_OK ? "OK" : "ERR");
    loadConfig();
  } else if (!strcmp(cmd, "WPWD?")) {
    n += snprintf(buf + n, bufsize - n, "%s", wifiPassword[0] ? wifiPassword : "-");
  } else if (!strncmp(cmd, "WPWD=", 5)) {
    const char* p = cmd + 5;
    n += snprintf(buf + n, bufsize - n, nvs_set_str(nvs, "WIFI_PWD", strcmp(p, "-") ? p : "") == ESP_OK ? "OK" : "ERR");
    loadConfig();
#else
  } else if (!strcmp(cmd, "SSID?") || !strcmp(cmd, "WPWD?")) {
    n += snprintf(buf + n, bufsize - n, "-");
#endif
#if ENABLE_MEMS
  } else if (!strcmp(cmd, "TEMP")) {
    n += snprintf(buf + n, bufsize - n, "%d", (int)deviceTemp);
  } else if (!strcmp(cmd, "ACC")) {
    n += snprintf(buf + n, bufsize - n, "%.1f/%.1f/%.1f", acc[0], acc[1], acc[2]);
  } else if (!strcmp(cmd, "GYRO")) {
    n += snprintf(buf + n, bufsize - n, "%.1f/%.1f/%.1f", gyr[0], gyr[1], gyr[2]);
  } else if (!strcmp(cmd, "GF")) {
    n += snprintf(buf + n, bufsize - n, "%f", (float)sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]));
#endif
  } else if (!strcmp(cmd, "ATE0")) {
    echo = 0;
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "ATE1")) {
    echo = 1;
    n += snprintf(buf + n, bufsize - n, "OK");
  } else if (!strcmp(cmd, "FS")) {
    n += snprintf(buf + n, bufsize - n, "%u",
#if STORAGE == STORAGE_NONE
    0
#else
    logger.size()
#endif
      );
  } else if (!strcmp(cmd, "RPM") && dataCache.rpm.valid) {
    n += snprintf(buf + n, bufsize - n, "%d", dataCache.rpm.value);
  } else if (!strcmp(cmd, "SPEED") && dataCache.speed.valid) {
    n += snprintf(buf + n, bufsize - n, "%d", dataCache.speed.value);
  } else if (!strcmp(cmd, "VIN")) {
    n += snprintf(buf + n, bufsize - n, "%s", vin[0] ? vin : "N/A");
  } else if (!strcmp(cmd, "LAT") && gd) {
    n += snprintf(buf + n, bufsize - n, "%f", gd->lat);
  } else if (!strcmp(cmd, "LNG") && gd) {
    n += snprintf(buf + n, bufsize - n, "%f", gd->lng);
  } else if (!strcmp(cmd, "ALT") && gd) {
    n += snprintf(buf + n, bufsize - n, "%d", (int)gd->alt);
  } else if (!strcmp(cmd, "SAT") && gd) {
    n += snprintf(buf + n, bufsize - n, "%u", (unsigned int)gd->sat);
  } else if (!strcmp(cmd, "SPD") && gd) {
    n += snprintf(buf + n, bufsize - n, "%d", (int)(gd->speed * 1852 / 1000));
  } else if (!strcmp(cmd, "CRS") && gd) {
    n += snprintf(buf + n, bufsize - n, "%u", (unsigned int)gd->heading);
  } else {
    n += snprintf(buf + n, bufsize - n, "ERROR");
  }
  Serial.print(" -> ");
  Serial.println((p = strchr(buf, '\r')) ? p + 1 : buf);
  if (n < bufsize - 1) {
    buf[n++] = '\r';
  } else {
    n = bufsize - 1;
  }
  buf[n] = 0;
  ble_send_response(buf, n, cmd);
#else
  if (timeout) delay(timeout);
#endif
}

void setup()
{
  delay(500);

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );
  err = nvs_open("storage", NVS_READWRITE, &nvs);
  if (err == ESP_OK) {
    loadConfig();
  }

Serial1.setRxBufferSize(8192);

#if ENABLE_OLED
  oled.begin();
  oled.setFontSize(FONT_SIZE_SMALL);
#endif
  Serial.begin(115200);
  

#ifdef PIN_LED
  pinMode(PIN_LED, OUTPUT);
  if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
#endif

  genDeviceID(devid);

#if CONFIG_MODE_TIMEOUT
  configMode();
#endif

#if LOG_EXT_SENSORS == 1
  pinMode(PIN_SENSOR1, INPUT);
  pinMode(PIN_SENSOR2, INPUT);
#elif LOG_EXT_SENSORS == 2
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
#endif

  showSysInfo();

  bufman.init();

#if ENABLE_OBD
  if (sys.begin()) {
    Serial.print("TYPE:");
    Serial.println(sys.devType);
    obd.begin(sys.link);
  }
#else
  sys.begin(false, true);
#endif

#if ENABLE_MEMS
if (!state.check(STATE_MEMS_READY)) do {
  Serial.print("MEMS:");
  mems = new ICM_42627;
  byte ret = mems->begin();
  if (ret) {
    state.set(STATE_MEMS_READY);
    Serial.println("ICM-42627");
    break;
  }
  delete mems;
  mems = new ICM_20948_I2C;
  ret = mems->begin();
  if (ret) {
    state.set(STATE_MEMS_READY);
    Serial.println("ICM-20948");
    break;
  } 
  delete mems;
  mems = 0;
  Serial.println("NO");
} while (0);
#endif

#if ENABLE_HTTPD
  IPAddress ip;
  if (serverSetup(ip)) {
    Serial.println("HTTPD:");
    Serial.println(ip);
#if ENABLE_OLED
    oled.println(ip);
#endif
  } else {
    Serial.println("HTTPD:NO");
  }
#endif

  state.set(STATE_WORKING);

#if ENABLE_BLE
  ble_init("FreematicsPlus");
#endif

  initialize();

  subtask.create(telemetry, "telemetry", 2, 8192);

#ifdef PIN_LED
  digitalWrite(PIN_LED, LOW);
#endif

  Serial.println("=== J1939 Data Logger Ready ===");
}

void loop()
{
  if (!state.check(STATE_WORKING)) {
    standby();
#ifdef PIN_LED
    if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
#endif
    initialize();
#ifdef PIN_LED
    digitalWrite(PIN_LED, LOW);
#endif
    return;
  }

  process();
}
