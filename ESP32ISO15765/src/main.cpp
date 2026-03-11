#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <CAN.h>

const char *ssid = "";
const char *password = "";

const char *serverUrl = "";
const String deviceID = "ESP32CANBUSTESTOBD";

#define CAN_TX_PIN 26
#define CAN_RX_PIN 27

float getOBDValue(int pid);
void sendAllDataToTraccar(float rpm, float speed, float load, float coolant, float map, float iat, float maf, float throttle, float fuel, float volt, float ignition);

float val_rpm = 0;
float val_speed = 0;
float val_load = 0;
float val_coolant = 0;
float val_map = 0;
float val_iat = 0;
float val_maf = 0;
float val_throttle = 0;
float val_fuel = 0;
float val_volt = 0;
float val_ignition = 0;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- ArtronShop ESP-OBD2 Monitor ---");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  // ตั้ง Timeout สำหรับ WiFi 10 วินาที (20 รอบ * 500ms)
  int wifi_timeout = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_timeout < 20)
  {
    delay(500);
    Serial.print(".");
    wifi_timeout++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi Connected");
  }
  else
  {
    Serial.println("\nWiFi Timeout. Running in Offline Mode (Serial Monitor Only).");
  }

  CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);
  if (!CAN.begin(500E3))
  { // 500E3 คือ 500kbps
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
  Serial.println("CAN Bus Started at 500kbps (11-bit)");
}

void loop()
{
  Serial.println("================================================");
  Serial.println("             OBD LIVE DATA             ");
  Serial.println("================================================");

  // อ่านค่า (PID ตามมาตรฐาน SAE J1979)
  val_rpm = getOBDValue(0x0C);
  delay(10);
  val_speed = getOBDValue(0x0D);
  delay(10);
  val_load = getOBDValue(0x04);
  delay(10);
  val_throttle = getOBDValue(0x11);
  delay(10);
  val_coolant = getOBDValue(0x05);
  delay(10);
  val_iat = getOBDValue(0x0F);
  delay(10);
  val_map = getOBDValue(0x0B);
  delay(10);
  val_maf = getOBDValue(0x10);
  delay(10);
  val_fuel = getOBDValue(0x2F);
  delay(10);
  val_volt = getOBDValue(0x42);
  delay(10);
  val_ignition = getOBDValue(0x0E);
  delay(10);

  Serial.printf("Voltage:                         %.1f V\n", val_volt);
  Serial.printf("Vehicle Speed Sensor:            %.0f Km/h\n", val_speed);
  Serial.printf("Engine RPM:                      %.0f Rpm\n", val_rpm);
  Serial.printf("Engine Coolant Temperature:      %.0f C\n", val_coolant);
  Serial.printf("Intake Air Temperature:          %.0f C\n", val_iat);
  Serial.printf("Air Flow Rate (MAF):             %.2f g/s\n", val_maf);
  Serial.printf("Intake Manifold Pressure (MAP):  %.0f Kpa\n", val_map);
  Serial.printf("Absolute Throttle Position:      %.0f %%\n", val_throttle);
  Serial.printf("Ignition Timing Advance:         %.0f °BTDC\n", val_ignition);
  Serial.printf("Calculated LOAD Value:           %.0f %%\n", val_load);
  Serial.printf("Fuel Level Input:                %.0f %%\n", val_fuel);

  Serial.println("------------------------------------------------");

  if (WiFi.status() == WL_CONNECTED)
  {
    sendAllDataToTraccar(val_rpm, val_speed, val_load, val_coolant, val_map, val_iat, val_maf, val_throttle, val_fuel, val_volt, val_ignition);
  }
  else
  {
    Serial.println("Offline Mode: Data not sent to server.");
  }

  delay(5000);
}

float getOBDValue(int pid)
{
  // สำหรับ 11-bit ใช้ CAN.beginPacket
  CAN.beginPacket(0x7DF); // 0x7DF คือ Broadcast ID มาตรฐาน 11-bit
  CAN.write(0x02);
  CAN.write(0x01);
  CAN.write(pid);
  CAN.write(0x55);
  CAN.write(0x55);
  CAN.write(0x55);
  CAN.write(0x55);
  CAN.write(0x55);
  CAN.endPacket();

  unsigned long start = millis();
  while (CAN.parsePacket() == 0)
  {
    if (millis() - start > 100)
      return 0;
  }

  long id = CAN.packetId();
  if (id >= 0x7E8 && id <= 0x7EF)
  {
    int len = CAN.read();
    int mode = CAN.read();
    int retPid = CAN.read();
    if (retPid == pid)
    {
      int A = CAN.read();
      int B = CAN.read();

      switch (pid)
      {
      case 0x0C:
        return ((A * 256.0) + B) / 4.0;
      case 0x0D:
        return A;
      case 0x04:
        return (A * 100.0) / 255.0;
      case 0x05:
        return A - 40.0;
      case 0x0B:
        return A;
      case 0x0F:
        return A - 40.0;
      case 0x10:
        return ((A * 256.0) + B) / 100.0;
      case 0x11:
        return (A * 100.0) / 255.0;
      case 0x2F:
        return (A * 100.0) / 255.0;
      case 0x42:
        return ((A * 256.0) + B) / 1000.0;
      case 0x0E:
        return (A - 128.0);
      default:
        return 0;
      }
    }
  }
  else
  {
    while (CAN.available())
      CAN.read();
  }
  return 0;
}

void sendAllDataToTraccar(float rpm, float speed, float load, float coolant, float map, float iat, float maf, float throttle, float fuel, float volt, float ignition)
{
  HTTPClient http;

  String url = String(serverUrl) + "/?id=" + deviceID + "&lat=13.7563&lon=100.5018" + "&speed=" + String(speed) + "&rpm=" + String(rpm) + "&engineLoad=" + String(load) + "&coolantTemp=" + String(coolant) + "&map=" + String(map) + "&intakeTemp=" + String(iat) + "&maf=" + String(maf) + "&throttle=" + String(throttle) + "&fuel=" + String(fuel) + "&voltage=" + String(volt) + "&ignition=" + String(ignition);

  http.begin(url);
  http.setReuse(false);
  http.setTimeout(5000);

  Serial.println(">> Sending to Traccar: " + url);

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0)
  {
    Serial.printf(">> Upload Status: Success (%d OK)\n", httpResponseCode);
  }
  else
  {
    Serial.printf(">> Upload Error: %d (Try checking Server/Firewall)\n", httpResponseCode);
  }

  http.end();
}

