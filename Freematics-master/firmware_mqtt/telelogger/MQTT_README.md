# Freematics Telelogger — เปลี่ยนเป็น MQTT

## ทำไมถึงช้า และ MQTT แก้ยังไง?

| | HTTP/UDP (เดิม) | MQTT (ใหม่) |
|---|---|---|
| Connection | เปิด-ปิดทุก request | เปิดครั้งเดียว persistent |
| Overhead | ~200-1000 bytes header | ~2 bytes/packet |
| Latency | 200–2000 ms | **20–200 ms** |
| เหมาะกับ | Request-Response | Real-time telemetry ✅ |

---

## ไฟล์ที่เพิ่ม/แก้ไข

```
firmware_v5/telelogger/
├── teleclient_mqtt.h    ← ใหม่: MQTT client class
├── teleclient_mqtt.cpp  ← ใหม่: MQTT implementation
├── config.h             ← แก้: เพิ่ม MQTT broker settings
├── telelogger.ino       ← แก้: ใช้ TeleClientMQTT แทน HTTP
└── platformio.ini       ← แก้: เพิ่ม PubSubClient library
```

---

## การติดตั้ง

### 1. ติดตั้ง Library
Arduino IDE: Sketch → Include Library → Manage Libraries → ค้นหา **PubSubClient** by Nick O'Leary → Install

PlatformIO: เพิ่มใน `platformio.ini` (ทำแล้ว):
```ini
lib_deps =
    knolleary/PubSubClient@^2.8
```

### 2. เลือก MQTT Broker

เปิด `config.h` แล้วเลือก broker:

#### ตัวเลือกที่แนะนำ (ฟรี, ไม่ต้องสมัคร — ทดสอบ)
```cpp
#define MQTT_HOST "broker.hivemq.com"
#define MQTT_PORT 1883
#define MQTT_USER ""
#define MQTT_PASS ""
```

#### HiveMQ Cloud (แนะนำ production — ฟรีสมัคร)
1. ไปที่ https://www.hivemq.com/mqtt-cloud-broker/
2. สมัครฟรี → Get Cluster
3. เอา hostname มาใส่:
```cpp
#define MQTT_HOST "xxxxxxxx.s2.eu.hivemq.cloud"
#define MQTT_PORT 8883    // TLS
#define MQTT_USER "your_user"
#define MQTT_PASS "your_pass"
```
> ⚠️ ถ้าใช้ port 8883 (TLS) ต้องเปลี่ยน `WiFiClient` เป็น `WiFiClientSecure` ใน `teleclient_mqtt.cpp`

#### EMQX Cloud (free trial)
1. https://www.emqx.com/en/cloud → Free Trial
2. ได้ hostname → ใส่ใน config.h

---

## MQTT Topics

```
freematics/{DEVICE_ID}/data    ← ข้อมูล OBD + GPS (publish ทุก interval)
freematics/{DEVICE_ID}/status  ← login/logout/offline (retained)
freematics/{DEVICE_ID}/cmd     ← รับ command จาก dashboard (subscribe)
```

### ทดสอบด้วย MQTT Explorer
1. ดาวน์โหลด: https://mqtt-explorer.com
2. เชื่อมต่อ broker เดียวกัน
3. Subscribe `freematics/#` → เห็นข้อมูล real-time

### ทดสอบด้วย mosquitto CLI
```bash
# รับข้อมูลจาก device
mosquitto_sub -h broker.hivemq.com -t "freematics/+/data" -v

# ส่ง command ไป device
mosquitto_pub -h broker.hivemq.com -t "freematics/DEVICE_ID/cmd" -m "PING"
```

---

## ถ้าใช้ Cellular (ไม่ใช่ WiFi)

ใน `teleclient_mqtt.cpp` มีส่วน:
```cpp
#else
  // สำหรับ Cellular: ใช้ TinyGSM หรือ modem TCP wrapper
  #error "Cellular MQTT: กรุณาตั้งค่า MQTT_NETWORK_CLIENT"
```

ต้องเพิ่ม TinyGSM:
```cpp
#include <TinyGsmClient.h>
// ... init modem ...
TinyGsmClient gsmClientForMQTT(modem);
#define MQTT_NETWORK_CLIENT gsmClientForMQTT
```
และเพิ่มใน platformio.ini:
```ini
lib_deps =
    knolleary/PubSubClient@^2.8
    vshymanskyy/TinyGSM@^0.11.7
```

---

## สาเหตุ delay ที่พบบ่อย (นอกจาก protocol)

1. **DATA_INTERVAL_TABLE** ใน config.h — ค่า default คือ `{1000, 2000, 5000}` ms  
   → ลดได้ถ้า network เร็วพอ เช่น `{500, 1000, 2000}`

2. **HTTP_CONN_TIMEOUT** — HTTP ต้องรอ response ทุก packet  
   → MQTT ไม่มีปัญหานี้ (fire-and-forget QoS 0)

3. **OBD polling ช้า** — บาง PID ใช้เวลา 50-200ms ต่อครั้ง  
   → ลด PID ที่ไม่จำเป็นใน `obdData[]` ใน telelogger.ino

4. **SERIALIZE_BUFFER_SIZE** — ถ้าข้อมูลเต็ม buffer ก่อนส่ง  
   → เพิ่ม buffer หรือส่งถี่ขึ้น
