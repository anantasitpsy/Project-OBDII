#ifndef CELL_MQTT_H
#define CELL_MQTT_H

/******************************************************************************
 * CellMQTT.h — MQTT layer สำหรับ Freematics ONE+
 *
 * MQTTPacket  — standalone encoder (ไม่ inherit ใคร)
 * CellMQTT    — extends CellSIMCOM เพียงอย่างเดียว
 * WifiMQTT    — extends ClientWIFI เพียงอย่างเดียว, ใช้ MQTTPacket
 *
 * ทำไม:
 *   CellSIMCOM และ ClientWIFI มี begin/setup/end/getIP/RSSI signature ต่างกัน
 *   ถ้า WifiMQTT inherit ทั้งสอง → ambiguity error ตอน compile
 *   แก้โดยแยก MQTT encode logic ออกเป็น MQTTPacket แทน
 ******************************************************************************/

#include <FreematicsPlus.h>
#include "config.h"    // ต้องมาก่อน #if ENABLE_WIFI

#define MQTT_CONNECT     0x10
#define MQTT_CONNACK     0x20
#define MQTT_PUBLISH     0x30
#define MQTT_SUBSCRIBE   0x82
#define MQTT_PINGREQ     0xC0
#define MQTT_PINGRESP    0xD0
#define MQTT_DISCONNECT  0xE0

#define MQTT_KEEPALIVE   120     // seconds — ต้องมากกว่า DATA_INTERVAL สูงสุด
#define MQTT_TIMEOUT     8000    // ms รอ CONNACK
#define MQTT_PKT_BUF     640     // bytes

// ============================================================
// MQTTPacket — MQTT 3.1.1 packet builder (ไม่ inherit ใคร)
// ทั้ง CellMQTT และ WifiMQTT ใช้ร่วมกันผ่าน composition
// ============================================================
class MQTTPacket
{
public:
    uint16_t buildConnect   (uint8_t* buf, const char* clientId,
                             const char* user, const char* pass,
                             const char* willTopic, const char* willMsg);
    uint16_t buildPublish   (uint8_t* buf, const char* topic,
                             const uint8_t* payload, uint16_t len);
    uint16_t buildSubscribe (uint8_t* buf, const char* topic, uint16_t packetId);
    uint16_t buildPingReq   (uint8_t* buf);
    uint16_t buildDisconnect(uint8_t* buf);

private:
    uint8_t  encodeLength(uint8_t* buf, uint32_t len);
    uint16_t writeStr    (uint8_t* buf, const char* str);
};

// ============================================================
// CellMQTT — extends CellSIMCOM เพียงอย่างเดียว
// .cell ใน TeleClientMQTT → begin/setup/end/RSSI/getIP ครบ
// ============================================================
class CellMQTT : public CellSIMCOM
{
public:
    bool mqttConnect   (const char* host, uint16_t port,
                        const char* clientId,
                        const char* user      = nullptr,
                        const char* pass      = nullptr,
                        const char* willTopic = nullptr,
                        const char* willMsg   = nullptr);
    void mqttDisconnect();
    bool mqttPublish   (const char* topic, const uint8_t* payload, uint16_t len);
    bool mqttPublish   (const char* topic, const char* payload) {
        return mqttPublish(topic, (const uint8_t*)payload, (uint16_t)strlen(payload));
    }
    bool mqttSubscribe (const char* topic);
    bool mqttLoop      ();
    bool mqttConnected () { return m_mqttConn; }

    virtual void onMessage(const char* topic, const uint8_t* payload, uint16_t len) {}

    uint32_t lastPingTime = 0;

private:
    bool tcpOpen (const char* host, uint16_t port);
    bool tcpClose();
    bool tcpSend (const uint8_t* data, uint16_t len);
    int  tcpRecv (uint8_t* buf, uint16_t maxlen, uint16_t timeout_ms = 2000);

    MQTTPacket m_pkt;
    uint8_t    m_buf[MQTT_PKT_BUF];
    bool       m_mqttConn = false;
    uint16_t   m_packetId = 1;
};

// ============================================================
// WifiMQTT — extends ClientWIFI เพียงอย่างเดียว
// .wifi ใน TeleClientMQTT → begin/setup/end/connected/getIP/RSSI ครบ
// ============================================================
#if ENABLE_WIFI
#include <WiFi.h>

class WifiMQTT : public ClientWIFI
{
public:
    bool mqttConnect   (const char* host, uint16_t port,
                        const char* clientId,
                        const char* user      = nullptr,
                        const char* pass      = nullptr,
                        const char* willTopic = nullptr,
                        const char* willMsg   = nullptr);
    void mqttDisconnect();
    bool mqttPublish   (const char* topic, const uint8_t* payload, uint16_t len);
    bool mqttPublish   (const char* topic, const char* payload) {
        return mqttPublish(topic, (const uint8_t*)payload, (uint16_t)strlen(payload));
    }
    bool mqttSubscribe (const char* topic);
    bool mqttLoop      ();
    bool mqttConnected () { return _conn && _client.connected(); }

    virtual void onMessage(const char* topic, const uint8_t* payload, uint16_t len) {}

private:
    bool rawSend(const uint8_t* data, uint16_t len);
    int  rawRecv(uint8_t* buf, uint16_t maxlen, uint16_t timeout_ms = 2000);

    WiFiClient _client;
    MQTTPacket _pkt;
    uint8_t    _buf[MQTT_PKT_BUF];
    bool       _conn = false;
    uint32_t   _lastPing = 0;
    uint16_t   _pktId = 1;
};
#endif // ENABLE_WIFI

#endif // CELL_MQTT_H
