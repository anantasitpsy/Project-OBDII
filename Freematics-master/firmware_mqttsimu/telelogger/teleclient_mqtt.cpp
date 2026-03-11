/******************************************************************************
 * teleclient_mqtt.cpp
 ******************************************************************************/
#include <FreematicsPlus.h>
#include "telestore.h"
#include "teleclient_mqtt.h"
#include "config.h"

extern int16_t rssi;
extern char devid[];
extern char vin[];
extern GPS_DATA* gd;

void TeleClientMQTT::initTopics()
{
    snprintf(topicData,   sizeof(topicData),   "%s/%s/data",   MQTT_TOPIC_PREFIX, devid);
    snprintf(topicStatus, sizeof(topicStatus), "%s/%s/status", MQTT_TOPIC_PREFIX, devid);
    snprintf(topicCmd,    sizeof(topicCmd),    "%s/%s/cmd",    MQTT_TOPIC_PREFIX, devid);
    snprintf(clientId,    sizeof(clientId),    "fmatics_%s",   devid);
    Serial.println("==============================");
    Serial.print("[MQTT] SUB topic: "); Serial.println(topicData);
    Serial.print("[MQTT] ClientID : "); Serial.println(clientId);
    Serial.println("==============================");
}

bool TeleClientMQTT::connect(bool quick)
{
    if (!initialized) { initTopics(); initialized = true; }

    char willMsg[48];
    snprintf(willMsg, sizeof(willMsg), "offline|%s", devid);

    const char* usr = strlen(MQTT_USER) ? MQTT_USER : nullptr;
    const char* pwd = strlen(MQTT_PASS) ? MQTT_PASS : nullptr;

    bool ok = false;
#if ENABLE_WIFI
    if (wifi.connected()) {
        if (wifi.mqttConnected()) {
            ok = true;
        } else {
            ok = wifi.mqttConnect(MQTT_HOST, MQTT_PORT, clientId,
                                   usr, pwd, topicStatus, willMsg);
            if (ok) wifi.mqttSubscribe(topicCmd);
        }
    }
    if (!ok)
#endif
    {
        if (cell.mqttConnected()) {
            ok = true;
        } else {
            ok = cell.mqttConnect(MQTT_HOST, MQTT_PORT, clientId,
                                   usr, pwd, topicStatus, willMsg);
            if (ok) cell.mqttSubscribe(topicCmd);
        }
    }

    if (ok && !login) {
        login = true;
        startTime = millis();
        lastSyncTime = millis();
        notify(EVENT_LOGIN);
    }
    return ok;
}

bool TeleClientMQTT::notify(byte event, const char* payload)
{
    char msg[160];
    snprintf(msg, sizeof(msg), "EV=%d|ID=%s|SSI=%d|VIN=%s",
             (int)event, devid, (int)rssi, vin);
    if (payload) {
        strncat(msg, "|",     sizeof(msg) - strlen(msg) - 1);
        strncat(msg, payload, sizeof(msg) - strlen(msg) - 1);
    }
    if (event == EVENT_LOGOUT) login = false;

    bool ok = false;
#if ENABLE_WIFI
    if (wifi.connected() && wifi.mqttConnected())
        ok = wifi.mqttPublish(topicStatus, msg);
    if (!ok)
#endif
    ok = cell.mqttPublish(topicStatus, msg);

    Serial.print("[MQTT] notify EV="); Serial.println((int)event);
    return ok;
}

bool TeleClientMQTT::transmit(const char* packetBuffer, unsigned int packetSize)
{
    bool ok = false;
#if ENABLE_WIFI
    if (wifi.connected() && wifi.mqttConnected()) {
        ok = wifi.mqttPublish(topicData,
                               (const uint8_t*)packetBuffer, (uint16_t)packetSize);
        if (ok) {
            Serial.print("[MQTT-WiFi] TX ");
            Serial.print(packetSize); Serial.println("B");
        }
    }
    if (!ok)
#endif
    {
        ok = cell.mqttPublish(topicData,
                               (const uint8_t*)packetBuffer, (uint16_t)packetSize);
        if (ok) {
            Serial.print("[MQTT-Cell] TX ");
            Serial.print(packetSize); Serial.println("B");
        }
    }

    if (ok) { txBytes += packetSize; txCount++; lastSyncTime = millis(); }
    else      Serial.println("[MQTT] TX failed");
    return ok;
}

bool TeleClientMQTT::ping()
{
    bool ok = false;
#if ENABLE_WIFI
    if (wifi.connected()) ok = wifi.mqttLoop();
    if (!ok)
#endif
    ok = cell.mqttLoop();
    if (ok) lastSyncTime = millis();
    return ok;
}

void TeleClientMQTT::inbound()
{
#if ENABLE_WIFI
    if (wifi.connected() && wifi.mqttConnected()) { wifi.mqttLoop(); return; }
#endif
    cell.mqttLoop();
}

void TeleClientMQTT::shutdown()
{
    if (login) { notify(EVENT_LOGOUT); login = false; }
#if ENABLE_WIFI
    wifi.mqttDisconnect();
#endif
    cell.mqttDisconnect();
    Serial.println("[MQTT] Shutdown");
}
