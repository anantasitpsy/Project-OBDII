#ifndef TELECLIENT_MQTT_H
#define TELECLIENT_MQTT_H

#include "teleclient.h"
#include "CellMQTT.h"

#ifndef MQTT_HOST
  #define MQTT_HOST "broker.hivemq.com"
  #define MQTT_PORT 1883
  #define MQTT_USER ""
  #define MQTT_PASS ""
#endif
#ifndef MQTT_PORT
  #define MQTT_PORT 1883
#endif
#ifndef MQTT_USER
  #define MQTT_USER ""
#endif
#ifndef MQTT_PASS
  #define MQTT_PASS ""
#endif

#define MQTT_TOPIC_PREFIX "truck/isuzu"

/******************************************************************************
 * TeleClientMQTT — drop-in แทน TeleClientHTTP/UDP
 *
 * .cell  เป็น CellMQTT  (extends CellSIMCOM)
 *   → .cell.begin()  .cell.setup()  .cell.end()  .cell.check()
 *   → .cell.RSSI()   .cell.getIP()  .cell.getBuffer()
 *   → .cell.IMEI     .cell.deviceName()  .cell.checkSIM()
 *   → .cell.setGPS() .cell.getLocation() .cell.getOperatorName()
 *   ทุก method เหมือนเดิม — มาจาก CellSIMCOM
 *
 * .wifi  เป็น WifiMQTT  (extends ClientWIFI)
 *   → .wifi.begin()   .wifi.setup()  .wifi.end()
 *   → .wifi.connected() .wifi.getIP() .wifi.RSSI()
 *   ทุก method เหมือนเดิม — มาจาก ClientWIFI
 ******************************************************************************/
class TeleClientMQTT : public TeleClient
{
public:
    bool connect(bool quick = false);
    bool notify(byte event, const char* payload = 0);
    bool transmit(const char* packetBuffer, unsigned int packetSize);
    bool ping();
    void inbound();
    void shutdown();

    CellMQTT cell;   // แทน CellUDP / CellHTTP — API เหมือนเดิมทุก method
#if ENABLE_WIFI
    WifiMQTT wifi;   // แทน WifiUDP / WifiHTTP — API เหมือนเดิมทุก method
#endif

private:
    void initTopics();
    char topicData[80];
    char topicStatus[80];
    char topicCmd[80];
    char clientId[36];
    bool initialized = false;
};

#endif // TELECLIENT_MQTT_H
