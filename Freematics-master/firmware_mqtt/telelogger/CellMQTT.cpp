/******************************************************************************
 * CellMQTT.cpp
 ******************************************************************************/
#include "config.h"    // ต้อง include ก่อนเพื่อให้ ENABLE_WIFI ถูก define
#include "CellMQTT.h"

// ============================================================
// MQTTPacket — builders
// ============================================================
uint8_t MQTTPacket::encodeLength(uint8_t* buf, uint32_t len)
{
    uint8_t i = 0;
    do {
        uint8_t enc = len % 128;
        len /= 128;
        if (len > 0) enc |= 0x80;
        buf[i++] = enc;
    } while (len > 0);
    return i;
}

uint16_t MQTTPacket::writeStr(uint8_t* buf, const char* str)
{
    uint16_t len = str ? (uint16_t)strlen(str) : 0;
    buf[0] = (len >> 8) & 0xFF;
    buf[1] =  len       & 0xFF;
    if (len) memcpy(buf + 2, str, len);
    return 2 + len;
}

uint16_t MQTTPacket::buildConnect(uint8_t* buf, const char* clientId,
                                   const char* user, const char* pass,
                                   const char* willTopic, const char* willMsg)
{
    uint8_t  payload[256];
    uint16_t plen = 0;
    plen += writeStr(payload + plen, clientId);

    bool hasWill = willTopic && willMsg;
    bool hasUser = user && strlen(user) > 0;
    bool hasPass = pass && strlen(pass) > 0;
    if (hasWill) { plen += writeStr(payload + plen, willTopic);
                   plen += writeStr(payload + plen, willMsg); }
    if (hasUser)   plen += writeStr(payload + plen, user);
    if (hasPass)   plen += writeStr(payload + plen, pass);

    uint8_t flags = 0x02; // CleanSession
    if (hasWill) flags |= 0x04;
    if (hasUser) flags |= 0x80;
    if (hasPass) flags |= 0x40;

    uint8_t vh[10] = {0x00,0x04,'M','Q','T','T',0x04, flags,
                      (uint8_t)(MQTT_KEEPALIVE >> 8),
                      (uint8_t)(MQTT_KEEPALIVE & 0xFF)};

    uint32_t remaining = 10 + plen;
    buf[0] = MQTT_CONNECT;
    uint8_t encLen[4];
    uint8_t encSize = encodeLength(encLen, remaining);
    memcpy(buf + 1, encLen, encSize);
    uint16_t pos = 1 + encSize;
    memcpy(buf + pos, vh, 10);        pos += 10;
    memcpy(buf + pos, payload, plen); pos += plen;
    return pos;
}

uint16_t MQTTPacket::buildPublish(uint8_t* buf, const char* topic,
                                   const uint8_t* payload, uint16_t payloadLen)
{
    uint16_t tLen = (uint16_t)strlen(topic);
    uint32_t remaining = 2 + tLen + payloadLen;
    buf[0] = MQTT_PUBLISH;
    uint8_t encLen[4];
    uint8_t encSize = encodeLength(encLen, remaining);
    memcpy(buf + 1, encLen, encSize);
    uint16_t pos = 1 + encSize;
    buf[pos++] = (tLen >> 8) & 0xFF;
    buf[pos++] =  tLen       & 0xFF;
    memcpy(buf + pos, topic,   tLen);       pos += tLen;
    memcpy(buf + pos, payload, payloadLen); pos += payloadLen;
    return pos;
}

uint16_t MQTTPacket::buildSubscribe(uint8_t* buf, const char* topic, uint16_t packetId)
{
    uint16_t tLen = (uint16_t)strlen(topic);
    uint32_t remaining = 2 + 2 + tLen + 1;
    buf[0] = MQTT_SUBSCRIBE;
    uint8_t encLen[4];
    uint8_t encSize = encodeLength(encLen, remaining);
    memcpy(buf + 1, encLen, encSize);
    uint16_t pos = 1 + encSize;
    buf[pos++] = (packetId >> 8) & 0xFF;
    buf[pos++] =  packetId       & 0xFF;
    buf[pos++] = (tLen >> 8) & 0xFF;
    buf[pos++] =  tLen       & 0xFF;
    memcpy(buf + pos, topic, tLen); pos += tLen;
    buf[pos++] = 0x00; // QoS 0
    return pos;
}

uint16_t MQTTPacket::buildPingReq   (uint8_t* buf) { buf[0]=MQTT_PINGREQ;   buf[1]=0; return 2; }
uint16_t MQTTPacket::buildDisconnect(uint8_t* buf) { buf[0]=MQTT_DISCONNECT; buf[1]=0; return 2; }

// ============================================================
// CellMQTT — TCP via AT commands
// ============================================================
bool CellMQTT::tcpOpen(const char* host, uint16_t port)
{
    String ip = queryIP(host);
    if (ip.length() == 0) ip = host;
    char cmd[128];
    if (m_type == CELL_SIM7070) {
        // ปิด socket เก่าก่อนเสมอ ไม่งั้น CAOPEN จะ fail ถ้า socket ค้างอยู่
        sendCommand("AT+CACLOSE=0\r", 1000);
        sendCommand("AT+CNACT=0,1\r");
        sendCommand("AT+CACID=0\r");
        snprintf(cmd, sizeof(cmd), "AT+CAOPEN=0,0,\"TCP\",\"%s\",%u\r", ip.c_str(), port);
        if (!sendCommand(cmd, 10000, "+CAOPEN: 0,0")) {
            Serial.println("[CMQTT] tcpOpen failed (SIM7070)"); return false;
        }
    } else {
        snprintf(cmd, sizeof(cmd), "AT+CIPOPEN=0,\"TCP\",\"%s\",%u\r", ip.c_str(), port);
        if (!sendCommand(cmd, 10000, "+CIPOPEN: 0,0")) {
            Serial.println("[CMQTT] tcpOpen failed"); return false;
        }
    }
    Serial.println("[CMQTT] TCP OK");
    return true;
}

bool CellMQTT::tcpClose()
{
    if (m_type == CELL_SIM7070) sendCommand("AT+CACLOSE=0\r");
    else                        sendCommand("AT+CIPCLOSE=0\r");
    return true;
}

bool CellMQTT::tcpSend(const uint8_t* data, uint16_t len)
{
    char cmd[40];
    if (m_type == CELL_SIM7070) {
        snprintf(cmd, sizeof(cmd), "AT+CASEND=0,%u\r", len);
        if (!sendCommand(cmd, 1000, "\r\n>")) {
            Serial.println("[CMQTT] CASEND prompt failed");
            return false;
        }
        m_device->xbWrite((const char*)data, len);
        const char* ok[] = {"OK\r\n", "ERROR"};
        int ret = m_device->xbReceive(m_buffer, RECV_BUF_SIZE, 2000, ok, 2);
        Serial.print("[CMQTT] tcpSend ret="); Serial.println(ret);
        return ret == 1;
    } else {
        snprintf(cmd, sizeof(cmd), "AT+CIPSEND=0,%u\r", len);
        if (!sendCommand(cmd, 1000, "\r\n>")) return false;
        m_device->xbWrite((const char*)data, len);
        const char* ok[] = {"OK\r\n", "ERROR", "+CIPSEND:"};
        return m_device->xbReceive(m_buffer, RECV_BUF_SIZE, 3000, ok, 3) >= 1;
    }
}

int CellMQTT::tcpRecv(uint8_t* buf, uint16_t maxlen, uint16_t timeout_ms)
{
    if (m_type == CELL_SIM7070) {
        // sendCommand(0,...) = รอ URC โดยไม่ส่ง AT command ใด
        Serial.print("[CMQTT] waiting CADATAIND timeout=");
        Serial.println(timeout_ms);
        bool got = sendCommand(0, timeout_ms, "+CADATAIND: 0");
        Serial.print("[CMQTT] CADATAIND got="); Serial.println(got);
        Serial.print("[CMQTT] m_buffer="); Serial.println(m_buffer);
        if (!got) return 0;
        // SIM7070 ส่ง "+CARECV: <len>,<data>" มาก่อน OK
        // ต้องรอ "+CARECV:" เป็น expected string ไม่ใช่ "OK"
        snprintf(m_buffer, RECV_BUF_SIZE, "AT+CARECV=0,%u\r", maxlen);
        bool recvOk = sendCommand(m_buffer, 2000, "+CARECV:");
        Serial.print("[CMQTT] CARECV ok="); Serial.print(recvOk);
        Serial.print(" buf="); Serial.println(m_buffer);
        if (!recvOk) return 0;
        char* p = strstr(m_buffer, "+CARECV:");
        if (!p) return 0;
        // skip past ": " to get length
        p += 8; // skip "+CARECV:"
        while (*p == ' ') p++; // skip spaces
        int recvLen = atoi(p);
        p = strchr(p, ','); if (!p) return 0; p++;
        if (recvLen > (int)maxlen) recvLen = maxlen;
        memcpy(buf, p, recvLen);
        Serial.print("[CMQTT] tcpRecv bytes="); Serial.println(recvLen);
        return recvLen;
    } else {
        // SIM7600/SIM7670/SIM5360
        if (!sendCommand(0, timeout_ms, "+IPD")) return 0;
        char* p = strstr(m_buffer, "+IPD,");
        if (p) {
            int len = atoi(p + 5);
            p = strchr(p, ':'); if (!p) return 0; p++;
            if (len > (int)maxlen) len = maxlen;
            memcpy(buf, p, len); return len;
        }
        return 0;
    }
}

bool CellMQTT::mqttConnect(const char* host, uint16_t port,
                             const char* clientId, const char* user, const char* pass,
                             const char* willTopic, const char* willMsg)
{
    m_mqttConn = false;
    if (!tcpOpen(host, port)) return false;
    uint16_t pktLen = m_pkt.buildConnect(m_buf, clientId, user, pass, willTopic, willMsg);
    if (!tcpSend(m_buf, pktLen)) { tcpClose(); return false; }

    uint8_t resp[8] = {0};
    int rlen = tcpRecv(resp, sizeof(resp), MQTT_TIMEOUT);
    if (rlen < 4 || resp[0] != MQTT_CONNACK || resp[3] != 0x00) {
        Serial.print("[CMQTT] CONNACK fail rc=");
        Serial.println(rlen > 3 ? (int)resp[3] : -1);
        tcpClose(); return false;
    }
    m_mqttConn = true;
    lastPingTime = millis();
    Serial.println("[CMQTT] Connected");
    return true;
}

void CellMQTT::mqttDisconnect()
{
    if (m_mqttConn) {
        m_pkt.buildDisconnect(m_buf);
        tcpSend(m_buf, 2);
        m_mqttConn = false;
    }
    tcpClose();
}

bool CellMQTT::mqttPublish(const char* topic, const uint8_t* payload, uint16_t len)
{
    if (!m_mqttConn) return false;
    uint16_t pktLen = m_pkt.buildPublish(m_buf, topic, payload, len);
    bool ok = tcpSend(m_buf, pktLen);
    if (!ok) m_mqttConn = false;
    return ok;
}

bool CellMQTT::mqttSubscribe(const char* topic)
{
    if (!m_mqttConn) return false;
    uint16_t pktLen = m_pkt.buildSubscribe(m_buf, topic, m_packetId++);
    bool ok = tcpSend(m_buf, pktLen);
    if (!ok) m_mqttConn = false;
    return ok;
}

bool CellMQTT::mqttLoop()
{
    if (!m_mqttConn) return false;
    if (millis() - lastPingTime >= (uint32_t)MQTT_KEEPALIVE * 500UL) {
        m_pkt.buildPingReq(m_buf);
        if (!tcpSend(m_buf, 2)) { m_mqttConn = false; return false; }
        lastPingTime = millis();
    }
    uint8_t resp[MQTT_PKT_BUF];
    int rlen = tcpRecv(resp, sizeof(resp), 50);
    if (rlen > 1 && (resp[0] & 0xF0) == MQTT_PUBLISH) {
        uint8_t* p = resp + 1;
        uint32_t remLen = 0; uint8_t shift = 0, b;
        do { b = *p++; remLen |= (uint32_t)(b & 0x7F) << shift; shift += 7; } while (b & 0x80);
        uint16_t tLen = ((uint16_t)p[0] << 8) | p[1]; p += 2;
        char topic[128] = {0};
        if (tLen > 127) tLen = 127;
        memcpy(topic, p, tLen); p += tLen;
        int payLen = rlen - (int)(p - resp);
        if (payLen > 0) onMessage(topic, p, (uint16_t)payLen);
    }
    return true;
}

// ============================================================
// WifiMQTT — TCP via WiFiClient
// ============================================================
#if ENABLE_WIFI

bool WifiMQTT::rawSend(const uint8_t* data, uint16_t len)
{
    if (!_client.connected()) return false;
    return (size_t)_client.write(data, len) == len;
}

int WifiMQTT::rawRecv(uint8_t* buf, uint16_t maxlen, uint16_t timeout_ms)
{
    uint32_t t0 = millis();
    while (!_client.available() && (millis() - t0) < timeout_ms) delay(5);
    if (!_client.available()) return 0;
    int n = 0;
    while (_client.available() && n < (int)maxlen) buf[n++] = _client.read();
    return n;
}

bool WifiMQTT::mqttConnect(const char* host, uint16_t port,
                             const char* clientId, const char* user, const char* pass,
                             const char* willTopic, const char* willMsg)
{
    _conn = false;
    Serial.print("[WMQTT] Connecting "); Serial.print(host);
    Serial.print(":"); Serial.println(port);
    if (!_client.connect(host, port)) {
        Serial.println("[WMQTT] TCP failed"); return false;
    }
    uint16_t pktLen = _pkt.buildConnect(_buf, clientId, user, pass, willTopic, willMsg);
    if (!rawSend(_buf, pktLen)) { _client.stop(); return false; }

    uint8_t resp[8] = {0};
    int rlen = rawRecv(resp, sizeof(resp), MQTT_TIMEOUT);
    if (rlen < 4 || resp[0] != MQTT_CONNACK || resp[3] != 0x00) {
        Serial.print("[WMQTT] CONNACK fail rc=");
        Serial.println(rlen > 3 ? (int)resp[3] : -1);
        _client.stop(); return false;
    }
    _conn = true;
    _lastPing = millis();
    Serial.println("[WMQTT] Connected");
    return true;
}

void WifiMQTT::mqttDisconnect()
{
    if (_conn) { _pkt.buildDisconnect(_buf); rawSend(_buf, 2); _conn = false; }
    _client.stop();
}

bool WifiMQTT::mqttPublish(const char* topic, const uint8_t* payload, uint16_t len)
{
    if (!mqttConnected()) { _conn = false; return false; }
    uint16_t pktLen = _pkt.buildPublish(_buf, topic, payload, len);
    bool ok = rawSend(_buf, pktLen);
    if (!ok) _conn = false;
    return ok;
}

bool WifiMQTT::mqttSubscribe(const char* topic)
{
    if (!_conn) return false;
    uint16_t pktLen = _pkt.buildSubscribe(_buf, topic, _pktId++);
    return rawSend(_buf, pktLen);
}

bool WifiMQTT::mqttLoop()
{
    if (!mqttConnected()) { _conn = false; return false; }
    if (millis() - _lastPing >= (uint32_t)MQTT_KEEPALIVE * 500UL) {
        _pkt.buildPingReq(_buf); rawSend(_buf, 2); _lastPing = millis();
    }
    if (_client.available()) {
        uint8_t resp[MQTT_PKT_BUF];
        int rlen = rawRecv(resp, sizeof(resp), 50);
        if (rlen > 1 && (resp[0] & 0xF0) == MQTT_PUBLISH) {
            uint8_t* p = resp + 1;
            uint32_t remLen = 0; uint8_t shift = 0, b;
            do { b = *p++; remLen |= (uint32_t)(b & 0x7F) << shift; shift += 7; } while (b & 0x80);
            uint16_t tLen = ((uint16_t)p[0] << 8) | p[1]; p += 2;
            char topic[128] = {0};
            if (tLen > 127) tLen = 127;
            memcpy(topic, p, tLen); p += tLen;
            int payLen = rlen - (int)(p - resp);
            if (payLen > 0) onMessage(topic, p, (uint16_t)payLen);
        }
    }
    return true;
}

#endif // ENABLE_WIFI
