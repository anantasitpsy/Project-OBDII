# -*- coding: utf-8 -*-

import sys, time, threading, argparse

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("[ERROR] pip install paho-mqtt"); sys.exit(1)

MQTT_HOST   = "localhost"
MQTT_PORT   = 1883
MQTT_USER   = ""
MQTT_PASS   = ""
BASE_TOPIC  = ""

PID_NAMES = {
    0x01: "MONITOR_STATUS",          0x02: "FREEZE_FRAME_DTC",
    0x03: "FUEL_SYSTEM_STATUS",
    0x04: "ENGINE_LOAD",             0x05: "COOLANT_TEMP",
    0x06: "ST_FUEL_TRIM_1",          0x07: "LT_FUEL_TRIM_1",
    0x08: "ST_FUEL_TRIM_2",          0x09: "LT_FUEL_TRIM_2",
    0x0A: "FUEL_PRESSURE",           0x0B: "INTAKE_MAP",
    0x0C: "RPM",                     0x0D: "SPEED",
    0x0E: "TIMING_ADVANCE",          0x0F: "INTAKE_TEMP",
    0x10: "MAF_FLOW",                0x11: "THROTTLE",
    0x12: "SECONDARY_AIR_STATUS",    0x13: "O2_SENSORS_PRESENT",
    0x14: "O2_B1S1_VOLTAGE",         0x15: "O2_B1S2_VOLTAGE",
    0x16: "O2_B1S3_VOLTAGE",         0x17: "O2_B1S4_VOLTAGE",
    0x18: "O2_B2S1_VOLTAGE",         0x19: "O2_B2S2_VOLTAGE",
    0x1A: "O2_B2S3_VOLTAGE",         0x1B: "O2_B2S4_VOLTAGE",
    0x1C: "OBD_STANDARDS",           0x1D: "O2_SENSORS_PRESENT_B2",
    0x1E: "AUX_INPUT",               0x1F: "RUNTIME",
    0x21: "DIST_MIL",
    0x22: "FUEL_RAIL_GAUGE_PRESS",   0x23: "FUEL_RAIL_ABS_PRESS",
    0x24: "O2WR_B1S1_LAMBDA",        0x25: "O2WR_B1S2_LAMBDA",
    0x26: "O2WR_B1S3_LAMBDA",        0x27: "O2WR_B1S4_LAMBDA",
    0x28: "O2WR_B2S1_LAMBDA",        0x29: "O2WR_B2S2_LAMBDA",
    0x2A: "O2WR_B2S3_LAMBDA",        0x2B: "O2WR_B2S4_LAMBDA",
    0x2C: "EGR_CMD",                 0x2D: "EGR_ERROR",
    0x2E: "EVAP_PURGE",              0x2F: "FUEL_LEVEL",
    0x30: "WARM_UPS",                0x31: "DISTANCE",
    0x32: "EVAP_PRESSURE",           0x33: "BAROMETRIC",
    0x34: "O2WR_B1S1_CURRENT",       0x35: "O2WR_B1S2_CURRENT",
    0x36: "O2WR_B1S3_CURRENT",       0x37: "O2WR_B1S4_CURRENT",
    0x38: "O2WR_B2S1_CURRENT",       0x39: "O2WR_B2S2_CURRENT",
    0x3A: "O2WR_B2S3_CURRENT",       0x3B: "O2WR_B2S4_CURRENT",
    0x3C: "CAT_TEMP_B1S1",           0x3D: "CAT_TEMP_B2S1",
    0x3E: "CAT_TEMP_B1S2",           0x3F: "CAT_TEMP_B2S2",
    0x41: "MONITOR_STATUS_DRIVE",
    0x42: "MODULE_VOLTAGE",          0x43: "ABS_ENGINE_LOAD",
    0x44: "AFR_RATIO",               0x45: "REL_THROTTLE",
    0x46: "AMBIENT_TEMP",
    0x47: "ABS_THROTTLE_B",          0x48: "ABS_THROTTLE_C",
    0x49: "ACC_PEDAL_D",             0x4A: "ACC_PEDAL_E",
    0x4B: "ACC_PEDAL_F",             0x4C: "CMD_THROTTLE_ACT",
    0x4D: "TIME_WITH_MIL",           0x4E: "TIME_CODES_CLEARED",
    0x4F: "MAX_VALUES",              0x50: "MAX_MAF",
    0x51: "FUEL_TYPE",               0x52: "ETHANOL",
    0x53: "ABS_EVAP_PRESSURE",       0x54: "EVAP_PRESSURE_2",
    0x55: "ST_O2_TRIM_B1_B3",        0x56: "LT_O2_TRIM_B1_B3",
    0x57: "ST_O2_TRIM_B2_B4",        0x58: "LT_O2_TRIM_B2_B4",
    0x59: "FUEL_RAIL_PRES",          0x5A: "REL_ACCEL_PEDAL",
    0x5B: "HYBRID_BATTERY",          0x5C: "OIL_TEMP",
    0x5D: "INJECT_TIMING",           0x5E: "FUEL_RATE",
    0x5F: "EMISSION_REQ",
    0x61: "TORQUE_DEMAND",           0x62: "TORQUE_PCT",
    0x63: "REF_TORQUE",              0x64: "ENGINE_PCT_TORQUE",
    0x65: "AUX_IO_SUPPORTED",        0x66: "MAF_SENSOR_B",
    0x67: "COOLANT_TEMP_2SENSOR",    0x68: "INTAKE_AIR_TEMP_SENSOR",
    0x69: "CMD_EGR_2",               0x6A: "CMD_DIESEL_INTAKE",
    0x6B: "EGR_TEMP",                0x6C: "CMD_THROTTLE_2",
    0x6D: "FUEL_PRESS_CTRL",         0x6E: "INJECT_PRESS_CTRL",
    0x6F: "TURBO_COMPRESSOR_INLET",
    0x70: "BOOST_PRESSURE",          0x71: "VGT_CTRL",
    0x72: "WASTEGATE_CTRL",          0x73: "EXHAUST_PRESSURE",
    0x74: "TURBO_RPM",               0x75: "TURBO_TEMP_A",
    0x76: "TURBO_TEMP_B",            0x77: "INTERCOOLER_TEMP",
    0x78: "EGT_BANK1",               0x79: "EGT_BANK2",
    0x7A: "DPF_DIFF_PRESSURE",       0x7B: "DPF_TEMP",
    0x7C: "NOX_NTE_CTRL",            0x7D: "PM_NTE_CTRL",
    0x7E: "ENGINE_RUN_TIME",         0x7F: "ENGINE_RUN_TIME_2",
    0x81: "RSSI",                    0x82: "DEVICE_TEMP",
    0x83: "NOX_SENSOR",              0x84: "MANIFOLD_SURF_TEMP",
    0x85: "NOX_REAGENT_SYS",         0x86: "PM_SENSOR",
    0x87: "INTAKE_MAP_B",            0x8D: "THROTTLE_POS_G",
    0x8E: "ENGINE_FRICTION_TORQUE",
    0xA6: "ODOMETER",
}

PID_UNITS = {
    0x04: ("%",    lambda v: v),
    0x11: ("%",    lambda v: v),
    0x2C: ("%",    lambda v: v * 100.0 / 255),
    0x2E: ("%",    lambda v: v * 100.0 / 255),
    0x2F: ("%",    lambda v: v),
    0x43: ("%",    lambda v: v * 100.0 / 255),
    0x45: ("%",    lambda v: v * 100.0 / 255),
    0x47: ("%",    lambda v: v * 100.0 / 255),
    0x48: ("%",    lambda v: v * 100.0 / 255),
    0x49: ("%",    lambda v: v * 100.0 / 255),
    0x4A: ("%",    lambda v: v * 100.0 / 255),
    0x4B: ("%",    lambda v: v * 100.0 / 255),
    0x4C: ("%",    lambda v: v * 100.0 / 255),
    0x52: ("%",    lambda v: v * 100.0 / 255),
    0x5A: ("%",    lambda v: v * 100.0 / 255),
    0x5B: ("%",    lambda v: v * 100.0 / 255),
    0x06: ("%",    lambda v: (v - 128) * 100.0 / 128),
    0x07: ("%",    lambda v: (v - 128) * 100.0 / 128),
    0x08: ("%",    lambda v: (v - 128) * 100.0 / 128),
    0x09: ("%",    lambda v: (v - 128) * 100.0 / 128),
    0x2D: ("%",    lambda v: (v - 128) * 100.0 / 128),
    0x55: ("%",    lambda v: (v - 128) * 100.0 / 128),
    0x56: ("%",    lambda v: (v - 128) * 100.0 / 128),
    0x57: ("%",    lambda v: (v - 128) * 100.0 / 128),
    0x58: ("%",    lambda v: (v - 128) * 100.0 / 128),
    0x0A: ("kPa",  lambda v: v * 3),
    0x0B: ("kPa",  lambda v: v),
    0x22: ("kPa",  lambda v: v * 0.079),
    0x23: ("kPa",  lambda v: v * 10),
    0x33: ("kPa",  lambda v: v),
    0x53: ("kPa",  lambda v: v / 200.0),
    0x54: ("Pa",   lambda v: v - 32767),
    0x59: ("kPa",  lambda v: v * 10),
    0x70: ("kPa",  lambda v: v * 0.03125),
    0x73: ("kPa",  lambda v: v),
    0x0C: ("RPM",  lambda v: v),
    0x0D: ("km/h", lambda v: v),
    0x74: ("RPM",  lambda v: v * 16),
    0x0E: ("deg",  lambda v: v),
    0x5D: ("deg",  lambda v: (v - 26880) / 128.0),
    0x10: ("g/s",  lambda v: v),
    0x66: ("g/s",  lambda v: v / 32.0),
    0x05: ("C",    lambda v: v),
    0x0F: ("C",    lambda v: v),
    0x46: ("C",    lambda v: v - 40),
    0x5C: ("C",    lambda v: v - 40),
    0x67: ("C",    lambda v: v / 1000),
    0x68: ("C",    lambda v: v / 1000),
    0x6B: ("C",    lambda v: v - 40),
    0x75: ("C",    lambda v: v - 40),
    0x76: ("C",    lambda v: v - 40),
    0x77: ("C",    lambda v: v - 40),
    0x81: ("dBm",  lambda v: v),
    0x82: ("C",    lambda v: v),
    0x84: ("C",    lambda v: v - 40),
    0x3C: ("C",    lambda v: v / 10.0 - 40),
    0x3D: ("C",    lambda v: v / 10.0 - 40),
    0x3E: ("C",    lambda v: v / 10.0 - 40),
    0x3F: ("C",    lambda v: v / 10.0 - 40),
    0x78: ("C",    lambda v: v / 10.0 - 40),
    0x79: ("C",    lambda v: v / 10.0 - 40),
    0x7B: ("C",    lambda v: v / 10.0 - 40),
    0x42: ("V",    lambda v: v),
    0x14: ("mV",   lambda v: v * 5),   0x15: ("mV",   lambda v: v * 5),
    0x16: ("mV",   lambda v: v * 5),   0x17: ("mV",   lambda v: v * 5),
    0x18: ("mV",   lambda v: v * 5),   0x19: ("mV",   lambda v: v * 5),
    0x1A: ("mV",   lambda v: v * 5),   0x1B: ("mV",   lambda v: v * 5),
    0x24: ("\u03bb",    lambda v: v / 100),
    0x25: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x26: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x27: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x28: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x29: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x2A: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x2B: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x34: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x35: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x36: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x37: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x38: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x39: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x3A: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x3B: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x44: ("\u03bb",    lambda v: v * 2.0 / 65536),
    0x61: ("%",    lambda v: v - 125),
    0x62: ("%",    lambda v: v - 125),
    0x64: ("%",    lambda v: v - 125),
    0x8E: ("%",    lambda v: v - 125),
    0x5E: ("L/h",  lambda v: v / 20.0),
    0x32: ("Pa",   lambda v: v - 32768),
    0x1F: ("s",    lambda v: v),
    0x21: ("km",   lambda v: v),
    0x31: ("km",   lambda v: v),
    0x4D: ("min",  lambda v: v),
    0x4E: ("min",  lambda v: v),
    0x30: ("",     lambda v: v),
    0x63: ("Nm",   lambda v: v),
    0xA6: ("km",   lambda v: v / 10.0),
    0x83: ("ppm",  lambda v: v),
    0x1C: ("",     lambda v: PROTOCOL_MAP.get(int(v) + 160, "Unknown")),
}

GPS_KEYS = {
    "A": "lat", "B": "lon", "C": "altitude",
    "D": "gps_speed", "E": "heading", "F": "satellites",
}

PROTOCOL_MAP = {
    163: "ISO 9141-2",
    164: "ISO 14230-4 (KWP 5BAUD)",
    165: "ISO 14230-4 (KWP FAST)",
    166: "ISO 15765-4 (CAN 11/500)",
    167: "ISO 15765-4 (CAN 29/500)",
    168: "ISO 15765-4 (CAN 11/250)",
    169: "ISO 15765-4 (CAN 29/250)",
}

sessions      = {}
sessions_lock = threading.Lock()

def get_session(devid):
    with sessions_lock:
        if devid not in sessions:
            sessions[devid] = {
                "rx_count": 0, "vin": "",
                "data": {}, "gps": {}, "imu": {},
                "protocol": 0,
            }
            print("[NEW DEVICE] devid={0}".format(devid))
        sessions[devid]["last_seen"] = time.time()
        return sessions[devid]

def decode_pid(key_str, raw_val):
    try:
        pid = int(key_str, 16) & 0xFF
        pid_label = "PID{0}".format(key_str.upper().lstrip("0") or "0")
    except ValueError:
        return key_str, raw_val, ""

    name     = PID_NAMES.get(pid, "UNKNOWN")
    label    = "[{0}] {1}".format(pid_label, name)

    if pid in PID_UNITS and not isinstance(raw_val, list):
        unit, fn = PID_UNITS[pid]
        try:
            calculated_val = fn(raw_val)
            if isinstance(calculated_val, (int, float)):
                calculated_val = round(calculated_val, 3)

            return label, calculated_val, unit
        except Exception:
            pass

    return label, raw_val, ""

def parse_packed(payload, devid=""):
    results = []
    for line in payload.strip().splitlines():
        line = line.strip()
        if not line: continue
        if "*" in line:
            line = line[:line.rfind("*")]
        if "#" in line:
            dev, rest = line.split("#", 1)
        else:
            dev, rest = devid, line
        for pair in rest.split(","):
            if ":" not in pair: continue
            k, v = pair.split(":", 1)
            k, v = k.strip(), v.strip()
            if ";" in v:
                try:
                    val = [float(x) if "." in x else int(x) for x in v.split(";")]
                except ValueError: continue
            else:
                try:
                    val = float(v) if "." in v else int(v)
                except ValueError: continue
            results.append((dev.strip(), k, val))
    return results

def handle_data(devid, payload):
    sess = get_session(devid)
    rows = parse_packed(payload, devid)

    print("\n[DATA] devid={0}  ({1} fields)".format(devid, len(rows)))
    print("-" * 60)

    for _, k, raw_val in rows:
        ku = k.strip().upper()

        if ku == "0":
            print("  {0:<40} = {1}".format("TIMESTAMP", raw_val))
            continue

        if ku == "FF":
            try:
                name = PROTOCOL_MAP.get(int(raw_val), "Unknown")
                print("  {0:<40} = {1}  (raw={2})".format("PROTOCOL", name, int(raw_val)))
                sess["protocol"] = int(raw_val)
            except Exception: pass
            continue

        if ku in GPS_KEYS:
            gname = GPS_KEYS[ku]
            print("  {0:<40} = {1}".format("GPS_" + gname, raw_val))
            sess["gps"][gname] = raw_val
            continue

        if ku == "20":
            if isinstance(raw_val, list):
                print("  {0:<40} = x={1}g y={2}g z={3}g".format("[PID20] ACCEL", *raw_val[:3]))
                sess["imu"]["accel"] = raw_val
            continue
        
        if ku == "10":
            ms  = int(raw_val) if isinstance(raw_val, (int, float)) else 0
            sec = ms // 1000
            hh, mm, ss = sec // 3600, (sec % 3600) // 60, sec % 60
            tstr = "{0:02d}:{1:02d}:{2:02d} UTC".format(hh, mm, ss)
            print("  {0:<40} = {1}  (raw={2})".format("GPS_TIME", tstr, raw_val))
            sess["gps"]["time"] = tstr
            continue

        if ku == "24":
            v = raw_val / 100.0 if isinstance(raw_val, (int, float)) else raw_val
            print("  {0:<40} = {1} V  (raw={2})".format("[PID24] BATTERY_VOLT", round(v, 2), raw_val))
            sess["data"][k] = v
            continue

        label, val, unit = decode_pid(k, raw_val)
        unit_str = " " + unit if unit else ""
        print("  {0:<40} = {1}{2}  (raw={3})".format(label, val, unit_str, raw_val))
        sess["data"][k] = val

    print("-" * 60)
    sess["rx_count"] += 1

def handle_status(devid, payload):
    sess = get_session(devid)
    print("[STATUS] devid={0}  {1}".format(devid, payload))
    sess["rx_count"] += 1

def on_connect(client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        topic = "{0}/#".format(BASE_TOPIC)
        client.subscribe(topic, qos=0)
        print("[MQTT] Connected \u2014 subscribed to: {0}".format(topic))
    else:
        print("[MQTT] Connect failed rc={0}".format(reason_code))

def on_disconnect(client, userdata, flags, reason_code, properties=None):
    print("[MQTT] Disconnected rc={0}".format(reason_code))

def on_message(client, userdata, msg):
    try:
        topic   = msg.topic
        payload = msg.payload.decode("utf-8", errors="replace").strip()
        parts   = topic.split("/")

        if len(parts) < 4:
            print("[WARN] Unexpected topic: {0}".format(topic))
            return

        devid  = parts[-2]
        action = parts[-1].lower()

        if action == "data":
            handle_data(devid, payload)
        elif action == "status":
            handle_status(devid, payload)
        else:
            print("[UNKNOWN] topic={0} payload={1}".format(topic, payload[:80]))

    except Exception as e:
        print("[ERROR] {0}".format(e))

def on_subscribe(client, userdata, mid, reason_codes, properties=None):
    print("[MQTT] Subscribed ok")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host",  default=MQTT_HOST)
    parser.add_argument("--port",  default=MQTT_PORT, type=int)
    parser.add_argument("--user",  default=MQTT_USER)
    parser.add_argument("--pass",  default=MQTT_PASS, dest="password")
    parser.add_argument("--topic", default=BASE_TOPIC)
    args = parser.parse_args()

    BASE_TOPIC = args.topic

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id="freematics-sub", clean_session=True)
    except AttributeError:
        client = mqtt.Client(client_id="freematics-sub", clean_session=True)
    client.username_pw_set(args.user, args.password)
    client.on_connect    = on_connect
    client.on_disconnect = on_disconnect
    client.on_message    = on_message
    client.on_subscribe  = on_subscribe

    print("[Subscriber] Connecting to {0}:{1}  topic={2}/#".format(
        args.host, args.port, BASE_TOPIC))

    client.connect(args.host, args.port, keepalive=60)
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n[Subscriber] Stopped")
        client.disconnect()
