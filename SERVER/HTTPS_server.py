# -*- coding: utf-8 -*-

from __future__ import print_function, division
import sys
import os
import json
import time
import threading
import traceback
import ssl

PY3 = sys.version_info[0] == 3

if PY3:
    from http.server import HTTPServer, BaseHTTPRequestHandler
    from urllib.parse import urlparse, parse_qs, unquote
else:
    from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler
    from urlparse import urlparse, parse_qs
    from urllib import unquote

HTTP_PORT = 8080
BIND_IP   = "0.0.0.0"
CERTFILE  = ""
KEYFILE   = ""

PID_NAMES = {
    0x01: "MONITOR_STATUS",
    0x02: "FREEZE_FRAME_DTC",
    0x03: "FUEL_SYSTEM_STATUS",
    0x04: "ENGINE_LOAD",
    0x05: "COOLANT_TEMP",
    0x06: "ST_FUEL_TRIM_1",
    0x07: "LT_FUEL_TRIM_1",
    0x08: "ST_FUEL_TRIM_2",
    0x09: "LT_FUEL_TRIM_2",
    0x0A: "FUEL_PRESSURE",
    0x0B: "INTAKE_MAP",
    0x0C: "RPM",
    0x0D: "SPEED",
    0x0E: "TIMING_ADVANCE",
    0x0F: "INTAKE_TEMP",
    0x10: "MAF_FLOW",
    0x11: "THROTTLE",
    0x12: "SECONDARY_AIR_STATUS",
    0x13: "O2_SENSORS_PRESENT",
    0x14: "O2_B1S1_VOLTAGE",
    0x15: "O2_B1S2_VOLTAGE",
    0x16: "O2_B1S3_VOLTAGE",
    0x17: "O2_B1S4_VOLTAGE",
    0x18: "O2_B2S1_VOLTAGE",
    0x19: "O2_B2S2_VOLTAGE",
    0x1A: "O2_B2S3_VOLTAGE",
    0x1B: "O2_B2S4_VOLTAGE",
    0x1C: "OBD_STANDARDS",
    0x1D: "O2_SENSORS_PRESENT_B2",
    0x1E: "AUX_INPUT",
    0x1F: "RUNTIME",
    0x21: "DIST_MIL",
    0x22: "FUEL_RAIL_GAUGE_PRESS",
    0x23: "FUEL_RAIL_ABS_PRESS",
    0x24: "Voltage",
    0x25: "O2WR_B1S2_LAMBDA",
    0x26: "O2WR_B1S3_LAMBDA",
    0x27: "O2WR_B1S4_LAMBDA",
    0x28: "O2WR_B2S1_LAMBDA",
    0x29: "O2WR_B2S2_LAMBDA",
    0x2A: "O2WR_B2S3_LAMBDA",
    0x2B: "O2WR_B2S4_LAMBDA",
    0x2C: "EGR_CMD",
    0x2D: "EGR_ERROR",
    0x2E: "EVAP_PURGE",
    0x2F: "FUEL_LEVEL",
    0x30: "WARM_UPS",
    0x31: "DISTANCE",
    0x32: "EVAP_PRESSURE",
    0x33: "BAROMETRIC",
    0x34: "O2WR_B1S1_CURRENT",
    0x35: "O2WR_B1S2_CURRENT",
    0x36: "O2WR_B1S3_CURRENT",
    0x37: "O2WR_B1S4_CURRENT",
    0x38: "O2WR_B2S1_CURRENT",
    0x39: "O2WR_B2S2_CURRENT",
    0x3A: "O2WR_B2S3_CURRENT",
    0x3B: "O2WR_B2S4_CURRENT",
    0x3C: "CAT_TEMP_B1S1",
    0x3D: "CAT_TEMP_B2S1",
    0x3E: "CAT_TEMP_B1S2",
    0x3F: "CAT_TEMP_B2S2",
    0x41: "MONITOR_STATUS_DRIVE",
    0x42: "MODULE_VOLTAGE",
    0x43: "ABS_ENGINE_LOAD",
    0x44: "AFR_RATIO",
    0x45: "REL_THROTTLE",
    0x46: "AMBIENT_TEMP",
    0x47: "ABS_THROTTLE_B",
    0x48: "ABS_THROTTLE_C",
    0x49: "ACC_PEDAL_D",
    0x4A: "ACC_PEDAL_E",
    0x4B: "ACC_PEDAL_F",
    0x4C: "CMD_THROTTLE_ACT",
    0x4D: "TIME_WITH_MIL",
    0x4E: "TIME_CODES_CLEARED",
    0x4F: "MAX_VALUES",
    0x50: "MAX_MAF",
    0x51: "FUEL_TYPE",
    0x52: "ETHANOL",
    0x53: "ABS_EVAP_PRESSURE",
    0x54: "EVAP_PRESSURE_2",
    0x55: "ST_O2_TRIM_B1_B3",
    0x56: "LT_O2_TRIM_B1_B3",
    0x57: "ST_O2_TRIM_B2_B4",
    0x58: "LT_O2_TRIM_B2_B4",
    0x59: "FUEL_RAIL_PRES",
    0x5A: "REL_ACCEL_PEDAL",
    0x5B: "HYBRID_BATTERY",
    0x5C: "OIL_TEMP",
    0x5D: "INJECT_TIMING",
    0x5E: "FUEL_RATE",
    0x5F: "EMISSION_REQ",
    0x61: "TORQUE_DEMAND",
    0x62: "TORQUE_PCT",
    0x63: "REF_TORQUE",
    0x64: "ENGINE_PCT_TORQUE",
    0x65: "AUX_IO_SUPPORTED",
    0x66: "MAF_SENSOR_B",
    0x67: "COOLANT_TEMP_2SENSOR",
    0x68: "INTAKE_AIR_TEMP_SENSOR",
    0x69: "CMD_EGR_2",
    0x6A: "CMD_DIESEL_INTAKE",
    0x6B: "EGR_TEMP",
    0x6C: "CMD_THROTTLE_2",
    0x6D: "FUEL_PRESS_CTRL",
    0x6E: "INJECT_PRESS_CTRL",
    0x6F: "TURBO_COMPRESSOR_INLET",
    0x70: "BOOST_PRESSURE",
    0x71: "VGT_CTRL",
    0x72: "WASTEGATE_CTRL",
    0x73: "EXHAUST_PRESSURE",
    0x74: "TURBO_RPM",
    0x75: "TURBO_TEMP_A",
    0x76: "TURBO_TEMP_B",
    0x77: "INTERCOOLER_TEMP",
    0x78: "EGT_BANK1",
    0x79: "EGT_BANK2",
    0x7A: "DPF_DIFF_PRESSURE",
    0x7B: "DPF_TEMP",
    0x7C: "NOX_NTE_CTRL",
    0x7D: "PM_NTE_CTRL",
    0x7E: "ENGINE_RUN_TIME",
    0x7F: "ENGINE_RUN_TIME_2",
    0x83: "NOX_SENSOR",
    0x84: "MANIFOLD_SURF_TEMP",
    0x85: "NOX_REAGENT_SYS",
    0x86: "PM_SENSOR",
    0x87: "INTAKE_MAP_B",
    0x8D: "THROTTLE_POS_G",
    0x8E: "ENGINE_FRICTION_TORQUE",
    0xA6: "ODOMETER",
    0x82: "DEVICE_TEMP",
}

PID_UNITS = {
    0x04: ("%",    lambda v: v),
    0x11: ("%",    lambda v: v ),
    0x2C: ("%",    lambda v: v * 100.0 / 255),
    0x2E: ("%",    lambda v: v * 100.0 / 255),
    0x2F: ("%",    lambda v: v ),
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
    0x14: ("mV",   lambda v: v * 5),
    0x15: ("mV",   lambda v: v * 5),
    0x16: ("mV",   lambda v: v * 5),
    0x17: ("mV",   lambda v: v * 5),
    0x18: ("mV",   lambda v: v * 5),
    0x19: ("mV",   lambda v: v * 5),
    0x1A: ("mV",   lambda v: v * 5),
    0x1B: ("mV",   lambda v: v * 5),
    0x24: ("V",    lambda v: v / 100),
    0x25: ("λ",    lambda v: v * 2.0 / 65536),
    0x26: ("λ",    lambda v: v * 2.0 / 65536),
    0x27: ("λ",    lambda v: v * 2.0 / 65536),
    0x28: ("λ",    lambda v: v * 2.0 / 65536),
    0x29: ("λ",    lambda v: v * 2.0 / 65536),
    0x2A: ("λ",    lambda v: v * 2.0 / 65536),
    0x2B: ("λ",    lambda v: v * 2.0 / 65536),
    0x34: ("λ",    lambda v: v * 2.0 / 65536),
    0x35: ("λ",    lambda v: v * 2.0 / 65536),
    0x36: ("λ",    lambda v: v * 2.0 / 65536),
    0x37: ("λ",    lambda v: v * 2.0 / 65536),
    0x38: ("λ",    lambda v: v * 2.0 / 65536),
    0x39: ("λ",    lambda v: v * 2.0 / 65536),
    0x3A: ("λ",    lambda v: v * 2.0 / 65536),
    0x3B: ("λ",    lambda v: v * 2.0 / 65536),
    0x44: ("λ",    lambda v: v * 2.0 / 65536),
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
    "A": ("lat",        "°",    lambda v: float(v)),
    "B": ("lon",        "°",    lambda v: float(v)),
    "C": ("altitude",   "m",    lambda v: float(v)),
    "D": ("gps_speed",  "km/h", lambda v: float(v)),
    "E": ("heading",    "°",    lambda v: float(v)),
    "F": ("satellites", "",     lambda v: int(float(v))),
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
next_feed_id  = 1

def get_session(devid):
    global next_feed_id
    with sessions_lock:
        if devid not in sessions:
            sessions[devid] = {
                "feed_id":       next_feed_id,
                "rx_count":      0,
                "vin":           "",
                "last_seen":     time.time(),
                "data":          {},
                "protocol":      0,
                "protocol_name": "Unknown",
                "gps":           {},
                "imu":           {},
            }
            next_feed_id += 1
        sessions[devid]["last_seen"] = time.time()
        return sessions[devid]

def decode_pid(key_str, raw_val):
    try:
        pid = int(key_str, 16) & 0xFF
    except (ValueError, TypeError):
        return key_str, raw_val, ""
    name = PID_NAMES.get(pid, "PID_0x{0:02X}".format(pid))

    if pid == 0x67 and isinstance(raw_val, list):
        if len(raw_val) >= 3:
            s1 = raw_val[1] - 40
            s2 = raw_val[2] - 40
            return name, "{0}C/{1}C".format(s1, s2), "sensor1/sensor2"
        elif len(raw_val) >= 2:
            return name, "{0}C".format(raw_val[1] - 40), "C"

    if pid in PID_UNITS and not isinstance(raw_val, list):
        unit, fn = PID_UNITS[pid]
        try:
            return name, round(fn(raw_val), 3), unit
        except Exception:
            pass
    return name, raw_val, ""

def parse_packed_data(body_str, devid=""):
    results = []
    for line in body_str.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        if "*" in line:
            line = line[:line.rfind("*")]
        if "#" in line:
            dev, rest = line.split("#", 1)
        else:
            dev, rest = devid, line
        dev = dev.strip()
        for pair in rest.split(","):
            pair = pair.strip()
            if ":" not in pair:
                continue
            k, v = pair.split(":", 1)
            k = k.strip()
            v = v.strip()
            if not k or not v:
                continue
            if ";" in v:
                try:
                    val = [float(x) if "." in x else int(x) for x in v.split(";")]
                except ValueError:
                    continue
            else:
                try:
                    val = float(v) if "." in v else int(v)
                except ValueError:
                    continue
            results.append((dev, k, val))
    return results

class FreematicsHandler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):
        pass

    def send_json(self, code, obj):
        try:
            body = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type",   "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Connection",     "close")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
            self.wfile.flush()
        except Exception as e:
            print("[WARN] send_json error: {0}".format(e))

    def do_GET(self):
        try:
            self._handle_get()
        except Exception:
            traceback.print_exc()
            try:
                self.send_json(500, {"result": "failed", "error": "internal error"})
            except Exception:
                pass

    def _handle_get(self):
        parsed = urlparse(self.path)
        path   = parsed.path.rstrip("/")
        qs     = parse_qs(parsed.query)

        def qget(k, default=""):
            vals = qs.get(k, [default])
            return vals[0] if vals else default

        parts = [p for p in path.split("/") if p]

        if "api" in parts:
            parts = parts[parts.index("api"):]

        if len(parts) >= 3 and parts[0] == "api" and parts[1] == "notify":
            devid = parts[2]
            ev    = qget("EV", "1")
            ts    = qget("TS", "0")
            vin   = qget("VIN", "")
            sess  = get_session(devid)
            if vin:
                sess["vin"] = vin
            sess["rx_count"] += 1
            ev_names = {"1": "LOGIN", "2": "LOGOUT", "4": "PING", "5": "SYNC"}
            print("[{0}] devid={1} id={2} vin={3} ts={4} from={5}".format(
                ev_names.get(ev, "EV" + ev), devid,
                sess["feed_id"], vin or "-", ts,
                self.client_address[0]))
            self.send_json(200, {"result": "done", "id": sess["feed_id"]})
            return

        if len(parts) >= 3 and parts[0] == "api" and parts[1] == "push":
            devid = parts[2]
            sess  = get_session(devid)
            ts    = qget("TS", "0")
            count = 0
            print("[PUSH] devid={0} id={1} ts={2} from={3}".format(
                devid, sess["feed_id"], ts, self.client_address[0]))
            for k, v_list in qs.items():
                try:
                    if k == "TS":
                        continue
                    raw_str = v_list[0]
                    ku = k.upper()
                    if ku == "FF":
                        proto_dec  = int(raw_str)
                        proto_name = PROTOCOL_MAP.get(proto_dec, "Unknown")
                        sess["protocol"]      = proto_dec
                        sess["protocol_name"] = proto_name
                        print("  [FF] {0}  ->  {1}".format(proto_dec, proto_name))
                        count += 1
                        continue
                    if ku in GPS_KEYS:
                        gname, gunit, gfn = GPS_KEYS[ku]
                        gval = gfn(raw_str)
                        unit_str = " " + gunit if gunit else ""
                        print("  [GPS:{0}] {1:<28} = {2}{3}".format(ku, gname, gval, unit_str))
                        sess["gps"][gname] = gval
                        count += 1
                        continue
                    if ku == "10":
                        ms   = int(raw_str)
                        sec  = ms // 1000
                        hh, mm, ss = sec // 3600, (sec % 3600) // 60, sec % 60
                        tstr = "{0:02d}:{1:02d}:{2:02d} UTC".format(hh, mm, ss)
                        print("  [GPS:10] {0:<28} = {1}".format("gps_time", tstr))
                        sess["gps"]["time"] = tstr
                        count += 1
                        continue
                    if ku == "20":
                        if ";" in raw_str:
                            imu = [float(x) for x in raw_str.split(";")]
                            print("  [IMU:20] ax={0}g ay={1}g az={2}g".format(*imu[:3]))
                            sess["imu"]["accel"] = imu
                        count += 1
                        continue
                    if ";" in raw_str:
                        raw_val = [float(x) if "." in x else int(x) for x in raw_str.split(";")]
                    else:
                        raw_val = float(raw_str) if "." in raw_str else int(raw_str)
                    name, val, unit = decode_pid(k, raw_val)
                    unit_str = " " + unit if unit else ""
                    print("  [{0}] {1:<30} = {2}{3}  (raw={4})".format(
                        k, name, val, unit_str, raw_val))
                    sess["data"][k] = val
                    count += 1
                except Exception as e:
                    print("  [WARN] push key={0} err={1}".format(k, e))
                    continue
            sess["rx_count"] += 1
            print("")
            self.send_json(200, {"result": count})
            return

        if len(parts) >= 2 and parts[0] == "api" and parts[1] == "channels":
            now      = time.time()
            channels = []
            with sessions_lock:
                for did, s in sessions.items():
                    channels.append({
                        "id":    s["feed_id"],
                        "devid": did,
                        "vin":   s["vin"],
                        "recv":  s["rx_count"],
                        "age":   int((now - s["last_seen"]) * 1000),
                    })
            self.send_json(200, {"channels": channels})
            return

        if len(parts) >= 3 and parts[0] == "api" and parts[1] == "get":
            devid = parts[2]
            with sessions_lock:
                sess = sessions.get(devid)
            if not sess:
                self.send_json(404, {"result": "failed", "error": "Device not found"})
                return
            data_out = []
            for k, v in sess["data"].items():
                try:
                    pid_dec = int(k, 16) & 0xFF
                except Exception:
                    pid_dec = 0
                data_out.append([pid_dec, v, 0])
            self.send_json(200, {
                "stats": {
                    "devid":         devid,
                    "feed_id":       sess["feed_id"],
                    "recv":          sess["rx_count"],
                    "age":           int((time.time() - sess["last_seen"]) * 1000),
                    "protocol":      sess.get("protocol", 0),
                    "protocol_name": sess.get("protocol_name", "Unknown"),
                },
                "gps":  sess.get("gps", {}),
                "imu":  sess.get("imu", {}),
                "data": data_out,
            })
            return

        self.send_json(404, {"result": "failed", "error": "Not found"})

    def do_POST(self):
        try:
            self._handle_post()
        except Exception:
            traceback.print_exc()
            try:
                self.send_json(500, {"result": "failed", "error": "internal error"})
            except Exception:
                pass

    def _handle_post(self):
        parsed = urlparse(self.path)
        path   = parsed.path.rstrip("/")
        parts  = [p for p in path.split("/") if p]
        length = int(self.headers.get("Content-Length", 0))
        body   = self.rfile.read(length).decode("utf-8", errors="replace") if length else ""

        print("[RAW] {0}".format(repr(body)))

        if "api" in parts:
            parts = parts[parts.index("api"):]

        if len(parts) >= 3 and parts[0] == "api" and parts[1] == "post":
            devid = parts[2]
            sess  = get_session(devid)
            rows  = parse_packed_data(body, devid)
            count = 0
            print("[POST] devid={0} id={1} from={2} pids={3}".format(
                devid, sess["feed_id"], self.client_address[0], len(rows)))
            for _, k, raw_val in rows:
                try:
                    ku = k.strip().upper()

                    if ku == "0":
                        print("  TICK={0}".format(raw_val))
                        count += 1
                        continue

                    if ku == "FF":
                        proto_dec  = int(raw_val) if not isinstance(raw_val, list) else 0
                        proto_name = PROTOCOL_MAP.get(proto_dec, "Unknown")
                        sess["protocol"]      = proto_dec
                        sess["protocol_name"] = proto_name
                        print("  [FF] {0}  ->  {1}".format(proto_dec, proto_name))
                        count += 1
                        continue

                    if ku in GPS_KEYS:
                        gname, gunit, gfn = GPS_KEYS[ku]
                        gval     = gfn(raw_val)
                        unit_str = " " + gunit if gunit else ""
                        print("  [GPS:{0}] {1:<28} = {2}{3}".format(ku, gname, gval, unit_str))
                        sess["gps"][gname] = gval
                        count += 1
                        continue

                    if ku == "20":
                        if isinstance(raw_val, list):
                            print("  [IMU:20] accel={0}".format(raw_val))
                            sess["imu"]["accel"] = raw_val
                        count += 1
                        continue

                    name, val, unit = decode_pid(k, raw_val)
                    unit_str = " " + unit if unit else ""
                    print("  [{0}] {1:<30} = {2}{3}  (raw={4})".format(
                        k, name, val, unit_str, raw_val))
                    sess["data"][k] = val
                    count += 1

                except Exception as e:
                    print("  [WARN] skip key={0} err={1}".format(k, e))
                    continue

            sess["rx_count"] += 1
            print("")
            self.send_json(200, {"result": count})
            return

        self.send_json(404, {"result": "failed", "error": "Not found"})

def run_server(port):
    if not os.path.exists(CERTFILE) or not os.path.exists(KEYFILE):
        print("[ERROR] Certificate files not found!")
        print("        Please ensure {0} and {1} exist in the current directory.".format(CERTFILE, KEYFILE))
        sys.exit(1)

    server = HTTPServer((BIND_IP, port), FreematicsHandler)

    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certfile=CERTFILE, keyfile=KEYFILE)
    server.socket = context.wrap_socket(server.socket, server_side=True)

    print("=" * 60)
    print("[HTTPS Server] Freematics Hub protocol")
    print("=" * 60)
    print("[HTTPS Server] Listening on https://{0}:{1}  (Ctrl+C to stop)".format(BIND_IP, port))
    print("[HTTPS Server] Certificate: {0}".format(CERTFILE))
    print("[HTTPS Server] Key:         {0}".format(KEYFILE))
    print("")
    print("  Endpoints (supports /hub/api/ and /api/ prefix):")
    print("  GET  /hub/api/notify/<DEVID>?EV=1&TS=...&VIN=...")
    print("  GET  /hub/api/push/<DEVID>?TS=...&<PID>=<val>...")
    print("  POST /hub/api/post/<DEVID>")
    print("  GET  /hub/api/channels")
    print("  GET  /hub/api/get/<DEVID>")
    print("-" * 60)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Server] Stopped")
        server.shutdown()

if __name__ == "__main__":
    args = sys.argv[1:]
    port = HTTP_PORT

    if "--port" in args:
        idx = args.index("--port")
        try:
            port = int(args[idx + 1])
        except (IndexError, ValueError):
            print("Usage: python HTTPS_server.py --port <number>")
            sys.exit(1)

    run_server(port)
