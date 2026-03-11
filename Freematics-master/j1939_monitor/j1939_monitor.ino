#include <FreematicsPlus.h>

FreematicsESP32 sys;
COBD obd;

void setup()
{
  Serial.begin(115200);
  while (!sys.begin())
    ;
  obd.begin(sys.link);
  while (!obd.init(PROTO_J1939))
    ;
  Serial.println("OBD:OK Protocol:J1939 data monitor started");
}

void loop()
{
  byte buf[128];
  int bytes = obd.receiveData(buf, sizeof(buf));

  if (bytes > 0)
  {
    if (bytes < 3)
      return; // ป้องกันข้อมูลไม่ครบ

    // คำนวณ PGN จาก 2 Byte แรก
    uint32_t pgn = (uint32_t)buf[0] << 8 | buf[1];

    // --- ส่วนแสดงผล Raw Data (แก้ Loop ให้เริ่มที่ 2) ---
    Serial.print("["); Serial.print(millis()); Serial.print("] ");
    Serial.print("PGN:"); Serial.print(pgn); Serial.print(" Data:");
    // เริ่มที่ n=2 เพื่อให้โชว์ Data 0 (buf[2]) ด้วย
    for (int n = 2; n < bytes; n++) {
      byte d = buf[n];
      if (d < 0x10) Serial.print('0');
      Serial.print(d, HEX);
      Serial.print(' ');
    }

    /* =====================================================
       Decode Section (สูตร: Data[x] --> buf[x+2])
       ===================================================== */

    // RPM [PGN 61444] (Data 3,4 -> buf 5,6)
    if (pgn == 61444)
    {
      float rpm = (buf[5] + (buf[6] * 256)) * 0.125;
      Serial.printf(" ==> RPM: %.0f", rpm);
    }

    // Speed [PGN 65265] (Data 1,2 -> buf 3,4)
    else if (pgn == 65265)
    {
      float speed = (buf[3] + (buf[4] * 256)) / 256.0;
      Serial.printf(" ==> Speed: %.1f km/h", speed);
    }

    // Coolant Temp [PGN 65262] (Data 0 -> buf 2)
    else if (pgn == 65262)
    {
      int temp = buf[2] - 40;
      Serial.printf(" ==> Coolant Temp: %d C", temp);
    }

    // Fuel Level [PGN 65276] (Data 1 -> buf 3)
    else if (pgn == 65276)
    {
      float fuel = (buf[3] * 0.4) - 1.7;
      Serial.printf(" ==> Fuel: %.1f %%", fuel);
    }

    // Throttle Pedal [PGN 65266] (Data 6 -> buf 8)
    else if (pgn == 65266)
    {
      float throttle = buf[8] * 0.4;
      Serial.printf(" ==> Throttle Pedal: %.1f %%", throttle);
    }

    // Load [PGN 61443] (Data 2 -> buf 4)
    else if (pgn == 61443)
    {
      // Throttle (Data 1 -> buf 3)
      // Load (Data 2 -> buf 4)
      int load = buf[4];
      Serial.printf(" ==> Load: %d %%", load);
    }

    // Turbo / Intake [PGN 65270] (Data 1,2 -> buf 3,4)
    else if (pgn == 65270)
    {
      int boost = buf[3] * 2;
      int intake = buf[4] - 40;
      Serial.printf(" ==> Boost: %d kPa | Intake: %d C", boost, intake);
    }

    // Intake Air Temp [PGN 65269] (Data 5 -> buf 7)
    else if (pgn == 65269)
    {
      int intakeAir = buf[7] - 40;
      Serial.printf(" ==> Intake Air Temp: %d C", intakeAir);
    }

    // MAF [PGN 61450] (Data 2,3 -> buf 4,5)
    else if (pgn == 61450)
    {
      float maf = (buf[4] + (buf[5] * 256)) * 0.05;
      Serial.printf(" ==> MAF: %.0f g/s", maf);
    }

    // Manifold Pressure [PGN 65194] (Data 1,2 -> buf 3,4)
    else if (pgn == 65194)
    {
      long mapRaw = buf[3] + (buf[4] * 256);
      float mapPa = mapRaw * 0.1;
      Serial.printf(" ==> Manifold Pressure: %.1f Pa", mapPa);
    }

    // Battery [PGN 65271] (Data 4,5 -> buf 6,7)
    else if (pgn == 65271)
    {
      float volt = (buf[6] + (buf[7] * 256)) * 0.05;
      Serial.printf(" ==> Battery: %.1f V", volt);
    }

    // Ignition Timing [PGN 65159] (Data 5,6 -> buf 7,8)
    else if (pgn == 65159)
    {
      int16_t rawValue = (int16_t)((buf[8] << 8) | buf[7]);
      float timing = 112.5 + (rawValue / 127.0);

      if (timing > 305.0)
      {
        timing -= 516.0;
      }
      Serial.printf(" ==> Ignition Timing: %.1f deg", timing);
    }

    // DTC [PGN 65226] (Data 0 -> buf 2)
    else if (pgn == 65226)
    {
      if (buf[2] == 0 || buf[2] == 0xFF)
        Serial.print(" ==> DTC: None (OK)");
      else
        Serial.print(" ==> DTC: Active Faults Found!");
    }

    Serial.println();
  }
}