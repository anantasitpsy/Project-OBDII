#include <Arduino.h>
#include "driver/twai.h"

#define RX_PIN 27
#define TX_PIN 26

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("--- J1939 Ultimate Decoder ---");
  Serial.printf("Pins: RX=%d, TX=%d\n", RX_PIN, TX_PIN);

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)TX_PIN,
        (gpio_num_t)RX_PIN,
        TWAI_MODE_NO_ACK
      );

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Driver Install FAILED!");
    return;
  }
  Serial.println("Driver installed OK");

  if (twai_start() != ESP_OK) {
    Serial.println("Driver Start FAILED!");
    return;
  }
  Serial.println("Driver started. Watching for Data...");
}

void loop() {
  twai_message_t message;

  if (twai_receive(&message, pdMS_TO_TICKS(1)) != ESP_OK) return;
  if (!message.extd) return;

  unsigned long id  = message.identifier;
  unsigned long pgn = (id >> 8) & 0x3FFFF;
  int src = id & 0xFF;

  Serial.printf("PGN: %lu\tSRC: %d\tDATA: ", pgn, src);
  for (int i = 0; i < message.data_length_code; i++) {
    if (message.data[i] < 0x10) Serial.print("0");
    Serial.print(message.data[i], HEX);
    Serial.print(" ");
  }

  // RPM (Engine Speed) [PGN 61444]
  if (pgn == 61444) {
    float rpm = (message.data[3] + (message.data[4] * 256)) * 0.125;
    Serial.printf(" ==> RPM: %.0f", rpm);
  }

  // Vehicle Speed [PGN 65265]
  else if (pgn == 65265) {
    float speed = (message.data[1] + (message.data[2] * 256)) / 256.0;
    Serial.printf(" ==> Speed: %.1f km/h", speed);
  }

  // Coolant Temperature [PGN 65262]
  else if (pgn == 65262) {
    int temp = message.data[0] - 40;
    Serial.printf(" ==> Coolant Temp: %d C", temp);
  }

  // Fuel Level [PGN 65276]
  else if (pgn == 65276) {
    float fuel = (message.data[1] * 0.4) - 1.7;
    Serial.printf(" ==> Fuel: %.1f %%", fuel);
  }

  // Throttle Pedal Position [PGN 65266]
  else if (pgn == 65266) {
    float throttle = message.data[6] * 0.4;
    Serial.printf(" ==> Throttle Pedal: %.1f %%", throttle);
  }

  // Load [PGN 61443]
  else if (pgn == 61443) {
    float throttle = message.data[1] * 0.4;
    int load = message.data[2];
    Serial.printf(" ==> Load: %d %%", load);
  }

  // Turbo / Intake [PGN 65270]
  else if (pgn == 65270) {
    int boost = message.data[1] * 2;
    int intake = message.data[2] - 40;
    Serial.printf(" ==> Boost: %d kPa | Intake: %d C", boost, intake);
  }

  // Intake Air Temperature [PGN 65269]
  else if (pgn == 65269) {
    int intakeAir = message.data[5] - 40;
    Serial.printf(" ==> Intake Air Temp: %d C", intakeAir);
  }

  // MAF [PGN 61450]
  else if (pgn == 61450) {
    float maf = (message.data[2] + (message.data[3] * 256)) * 0.05;
    Serial.printf(" ==> MAF: %.0f g/s", maf);
  }

  // Manifold Pressure [PGN 65194]
  else if (pgn == 65194) {
    long mapRaw = message.data[1] + (message.data[2] * 256);
    float mapPa = mapRaw * 0.1;
    Serial.printf(" ==> Manifold Pressure: %.1f Pa", mapPa);
  }

  // Battery Voltage [PGN 65271]
  else if (pgn == 65271) {
    float volt = (message.data[4] + (message.data[5] * 256)) * 0.05;
    Serial.printf(" ==> Battery: %.1f V", volt);
  }

  // Ignition Timing [PGN 65159]
  else if (pgn == 65159) {
    int16_t rawValue = (int16_t)((message.data[6] << 8) | message.data[5]);
    float timing = 112.5 + (rawValue / 127.0);

    if (timing > 305.0) {
      timing -= 516.0;
    }

    Serial.printf(" ==> Ignition Timing: %.1f deg", timing);
  }

  // Diagnostic Trouble Codes [PGN 65226]
  else if (pgn == 65226) {
    if (message.data[0] == 0 || message.data[0] == 0xFF)
      Serial.print(" ==> DTC: None (OK)");
    else
      Serial.print(" ==> DTC: Active Faults Found!");
  }

  Serial.println();
}
