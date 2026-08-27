/*
 * ====================================================================
 * GROWATT SPH 10000TL3 BH-UP - Modbus RTU Reader
 * ====================================================================
 *
 * Reads live data from a Growatt SPH hybrid inverter over Modbus RTU
 * (RS485), serves a dashboard and JSON API from a WiFi access point,
 * and switches a relay to dump surplus solar into a water heater.
 *
 * Hardware:
 *  - LILYGO TTGO T-Call V1.3 (ESP32 + SIM800L + IP5306)
 *  - Waveshare TTL <-> RS485 transceiver (A / B screw terminals)
 *  - Relay module with a 5V coil, fed from a separate 5V supply. Its
 *    IN pin is driven straight from 3.3V logic, no series resistor.
 *
 * GPIO mapping:
 *  - GPIO19 (RX2) <-- transceiver RO   [GPIO16/17 taken by PSRAM!]
 *  - GPIO18 (TX2) --> transceiver DI
 *  - GPIO25       --> transceiver DE+RE [GPIO4/5 taken by SIM800!]
 *  - GPIO2        --> relay module IN (3.3V logic, no resistor)
 *  - GPIO13       --> built-in LED (status)   [LED on T-Call]
 *
 * Pins taken by the T-Call board (DO NOT USE):
 *  - GPIO4:  SIM800 PWRKEY
 *  - GPIO5:  SIM800 RST
 *  - GPIO16/17: PSRAM
 *  - GPIO21/22: I2C (IP5306)
 *  - GPIO23: SIM800 POWER_ON
 *  - GPIO26/27: SIM800 UART
 *  - GPIO32/33: SIM800 DTR/RI
 *
 * RS485 to the Growatt - VERIFIED BY MEASUREMENT, 2026-08-19:
 *  - Port: RS485-3   <<< the only one where the inverter acts as a
 *    SLAVE and answers. On the METER and the other ports the inverter
 *    is the MASTER polling the energy meter, so no data can be pulled
 *    from there at all.
 *  - RJ45 pin 5 (white/blue)   -> terminal A  (D+)
 *  - RJ45 pin 1 (white/orange) -> terminal B  (D-)
 *  - RJ45 pin 2 (orange)       -> GND (optional)
 *  - Slave ID 1, 9600 8N1, function 0x04 (Read Input Registers)
 *
 * Web interface:
 *  - WiFi SSID: Growatt-heater
 *  - Password:  NONE - open network (deliberate; it is only joined
 *               occasionally. Anyone in range can switch the SSR!)
 *  - IP:        192.168.10.1
 *  - GET  /              - dashboard
 *  - GET  /api/data      - JSON state
 *  - POST /api/ssr/on    - force SSR on
 *  - POST /api/ssr/off   - force SSR off
 *  - POST /api/ssr/auto  - surplus mode
 *  - POST /api/limits?pv=330&soc=90 - change thresholds at runtime
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 @Oliwer11
 * ====================================================================
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <string.h>
#include <math.h>
// Note: ModbusMaster is deliberately NOT used here. The transceiver
// echoes back the tail of our own transmission and the library reads
// that as a reply from a foreign address (0xE0). The implementation
// below instead collects the whole stream and locates a valid frame
// inside it by CRC - exactly what the sniffer sketch does, the one
// the register map was verified with.

// ============ CONFIGURATION ============

// WiFi - the AP is INTENTIONALLY open, no password. It is only joined
// occasionally and the range covers the owner's own yard. Consequence:
// whoever connects can switch the SSR and change the thresholds. To
// bring the password back, pass it as the second argument of
// WiFi.softAP().
const char *WIFI_SSID = "Growatt-heater";

// GPIO pins (T-Call V1.3 - see the pin map in the header!)
#define MAX485_DE_RE_PIN 25  // RS485 direction control (DE+RE tied together)
#define SSR_PIN 2            // relay module IN (name kept from the SSR design)
#define LED_PIN 13           // built-in LED on the T-Call V1.3

// Modbus
#define MODBUS_SLAVE_ID 1  // Growatt default slave ID
#define MODBUS_BAUD 9600
#define MODBUS_RX_PIN 19  // UART2 remapped (GPIO16/17 = PSRAM!)
#define MODBUS_TX_PIN 18  // UART2 remapped

// Register ranges - EXACTLY THE ONES THE SNIFFER VERIFIED. Do not
// change them without trying the new range with the sniffer first;
// the inverter does not necessarily serve an arbitrary block.
#define BLOCK1_START 0     // PV: status, power, string voltages and currents
#define BLOCK1_COUNT 17    // 0-16
#define BLOCK2_START 1000  // battery, grid flow, house load
#define BLOCK2_COUNT 41    // 1000-1040

// Optional block - Pac/Fac/Vac. Taken from the ha-growatt-modbus map
// and NOT verified, so it is read separately: if it fails, the main
// data is unaffected.
#define BLOCK3_START 35
#define BLOCK3_COUNT 4     // 35-38

// Custom Modbus master
#define MB_RESP_TIMEOUT_MS 600  // how long to wait for a reply
#define MB_RX_BUF 200           // 41 registers = 87 B reply + echo

// Timing
#define READ_INTERVAL_MS 30000  // poll every 30 s
#define MODBUS_DELAY_MS 1000    // gap between Modbus blocks (Growatt needs >850 ms)
#define WATCHDOG_MS 300000      // 5 min watchdog (turns the SSR off after a comms loss)

// ============ SSR THRESHOLDS ============
//
// AUTO mode watches TWO things only: the voltage of both PV strings
// and the battery state of charge. Both conditions must hold AT THE
// SAME TIME, otherwise the output stays off.
//
//   BOTH strings > ssrMinPvVoltage   and   SOC > ssrMinSoc  ->  ON
//   any condition stops holding                             ->  OFF
//
// Evaluated every SSR_CHECK_INTERVAL_MS, i.e. 60 s. Modbus is polled
// every 30 s, so the decision is made from every second reading.
//
// The thresholds can be changed from the dashboard (POST /api/limits),
// but only AT RUNTIME - they are not persisted to flash, so after a
// power cut the system comes back on these defaults. That was the
// requested behaviour.
#define SSR_DEFAULT_PV_VOLTAGE 330  // V, default string voltage threshold
#define SSR_DEFAULT_SOC 90          // %, default battery charge threshold

// Range accepted when changing the thresholds from the UI. This check
// is the binding one - browser side JS can be bypassed, and these two
// numbers decide whether 230 V gets switched. Never widen the lower
// bound to a point where the SSR could run on a nearly empty battery.
#define SSR_PV_VOLTAGE_MIN 300  // V
#define SSR_PV_VOLTAGE_MAX 400  // V
#define SSR_SOC_MIN 80          // %
#define SSR_SOC_MAX 95          // %

// So the same bounds can be embedded in the /api/limits error text -
// two-step stringify, otherwise the macro would not be expanded.
#define SSR_STR_HELPER(x) #x
#define SSR_STR(x) SSR_STR_HELPER(x)

float ssrMinPvVoltage = SSR_DEFAULT_PV_VOLTAGE;  // V, BOTH strings must exceed this
uint16_t ssrMinSoc = SSR_DEFAULT_SOC;            // %, battery must be ABOVE this

#define SSR_CHECK_INTERVAL_MS 60000      // how often to evaluate (ms)
#define SURPLUS_WATCHDOG_MS 90000        // AUTO: turn off after 90 s without comms

// ============ GLOBALS ============

HardwareSerial RS485Serial(2);  // UART2 remapped to GPIO18/19
AsyncWebServer server(80);

// The block just read lands here. Access goes through regAt()/regPair(),
// which take an ABSOLUTE register address - not an offset into the reply.
uint16_t regBuf[64];
uint16_t regBufStart = 0;
uint16_t regBufCount = 0;

struct GrowattData {
  // --- BLOCK 0-40, verified ---
  uint16_t inverterStatus = 0;
  String inverterStatusText = "Unknown";
  float pvTotalPower = 0;  // reg 1-2    W
  float pv1Voltage = 0;    // reg 3      V
  float pv1Current = 0;    // reg 4      A
  float pv1Power = 0;      // reg 5-6    W
  float pv2Voltage = 0;    // reg 7      V
  float pv2Current = 0;    // reg 8      A
  float pv2Power = 0;      // reg 9-10   W

  // --- BLOCK 35-38, from the ha-growatt-modbus map, NOT VERIFIED YET ---
  bool acAvailable = false;
  float acOutputPower = 0;  // reg 35-36  W
  float frequency = 0;      // reg 37     Hz
  float gridVoltage = 0;    // reg 38     V

  // --- BLOCK 1000-1040, verified against the energy balance ---
  float batteryDischarge = 0;  // reg 1009-1010  W
  float batteryCharge = 0;     // reg 1011-1012  W
  float batteryVoltage = 0;    // reg 1013       V
  uint16_t batterySOC = 0;     // reg 1014       %
  float powerFromGrid = 0;     // reg 1021-1022  W (import)
  float powerToGrid = 0;       // reg 1029-1030  W (export = SURPLUS)
  float loadConsumption = 0;   // reg 1037-1038  W
  float batteryTemp = 0;       // reg 1040       degC
  uint16_t systemMode = 0;     // reg 1000

  // Derived
  float batteryPower = 0;      // + charging, - discharging
  String batteryDirection = "idle";
  float balanceIn = 0, balanceOut = 0;  // consistency check

  // Communication status
  unsigned long lastUpdate = 0;
  bool isOnline = false;
  bool block2Available = false;  // did 1000-1040 come through?
  uint16_t lastError = 0;
  uint32_t totalReads = 0;
  uint32_t totalErrors = 0;
};

GrowattData growatt;

enum SSRMode {
  SSR_OFF,          // always off
  SSR_ON,           // always on
  SSR_AUTO_ON,      // AUTO: SSR on, next check in 60 s
  SSR_AUTO_OFF      // AUTO: SSR off, next attempt in 60 s
};
SSRMode ssrMode = SSR_OFF;
bool ssrState = false;
unsigned long ssrCheckStart = 0;

unsigned long lastSuccessfulRead = 0;
unsigned long lastReadAttempt = 0;
unsigned long bootTime = 0;

// ============ RS485 DIRECTION CONTROL ============

void preTransmission() {
  digitalWrite(MAX485_DE_RE_PIN, HIGH);
  delayMicroseconds(50);
}

void postTransmission() {
  // CRITICAL: Serial.flush() on the ESP32 returns before the last bits
  // have actually left the UART. If DE drops too early the tail of the
  // frame never reaches the bus, the slave receives a truncated frame
  // and DOES NOT ANSWER. This is exactly what we measured - the only
  // thing in the receive buffer was the tail of our own transmission.
  // Wait two byte times (1 byte = 10 bits).
  delayMicroseconds((10UL * 1000000UL / MODBUS_BAUD) * 2);
  digitalWrite(MAX485_DE_RE_PIN, LOW);
}

// ============ CUSTOM MODBUS RTU MASTER ============

uint16_t modbusCRC(const uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

const char *modbusError(uint8_t code) {
  switch (code) {
    case 0x00: return "OK";
    case 0x01: return "Illegal Function";
    case 0x02: return "Illegal Data Address - inverter has no such range";
    case 0x03: return "Illegal Data Value";
    case 0x04: return "Slave Device Failure";
    case 0xE2: return "SILENCE - no reply at all";
    case 0xE3: return "data arrived, but no valid frame in it";
    default: return "unknown error";
  }
}

uint8_t mbRx[MB_RX_BUF];

// Reads input registers (fn 0x04) into regBuf[]. Returns 0 on success.
//
// The key difference from an off-the-shelf library: the transceiver
// echoes back the tail of our own transmission, so the FIRST bytes
// received are NOT the reply. Everything is collected first and the
// reply is then located inside it by CRC.
uint8_t readInputBlock(uint16_t start, uint16_t count) {
  if (count > 64) return 0x03;

  uint8_t req[8];
  req[0] = MODBUS_SLAVE_ID;
  req[1] = 0x04;
  req[2] = start >> 8;
  req[3] = start & 0xFF;
  req[4] = count >> 8;
  req[5] = count & 0xFF;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  while (RS485Serial.available()) RS485Serial.read();

  preTransmission();
  RS485Serial.write(req, 8);
  RS485Serial.flush();
  postTransmission();

  const uint16_t expLen = 5 + count * 2;
  uint16_t len = 0;
  unsigned long t0 = millis();

  while (millis() - t0 < MB_RESP_TIMEOUT_MS) {
    while (RS485Serial.available() && len < MB_RX_BUF) {
      mbRx[len++] = RS485Serial.read();
    }
    if (len < expLen) continue;

    // Look for a valid reply inside whatever has been collected
    for (uint16_t i = 0; i + expLen <= len; i++) {
      if (mbRx[i] != MODBUS_SLAVE_ID || mbRx[i + 1] != 0x04) continue;
      if (mbRx[i + 2] != count * 2) continue;
      uint16_t calc = modbusCRC(&mbRx[i], expLen - 2);
      uint16_t recv = mbRx[i + expLen - 2] | ((uint16_t)mbRx[i + expLen - 1] << 8);
      if (calc != recv) continue;

      regBufStart = start;
      regBufCount = count;
      for (uint16_t k = 0; k < count; k++) {
        regBuf[k] = ((uint16_t)mbRx[i + 3 + k * 2] << 8) | mbRx[i + 4 + k * 2];
      }
      return 0x00;
    }
  }

  // An exception from the inverter is a short frame. It is searched for
  // only after no reply was found, so that a 5 byte stretch of payload
  // is not mistaken for an exception.
  for (uint16_t i = 0; i + 5 <= len; i++) {
    if (mbRx[i] != MODBUS_SLAVE_ID || mbRx[i + 1] != 0x84) continue;
    uint16_t calc = modbusCRC(&mbRx[i], 3);
    uint16_t recv = mbRx[i + 3] | ((uint16_t)mbRx[i + 4] << 8);
    if (calc == recv) return mbRx[i + 2];
  }

  return (len == 0) ? 0xE2 : 0xE3;
}

// ============ HELPERS ============

uint32_t combineRegisters(uint16_t high, uint16_t low) {
  return ((uint32_t)high << 16) | low;
}

// These take an ABSOLUTE register address and work out the offset into
// the reply themselves - that is the trap it is easy to get wrong.
uint16_t regAt(uint16_t addr) {
  uint16_t idx = addr - regBufStart;
  return (addr >= regBufStart && idx < regBufCount) ? regBuf[idx] : 0;
}

float regPair(uint16_t addr) {
  return combineRegisters(regAt(addr), regAt(addr + 1)) * 0.1;
}

// The SPH status enumeration has not been verified against the display.
// Value 5 is what the inverter reports during normal operation while
// charging the battery.
String getStatusText(uint16_t status) {
  switch (status) {
    case 0: return "Waiting";
    case 1: return "Self-test";
    case 3: return "FAULT";
    case 4: return "Flash / update";
    case 5: return "Running (PV + battery)";
    case 6: return "Running (battery)";
    default: return "Status " + String(status);
  }
}

void blinkLED(int times, int delayMs = 100) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}

// ============ MODBUS READS ============

// BLOCK 1: registers 0-16. Verified two ways - the sum of the string
// powers equals total PV power, and U*I equals the string power.
bool readBlockPV() {
  uint8_t result = readInputBlock(BLOCK1_START, BLOCK1_COUNT);

  if (result != 0x00) {
    Serial.printf("[ERR] Block %d-%d failed: 0x%02X (%s)\n",
                  BLOCK1_START, BLOCK1_START + BLOCK1_COUNT - 1,
                  result, modbusError(result));
    growatt.lastError = result;
    return false;
  }

  growatt.inverterStatus = regAt(0);
  growatt.inverterStatusText = getStatusText(growatt.inverterStatus);

  growatt.pvTotalPower = regPair(1);
  growatt.pv1Voltage = regAt(3) * 0.1;
  growatt.pv1Current = regAt(4) * 0.1;
  growatt.pv1Power = regPair(5);
  growatt.pv2Voltage = regAt(7) * 0.1;
  growatt.pv2Current = regAt(8) * 0.1;
  growatt.pv2Power = regPair(9);

  return true;
}

// BLOCK 3 (optional): Pac, Fac, Vac. From the ha-growatt-modbus map,
// NOT VERIFIED. A failure here is not treated as an error - the main
// data is available without it.
void readBlockAC() {
  if (readInputBlock(BLOCK3_START, BLOCK3_COUNT) != 0x00) {
    growatt.acAvailable = false;
    return;
  }
  growatt.acAvailable = true;
  growatt.acOutputPower = regPair(35);
  growatt.frequency = regAt(37) * 0.01;
  growatt.gridVoltage = regAt(38) * 0.1;
}

// BLOCK 2: registers 1000-1040. Verified against the energy balance -
// PV power matches battery charging plus house load exactly.
bool readBlockStorage() {
  uint8_t result = readInputBlock(BLOCK2_START, BLOCK2_COUNT);

  if (result != 0x00) {
    Serial.printf("[ERR] Block %d-%d failed: 0x%02X (%s)\n",
                  BLOCK2_START, BLOCK2_START + BLOCK2_COUNT - 1,
                  result, modbusError(result));
    growatt.block2Available = false;
    return false;
  }

  growatt.block2Available = true;

  growatt.systemMode = regAt(1000);
  growatt.batteryDischarge = regPair(1009);
  growatt.batteryCharge = regPair(1011);
  growatt.batteryVoltage = regAt(1013) * 0.1;
  growatt.batterySOC = regAt(1014);
  growatt.powerFromGrid = regPair(1021);
  growatt.powerToGrid = regPair(1029);
  growatt.loadConsumption = regPair(1037);
  growatt.batteryTemp = regAt(1040) * 0.1;

  // Positive = charging, negative = discharging
  growatt.batteryPower = growatt.batteryCharge - growatt.batteryDischarge;
  if (growatt.batteryPower > 50) growatt.batteryDirection = "charging";
  else if (growatt.batteryPower < -50) growatt.batteryDirection = "discharging";
  else growatt.batteryDirection = "idle";

  // Balance: what flows into the house must equal what flows out of it.
  // If the two drift apart, the register map is wrong and none of the
  // values can be trusted.
  growatt.balanceIn = growatt.pvTotalPower + growatt.batteryDischarge + growatt.powerFromGrid;
  growatt.balanceOut = growatt.loadConsumption + growatt.batteryCharge + growatt.powerToGrid;

  return true;
}

bool readGrowattData() {
  growatt.totalReads++;

  Serial.println("\n--- Reading Growatt ---");

  // Block 1: PV (mandatory). If it fails we are offline.
  if (!readBlockPV()) {
    growatt.totalErrors++;
    growatt.isOnline = false;
    return false;
  }

  // Gap before the next request (Growatt needs at least 850 ms)
  delay(MODBUS_DELAY_MS);

  // Block 2: battery and grid. AUTO mode cannot decide without it.
  readBlockStorage();

  delay(MODBUS_DELAY_MS);

  // Block 3: extra AC values. Nothing breaks if it fails.
  readBlockAC();

  // Update timestamps
  growatt.lastUpdate = millis();
  growatt.isOnline = true;
  lastSuccessfulRead = millis();

  // LED blink on a successful read
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);

  return true;
}

const char *ssrModeText() {
  switch (ssrMode) {
    case SSR_ON: return "ON (manual)";
    case SSR_AUTO_ON: return "AUTO - on";
    case SSR_AUTO_OFF: return "AUTO - waiting for surplus";
    default: return "OFF";
  }
}

void printDataToSerial() {
  Serial.println("=====================================================");
  Serial.printf("[STATE]   %s   (reg 0 = %u)\n",
                growatt.inverterStatusText.c_str(), growatt.inverterStatus);

  Serial.println("--- STRINGS (registers 0-16, verified) ---");
  Serial.printf("  PV total      %8.1f W\n", growatt.pvTotalPower);
  Serial.printf("  MPPT1         %8.1f W   %6.1f V  %5.1f A\n",
                growatt.pv1Power, growatt.pv1Voltage, growatt.pv1Current);
  Serial.printf("  MPPT2         %8.1f W   %6.1f V  %5.1f A\n",
                growatt.pv2Power, growatt.pv2Voltage, growatt.pv2Current);
  // Map check: the string powers must add up to the total PV power
  {
    float sum = growatt.pv1Power + growatt.pv2Power;
    float d = fabs(sum - growatt.pvTotalPower);
    Serial.printf("  check         MPPT1+MPPT2 = %.1f W  (diff %.1f W) %s\n",
                  sum, d,
                  (growatt.pvTotalPower > 100.0 && d > growatt.pvTotalPower * 0.05)
                    ? "!!! MISMATCH" : "OK");
  }

  if (growatt.acAvailable) {
    Serial.println("--- AC (registers 35-38, NOT VERIFIED) ---");
    Serial.printf("  inverter out  %8.1f W\n", growatt.acOutputPower);
    Serial.printf("  frequency     %8.2f Hz\n", growatt.frequency);
    Serial.printf("  grid voltage  %8.1f V\n", growatt.gridVoltage);
  } else {
    Serial.println("--- AC (registers 35-38) unavailable ---");
  }

  if (growatt.block2Available) {
    Serial.println("--- BATTERY (registers 1009-1014, 1040) ---");
    Serial.printf("  SOC           %8u %%\n", growatt.batterySOC);
    Serial.printf("  voltage       %8.1f V\n", growatt.batteryVoltage);
    Serial.printf("  charging      %8.1f W\n", growatt.batteryCharge);
    Serial.printf("  discharging   %8.1f W\n", growatt.batteryDischarge);
    Serial.printf("  direction     %8s\n", growatt.batteryDirection.c_str());
    Serial.printf("  temperature   %8.1f C\n", growatt.batteryTemp);

    Serial.println("--- GRID AND HOUSE (registers 1021-1038) ---");
    Serial.printf("  import        %8.1f W\n", growatt.powerFromGrid);
    Serial.printf("  EXPORT        %8.1f W   <<< surplus for the SSR\n", growatt.powerToGrid);
    Serial.printf("  house load    %8.1f W\n", growatt.loadConsumption);

    Serial.println("--- ENERGY BALANCE ---");
    Serial.printf("  IN   PV %.0f + discharge %.0f + import %.0f = %.0f W\n",
                  growatt.pvTotalPower, growatt.batteryDischarge,
                  growatt.powerFromGrid, growatt.balanceIn);
    Serial.printf("  OUT  house %.0f + charge %.0f + export %.0f = %.0f W\n",
                  growatt.loadConsumption, growatt.batteryCharge,
                  growatt.powerToGrid, growatt.balanceOut);
    {
      float d = fabs(growatt.balanceIn - growatt.balanceOut);
      float ref = max(growatt.balanceIn, growatt.balanceOut);
      Serial.printf("  diff %.0f W  %s\n", d,
                    (ref > 100.0 && d > ref * 0.05) ? "!!! MISMATCH - map is wrong" : "OK");
    }
  } else {
    Serial.println("--- BLOCK 1000-1040 UNAVAILABLE - AUTO mode cannot decide ---");
  }

  Serial.println("--- SSR ---");
  Serial.printf("  output        %8s   mode: %s\n",
                ssrState ? "ON" : "off", ssrModeText());
  Serial.printf("  volt.thresh.  %8.1f V    MPPT1 %.1f %s   MPPT2 %.1f %s\n",
                ssrMinPvVoltage,
                growatt.pv1Voltage, growatt.pv1Voltage > ssrMinPvVoltage ? "OK" : "LOW",
                growatt.pv2Voltage, growatt.pv2Voltage > ssrMinPvVoltage ? "OK" : "LOW");
  Serial.printf("  batt. thresh. %8u %%    SOC   %u %s\n",
                ssrMinSoc, growatt.batterySOC,
                growatt.batterySOC > ssrMinSoc ? "OK" : "LOW");

  Serial.printf("[STAT]    reads=%u, errors=%u, uptime=%lu s\n",
                growatt.totalReads, growatt.totalErrors,
                (millis() - bootTime) / 1000);
  Serial.println("=====================================================");
}

// ============ SSR LOGIC ============

void setSSR(bool on) {
  ssrState = on;
  digitalWrite(SSR_PIN, on ? HIGH : LOW);
}

void updateSSR() {
  // Watchdog 1: drop AUTO after 90 s without a successful read.
  // A manual SSR_ON deliberately survives - that is the operator's
  // conscious decision.
  if ((ssrMode == SSR_AUTO_ON || ssrMode == SSR_AUTO_OFF) &&
      millis() - lastSuccessfulRead > SURPLUS_WATCHDOG_MS) {
    Serial.println("[AUTO] No communication for 90 s - dropping AUTO");
    setSSR(false);
    ssrMode = SSR_OFF;
    return;
  }

  // Watchdog 2: long outage - a backstop in case the first one is missed
  if (millis() - lastSuccessfulRead > WATCHDOG_MS) {
    if (ssrMode == SSR_AUTO_ON || ssrMode == SSR_AUTO_OFF) {
      Serial.println("[WDT] 5 min watchdog - dropping AUTO");
      setSSR(false);
      ssrMode = SSR_OFF;
    }
    return;
  }

  if (!growatt.isOnline) return;

  // Without block 1000-1040 there is no battery SOC and AUTO has
  // nothing to decide on. Fail-safe: turn off.
  if ((ssrMode == SSR_AUTO_ON || ssrMode == SSR_AUTO_OFF) && !growatt.block2Available) {
    Serial.println("[AUTO] Block 1000-1040 missing - dropping AUTO");
    setSSR(false);
    ssrMode = SSR_OFF;
    return;
  }

  switch (ssrMode) {

    case SSR_OFF:
    case SSR_ON:
      // Manual modes - no automation
      break;

    case SSR_AUTO_ON:
    case SSR_AUTO_OFF:
      // The same evaluation for both states - the condition is
      // symmetric, so two separate branches are not needed. While it
      // holds the water heats; the moment it stops holding, off it goes.
      if (millis() - ssrCheckStart >= SSR_CHECK_INTERVAL_MS) {
        ssrCheckStart = millis();

        bool pv1Ok = (growatt.pv1Voltage > ssrMinPvVoltage);
        bool pv2Ok = (growatt.pv2Voltage > ssrMinPvVoltage);
        bool socOk = (growatt.batterySOC > ssrMinSoc);
        bool allOk = pv1Ok && pv2Ok && socOk;

        Serial.printf("[AUTO] thresholds %.0f V / %u %% | MPPT1 %.1f V %s | MPPT2 %.1f V %s | SOC %u %% %s\n",
                      ssrMinPvVoltage, ssrMinSoc,
                      growatt.pv1Voltage, pv1Ok ? "OK" : "LOW",
                      growatt.pv2Voltage, pv2Ok ? "OK" : "LOW",
                      growatt.batterySOC, socOk ? "OK" : "LOW");

        if (allOk && ssrMode == SSR_AUTO_OFF) {
          Serial.println("[AUTO] Conditions met - SWITCHING ON");
          setSSR(true);
          ssrMode = SSR_AUTO_ON;
        } else if (!allOk && ssrMode == SSR_AUTO_ON) {
          Serial.println("[AUTO] Condition no longer holds - SWITCHING OFF");
          setSSR(false);
          ssrMode = SSR_AUTO_OFF;
        }
      }
      break;
  }
}

// ============ WEB INTERFACE ============

const char HTML_PAGE[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Growatt Heater Monitor</title>
<style>
  * { box-sizing: border-box; }
  body {
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
    background: linear-gradient(135deg, #0d1117 0%, #1a2332 100%);
    color: #e6edf3;
    margin: 0;
    padding: 20px;
    min-height: 100vh;
  }
  .container { max-width: 1100px; margin: auto; }
  h1 {
    color: #58a6ff;
    margin: 0 0 8px 0;
    font-size: 22px;
  }
  .subtitle { color: #8b949e; margin-bottom: 24px; font-size: 14px; }
  .status-bar {
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
    margin-bottom: 24px;
    align-items: center;
  }
  .badge {
    padding: 6px 14px;
    border-radius: 16px;
    font-size: 13px;
    font-weight: 600;
    letter-spacing: 0.3px;
  }
  .badge.online { background: #1f6f3d; color: #4ade80; }
  .badge.offline { background: #6f1f1f; color: #f87171; }
  .badge.ssr-on  { background: #6f4f1f; color: #fbbf24; }
  .badge.ssr-off { background: #30363d; color: #8b949e; }
  .badge.ssr-auto { background: #1f4f2f; color: #4ade80; }
  .badge.info { background: #1f4f6f; color: #60a5fa; }
  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
    gap: 14px;
    margin-bottom: 20px;
  }
  .card {
    background: rgba(22, 27, 34, 0.7);
    border: 1px solid #30363d;
    border-radius: 10px;
    padding: 16px;
    backdrop-filter: blur(10px);
    transition: transform 0.2s, border-color 0.2s;
  }
  .card:hover { transform: translateY(-2px); border-color: #58a6ff; }
  .label {
    font-size: 11px;
    color: #8b949e;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    margin-bottom: 4px;
  }
  .value {
    font-size: 26px;
    font-weight: 700;
    color: #e6edf3;
  }
  .unit { font-size: 14px; color: #8b949e; font-weight: 400; margin-left: 4px; }
  .pos { color: #4ade80; }
  .neg { color: #f87171; }
  .warning { color: #fbbf24; }
  .section-title {
    font-size: 15px;
    color: #8b949e;
    text-transform: uppercase;
    letter-spacing: 1px;
    margin: 24px 0 8px 0;
    padding-bottom: 6px;
    border-bottom: 1px solid #30363d;
  }
  .controls {
    background: rgba(22, 27, 34, 0.7);
    border: 1px solid #30363d;
    border-radius: 10px;
    padding: 16px;
    margin-top: 20px;
  }
  .btn {
    background: #238636;
    color: white;
    border: none;
    padding: 10px 20px;
    border-radius: 6px;
    cursor: pointer;
    font-size: 14px;
    font-weight: 600;
    margin-right: 8px;
    transition: background 0.2s;
  }
  .btn:hover { background: #2ea043; }
  .btn.danger { background: #da3633; }
  .btn.danger:hover { background: #f85149; }
  .btn.auto { background: #1a7f37; }
  .btn.auto:hover { background: #2ea043; }
  .limits {
    display: flex;
    flex-wrap: wrap;
    gap: 16px;
    align-items: flex-end;
  }
  .limit-item { display: flex; flex-direction: column; gap: 6px; }
  .limit-item label {
    font-size: 11px;
    color: #8b949e;
    text-transform: uppercase;
    letter-spacing: 0.5px;
  }
  .limit-item input {
    background: #0d1117;
    border: 1px solid #30363d;
    border-radius: 6px;
    color: #e6edf3;
    font-size: 20px;
    font-weight: 700;
    padding: 8px 12px;
    width: 110px;
    text-align: center;
  }
  .limit-item input:focus { outline: none; border-color: #58a6ff; }
  .limit-msg { margin-top: 12px; font-size: 13px; min-height: 18px; }
  @media (max-width: 600px) {
    .controls { display: flex; flex-direction: column; gap: 10px; }
    .btn { width: 100%; margin-right: 0; padding: 14px; font-size: 16px; }
    .limits { flex-direction: column; align-items: stretch; }
    .limit-item input { width: 100%; padding: 14px; }
  }
  .footer {
    margin-top: 30px;
    padding-top: 16px;
    border-top: 1px solid #30363d;
    color: #8b949e;
    font-size: 12px;
    text-align: center;
  }
</style>
</head>
<body>
<div class="container">
  <h1>Growatt SPH 10000 BH-UP</h1>
  <div class="subtitle">Modbus Reader + SSR Heater Control</div>

  <div class="status-bar" id="statusBar"></div>

  <div class="controls">
    <div class="label" style="margin-bottom: 12px;">SSR control</div>
    <button class="btn" onclick="ssrControl('on')">🔥 Turn on</button>
    <button class="btn danger" onclick="ssrControl('off')">⛔ Turn off</button>
    <button class="btn auto" onclick="ssrControl('auto')">♻️ Surplus mode</button>
  </div>

  <div class="section-title">SSR conditions (evaluated every 60 s)</div>

  <div class="controls">
    <div class="label" style="margin-bottom: 12px;">
      Switching thresholds &mdash; kept until reboot only, then back to defaults
    </div>
    <div class="limits">
      <div class="limit-item">
        <label for="inPv" id="lblPv">String voltage</label>
        <input id="inPv" type="text" inputmode="numeric" maxlength="3"
               oninput="onlyDigits(this)" onkeydown="if(event.key==='Enter')saveLimits()">
      </div>
      <div class="limit-item">
        <label for="inSoc" id="lblSoc">Battery charge</label>
        <input id="inSoc" type="text" inputmode="numeric" maxlength="2"
               oninput="onlyDigits(this)" onkeydown="if(event.key==='Enter')saveLimits()">
      </div>
      <button class="btn" onclick="saveLimits()">💾 Save thresholds</button>
    </div>
    <div class="limit-msg" id="limitMsg"></div>
  </div>

  <div class="grid" id="ssrGrid"></div>

  <div class="section-title">Solar production</div>
  <div class="grid" id="solarGrid"></div>

  <div class="section-title">AC output (unverified registers)</div>
  <div class="grid" id="acGrid"></div>

  <div class="section-title">Grid and house load</div>
  <div class="grid" id="gridGrid"></div>

  <div class="section-title">Battery</div>
  <div class="grid" id="batteryGrid"></div>

  <div class="section-title">Energy balance (register map check)</div>
  <div class="grid" id="statsGrid"></div>

  <div class="footer">
    <span id="updateTime">Loading...</span> |
    Uptime: <span id="uptime">-</span> |
    Reads: <span id="reads">0</span> / Errors: <span id="errors">0</span>
  </div>
</div>

<script>
function formatUptime(seconds) {
  const d = Math.floor(seconds / 86400);
  const h = Math.floor((seconds % 86400) / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  if (d > 0) return d + 'd ' + h + 'h ' + m + 'm';
  if (h > 0) return h + 'h ' + m + 'm';
  return m + 'm';
}

function card(label, value, unit, cls) {
  cls = cls || '';
  const v = (typeof value === 'number') ? value.toFixed(1) : value;
  return `<div class="card">
    <div class="label">${label}</div>
    <div class="value ${cls}">${v} <span class="unit">${unit}</span></div>
  </div>`;
}

// Accepted ranges. These values are only a pre-fill - the first
// /api/data overwrites them with the real bounds from the firmware so
// there is a single source of truth. The binding check is on the ESP32
// either way.
let limits = { pvMin: 300, pvMax: 400, socMin: 80, socMax: 95 };

// The field must hold digits only. type="number" cannot limit the
// digit count, hence text + a filter: strings 3 digits, battery 2
// (maxlength in the HTML).
function onlyDigits(el) {
  el.value = el.value.replace(/[^0-9]/g, '');
}

function showLimitMsg(text, ok) {
  const m = document.getElementById('limitMsg');
  m.textContent = text;
  m.className = 'limit-msg ' + (ok ? 'pos' : 'neg');
}

// Refresh runs every 5 s - it must not overwrite what is being typed,
// so a field is skipped while it holds the caret.
function fillLimits(d) {
  limits = { pvMin: d.ssrPvMin, pvMax: d.ssrPvMax,
             socMin: d.ssrSocMin, socMax: d.ssrSocMax };

  document.getElementById('lblPv').textContent =
    `String voltage (${limits.pvMin}-${limits.pvMax} V)`;
  document.getElementById('lblSoc').textContent =
    `Battery charge (${limits.socMin}-${limits.socMax} %)`;

  const pv = document.getElementById('inPv');
  const soc = document.getElementById('inSoc');
  if (document.activeElement !== pv)  pv.value  = Math.round(d.ssrMinPvVoltage);
  if (document.activeElement !== soc) soc.value = d.ssrMinSoc;
}

async function saveLimits() {
  const pv  = parseInt(document.getElementById('inPv').value, 10);
  const soc = parseInt(document.getElementById('inSoc').value, 10);

  if (!(pv >= limits.pvMin && pv <= limits.pvMax)) {
    showLimitMsg(`String voltage must be between ${limits.pvMin} and ${limits.pvMax} V`, false);
    return;
  }
  if (!(soc >= limits.socMin && soc <= limits.socMax)) {
    showLimitMsg(`Battery charge must be between ${limits.socMin} and ${limits.socMax} %`, false);
    return;
  }

  try {
    const r = await fetch(`/api/limits?pv=${pv}&soc=${soc}`, { method: 'POST' });
    const res = await r.json();
    if (r.ok && res.success) {
      showLimitMsg(`Saved: ${pv} V / ${soc} % (evaluated immediately)`, true);
      refresh();
    } else {
      showLimitMsg('Error: ' + (res.error || r.statusText), false);
    }
  } catch(e) {
    showLimitMsg('Error: ' + e.message, false);
  }
}

async function refresh() {
  try {
    const r = await fetch('/api/data');
    const d = await r.json();

    // Status bar
    const statusHtml = `
      <span class="badge ${d.isOnline ? 'online' : 'offline'}">${d.isOnline ? '● ONLINE' : '● OFFLINE'}</span>
      <span class="badge ${d.ssrMode === 'on' ? 'ssr-on' : (d.ssrMode === 'auto_on' || d.ssrMode === 'auto_off') ? 'ssr-auto' : 'ssr-off'}">
        SSR ${d.ssrMode === 'on' ? '🔥 ON' :
              d.ssrMode === 'auto_on'  ? '♻️ AUTO: on (check in ' + d.ssrNextCheck + 's)' :
              d.ssrMode === 'auto_off' ? '♻️ AUTO: off (retry in ' + d.ssrNextCheck + 's)' :
              '⛔ OFF'}
      </span>
      <span class="badge info">${d.inverterStatusText}</span>
      ${!d.block2Available ? '<span class="badge offline">⚠ Block 1000-1040 unavailable</span>' : ''}
    `;
    document.getElementById('statusBar').innerHTML = statusHtml;

    // Thresholds into the fields (and ranges into the labels)
    fillLimits(d);

    // SSR conditions - green = met, red = not met.
    // All three must hold at the same time.
    const pv1Ok = d.pv1Voltage > d.ssrMinPvVoltage;
    const pv2Ok = d.pv2Voltage > d.ssrMinPvVoltage;
    const socOk = d.batterySOC > d.ssrMinSoc;
    document.getElementById('ssrGrid').innerHTML =
      card('MPPT1 voltage', d.pv1Voltage, 'V &gt; ' + d.ssrMinPvVoltage, pv1Ok ? 'pos' : 'neg') +
      card('MPPT2 voltage', d.pv2Voltage, 'V &gt; ' + d.ssrMinPvVoltage, pv2Ok ? 'pos' : 'neg') +
      card('Battery charge', d.batterySOC, '% &gt; ' + d.ssrMinSoc, socOk ? 'pos' : 'neg') +
      card('Result', (pv1Ok && pv2Ok && socOk) ? 'MET' : 'NOT MET', '',
           (pv1Ok && pv2Ok && socOk) ? 'pos' : 'neg');

    // Strings - the verified part of the map
    document.getElementById('solarGrid').innerHTML =
      card('PV total', d.pvTotalPower, 'W', 'pos') +
      card('MPPT1 power', d.pv1Power, 'W') +
      card('MPPT1 voltage', d.pv1Voltage, 'V') +
      card('MPPT1 current', d.pv1Current, 'A') +
      card('MPPT2 power', d.pv2Power, 'W') +
      card('MPPT2 voltage', d.pv2Voltage, 'V') +
      card('MPPT2 current', d.pv2Current, 'A');

    // AC - from the map, not verified against the display
    document.getElementById('acGrid').innerHTML = d.acAvailable
      ? card('Inverter output', d.acOutputPower, 'W', 'pos') +
        card('Frequency', d.frequency, 'Hz') +
        card('Grid voltage', d.gridVoltage, 'V')
      : '<div class="card"><div class="label">Info</div><div class="value" style="font-size:14px">Registers 35-38 unavailable</div></div>';

    // Grid and house
    if (d.block2Available) {
      document.getElementById('gridGrid').innerHTML =
        card('Export to grid', d.powerToGrid, 'W', d.powerToGrid > 0 ? 'pos' : '') +
        card('Import from grid', d.powerFromGrid, 'W', d.powerFromGrid > 0 ? 'neg' : '') +
        card('House load', d.loadConsumption, 'W', 'warning');
    } else {
      document.getElementById('gridGrid').innerHTML =
        '<div class="card"><div class="label">Info</div><div class="value" style="font-size:14px">Block 1000-1040 could not be read</div></div>';
    }

    // Battery
    if (d.block2Available) {
      const socClass = d.batterySOC > 80 ? 'pos' : (d.batterySOC > 30 ? '' : 'neg');
      document.getElementById('batteryGrid').innerHTML =
        card('SOC', d.batterySOC, '%', socClass) +
        card('Voltage', d.batteryVoltage, 'V') +
        card('Charging', d.batteryCharge, 'W', d.batteryCharge > 0 ? 'pos' : '') +
        card('Discharging', d.batteryDischarge, 'W', d.batteryDischarge > 0 ? 'neg' : '') +
        card('Temperature', d.batteryTemp, '°C', d.batteryTemp > 45 ? 'warning' : '') +
        card('Direction', d.batteryDirection, '');
    } else {
      document.getElementById('batteryGrid').innerHTML =
        '<div class="card"><div class="label">Info</div><div class="value" style="font-size:14px">N/A</div></div>';
    }

    // Balance - a check that the register map still holds
    if (d.block2Available) {
      const diff = Math.abs(d.balanceIn - d.balanceOut);
      const ref = Math.max(d.balanceIn, d.balanceOut);
      const bad = ref > 100 && diff > ref * 0.05;
      document.getElementById('statsGrid').innerHTML =
        card('Balance in', d.balanceIn, 'W') +
        card('Balance out', d.balanceOut, 'W') +
        card('Difference', diff, 'W', bad ? 'neg' : 'pos');
    } else {
      document.getElementById('statsGrid').innerHTML = '';
    }

    // Footer
    document.getElementById('updateTime').textContent =
      'Last update: ' + new Date().toLocaleTimeString();
    document.getElementById('uptime').textContent = formatUptime(d.uptime);
    document.getElementById('reads').textContent = d.totalReads;
    document.getElementById('errors').textContent = d.totalErrors;

  } catch(e) {
    console.error('Refresh error:', e);
  }
}

async function ssrControl(action) {
  try {
    const r = await fetch('/api/ssr/' + action, { method: 'POST' });
    if (r.ok) refresh();
    else alert('Error: ' + r.statusText);
  } catch(e) {
    alert('Error: ' + e.message);
  }
}

refresh();
setInterval(refresh, 5000);
</script>
</body>
</html>
)=====";

void setupWebServer() {
  // Main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", HTML_PAGE);
  });

  // JSON API
  server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *req) {
    StaticJsonDocument<2048> doc;

    doc["isOnline"] = growatt.isOnline;
    doc["block2Available"] = growatt.block2Available;
    doc["inverterStatus"] = growatt.inverterStatus;
    doc["inverterStatusText"] = growatt.inverterStatusText;
    doc["systemMode"] = growatt.systemMode;
    doc["lastError"] = growatt.lastError;

    doc["pvTotalPower"] = growatt.pvTotalPower;
    doc["pv1Voltage"] = growatt.pv1Voltage;
    doc["pv1Current"] = growatt.pv1Current;
    doc["pv1Power"] = growatt.pv1Power;
    doc["pv2Voltage"] = growatt.pv2Voltage;
    doc["pv2Current"] = growatt.pv2Current;
    doc["pv2Power"] = growatt.pv2Power;

    doc["acAvailable"] = growatt.acAvailable;
    doc["acOutputPower"] = growatt.acOutputPower;
    doc["frequency"] = growatt.frequency;
    doc["gridVoltage"] = growatt.gridVoltage;

    doc["powerToGrid"] = growatt.powerToGrid;
    doc["powerFromGrid"] = growatt.powerFromGrid;
    doc["loadConsumption"] = growatt.loadConsumption;

    doc["batterySOC"] = growatt.batterySOC;
    doc["batteryVoltage"] = growatt.batteryVoltage;
    doc["batteryCharge"] = growatt.batteryCharge;
    doc["batteryDischarge"] = growatt.batteryDischarge;
    doc["batteryPower"] = growatt.batteryPower;
    doc["batteryTemp"] = growatt.batteryTemp;
    doc["batteryDirection"] = growatt.batteryDirection;

    doc["balanceIn"] = growatt.balanceIn;
    doc["balanceOut"] = growatt.balanceOut;
    doc["ssrMinPvVoltage"] = ssrMinPvVoltage;
    doc["ssrMinSoc"] = ssrMinSoc;
    doc["ssrPvMin"] = SSR_PV_VOLTAGE_MIN;
    doc["ssrPvMax"] = SSR_PV_VOLTAGE_MAX;
    doc["ssrSocMin"] = SSR_SOC_MIN;
    doc["ssrSocMax"] = SSR_SOC_MAX;

    doc["ssrState"] = ssrState;
    const char* modeStr = (ssrMode == SSR_ON) ? "on" :
                          (ssrMode == SSR_AUTO_ON) ? "auto_on" :
                          (ssrMode == SSR_AUTO_OFF) ? "auto_off" : "off";
    doc["ssrMode"] = modeStr;
    doc["ssrNextCheck"] = (ssrMode == SSR_AUTO_ON || ssrMode == SSR_AUTO_OFF)
      ? max(0L, (long)(SSR_CHECK_INTERVAL_MS - (millis() - ssrCheckStart)) / 1000) : 0;
    doc["totalReads"] = growatt.totalReads;
    doc["totalErrors"] = growatt.totalErrors;
    doc["uptime"] = (millis() - bootTime) / 1000;
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["wifiRssi"] = WiFi.RSSI();

    String json;
    serializeJson(doc, json);
    req->send(200, "application/json", json);
  });

  // Manual SSR ON (stays on)
  server.on("/api/ssr/on", HTTP_POST, [](AsyncWebServerRequest *req) {
    Serial.println("[API] Manual ON");
    setSSR(true);
    ssrMode = SSR_ON;
    req->send(200, "application/json", "{\"success\":true,\"state\":\"on\"}");
  });

  // Manual SSR OFF (stays off)
  server.on("/api/ssr/off", HTTP_POST, [](AsyncWebServerRequest *req) {
    Serial.println("[API] Manual OFF");
    setSSR(false);
    ssrMode = SSR_OFF;
    req->send(200, "application/json", "{\"success\":true,\"state\":\"off\"}");
  });

  // AUTO mode - surplus.
  // It starts in the OFF state: whether there is surplus is decided
  // from the data, not by pressing the button. This used to switch the
  // SSR on straight away, which meant heating even with no surplus.
  server.on("/api/ssr/auto", HTTP_POST, [](AsyncWebServerRequest *req) {
    Serial.println("[API] AUTO mode - waiting for the surplus evaluation");
    setSSR(false);
    ssrMode = SSR_AUTO_OFF;
    // evaluate on the very next read, no need to wait 60 s
    ssrCheckStart = millis() - SSR_CHECK_INTERVAL_MS;
    req->send(200, "application/json", "{\"success\":true,\"state\":\"auto\"}");
  });

  // Runtime threshold change: POST /api/limits?pv=330&soc=90
  //
  // The values are NOT persisted to flash - after a reboot the
  // SSR_DEFAULT_* values apply again. That was the requested behaviour.
  server.on("/api/limits", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("pv") || !req->hasParam("soc")) {
      req->send(400, "application/json",
                "{\"success\":false,\"error\":\"missing parameter pv or soc\"}");
      return;
    }

    // toInt() yields 0 on non-numeric input, so anything invalid falls
    // through to the range check below and gets rejected.
    long pv = req->getParam("pv")->value().toInt();
    long soc = req->getParam("soc")->value().toInt();

    // This check is the binding one - browser side JS can be bypassed
    // and these two numbers decide whether 230 V gets switched. Never
    // remove it.
    if (pv < SSR_PV_VOLTAGE_MIN || pv > SSR_PV_VOLTAGE_MAX) {
      Serial.printf("[API] Rejected voltage threshold: %ld V\n", pv);
      req->send(400, "application/json",
                "{\"success\":false,\"error\":\"voltage out of range "
                SSR_STR(SSR_PV_VOLTAGE_MIN) "-" SSR_STR(SSR_PV_VOLTAGE_MAX) " V\"}");
      return;
    }
    if (soc < SSR_SOC_MIN || soc > SSR_SOC_MAX) {
      Serial.printf("[API] Rejected battery threshold: %ld %%\n", soc);
      req->send(400, "application/json",
                "{\"success\":false,\"error\":\"state of charge out of range "
                SSR_STR(SSR_SOC_MIN) "-" SSR_STR(SSR_SOC_MAX) " %\"}");
      return;
    }

    ssrMinPvVoltage = (float)pv;
    ssrMinSoc = (uint16_t)soc;
    Serial.printf("[API] New thresholds: strings %ld V, battery %ld %%\n", pv, soc);

    // Apply the new condition right away instead of up to 60 s later.
    // This matters when a threshold is tightened - the SSR should then
    // switch off without waiting.
    ssrCheckStart = millis() - SSR_CHECK_INTERVAL_MS;

    req->send(200, "application/json", "{\"success\":true}");
  });

  // 404
  server.onNotFound([](AsyncWebServerRequest *req) {
    req->send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("[WEB] Server listening on port 80");
}

// ============ SETUP ============

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n");
  Serial.println("==========================================");
  Serial.println(" Growatt SPH BH-UP Modbus Reader");
  Serial.println(" Hardware: LILYGO TTGO T-Call V1.3");
  Serial.println("==========================================");

  bootTime = millis();

  // GPIO setup
  pinMode(MAX485_DE_RE_PIN, OUTPUT);
  digitalWrite(MAX485_DE_RE_PIN, LOW);  // RX mode

  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);  // SSR off at boot (SAFE STATE)

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Boot blink (3 short)
  blinkLED(3, 80);

  // RS485 / Modbus
  RS485Serial.begin(MODBUS_BAUD, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  Serial.printf("[MODBUS] Slave ID=%d, %d baud, RX=GPIO%d, TX=GPIO%d\n",
                MODBUS_SLAVE_ID, MODBUS_BAUD, MODBUS_RX_PIN, MODBUS_TX_PIN);
  Serial.printf("[MODBUS] Blocks: %d-%d, %d-%d, %d-%d (fn 0x04)\n",
                BLOCK1_START, BLOCK1_START + BLOCK1_COUNT - 1,
                BLOCK2_START, BLOCK2_START + BLOCK2_COUNT - 1,
                BLOCK3_START, BLOCK3_START + BLOCK3_COUNT - 1);
  Serial.println("[MODBUS] The cable must be in port RS485-3!");

  // ============ WiFi AP mode ============
  Serial.println("[WIFI] Starting AP mode...");

  WiFi.mode(WIFI_AP);

  IPAddress local_IP(192, 168, 10, 1);
  IPAddress gateway(192, 168, 10, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(local_IP, gateway, subnet);

  // No second argument = open network, no password.
  if (WiFi.softAP(WIFI_SSID)) {
    Serial.println("[WIFI] AP started");
    Serial.printf("[WIFI] SSID: %s (open network, no password)\n", WIFI_SSID);
    Serial.printf("[WIFI] IP: %s\n", WiFi.softAPIP().toString().c_str());

    setupWebServer();

    // Long blink = OK
    blinkLED(2, 300);
  } else {
    Serial.println("[WIFI] ERROR - the AP failed to start!");
    blinkLED(10, 100);
  }

  lastSuccessfulRead = millis();

  Serial.println("==========================================");
  Serial.println(" System ready, starting to poll...");
  Serial.println("==========================================\n");
}

// ============ LOOP ============

void loop() {
  // Modbus poll on the read interval
  if (millis() - lastReadAttempt >= READ_INTERVAL_MS) {
    lastReadAttempt = millis();

    if (readGrowattData()) {
      printDataToSerial();
    }
  }

  // The watchdogs and the state machine run independently of the read
  // cycle - otherwise a complete Modbus outage would leave the SSR on
  // until the next read attempt, i.e. 30 s later.
  updateSSR();

  delay(10);
}
