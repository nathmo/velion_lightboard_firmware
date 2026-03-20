/*
 * Velion Lightboard Firmware - Mainboard404 Emergency Variant
 *
 * Key goals:
 * - Sonar removed
 * - Parse CAN 0x400 / 0x405 / 0x407
 * - Build and transmit control CAN 0x5FF every 50 ms
 * - Drive 8 MOSFET outputs through MCP23017 with configurable role mapping
 * - Host WiFi AP web page for diagnostics, manual output control, and mapping config
 */

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_VEML7700.h>
#include <stdarg.h>
#include "driver/twai.h"

// --------------------------
// Board pins (ESP32-C3)
// --------------------------
#define PIN_SDA            3
#define PIN_SCL            4
#define PIN_CAN_TX         GPIO_NUM_5
#define PIN_CAN_RX         GPIO_NUM_6
#define PIN_CAN_S          7

// --------------------------
// MCP23017 definitions
// --------------------------
#define MCP_ADDR           0x20

// Channel index 0..7 -> MCP pin map
// CH0=GPB7(15), CH1=GPB6(14), CH2=GPB5(13), CH3=GPB4(12),
// CH4=GPB3(11), CH5=GPB2(10), CH6=GPB1(9), CH7=GPB0(8)
static const uint8_t kMosfetChannelToMcpPin[8] = {15, 14, 13, 12, 11, 10, 9, 8};

// --------------------------
// CAN IDs and bit mapping
// --------------------------
#define CAN_ID_LIGHT_CONTROL 0x400
#define CAN_ID_BUTTONS       0x405
#define CAN_ID_THROTTLE      0x407
#define CAN_ID_CONTROL_OUT   0x5FF

// 0x400 bit map
#define BIT_HORN         0
#define BIT_RIGHT_BLINK  3
#define BIT_LEFT_BLINK   4

// 0x405 bit map (byte0)
#define BTN_A_MASK 0x08
#define BTN_B_MASK 0x04

// --------------------------
// Timing constants
// --------------------------
#define LOOP_DELAY_MS            1
#define CAN_CMD_TIMEOUT_MS       3000
#define CAN_TX_5FF_MS            50
#define SENSOR_READ_INTERVAL_MS  20
#define VEML_READ_INTERVAL_MS    200
#define DEBUG_PRINT_INTERVAL_MS  2000
#define BUTTON_LONG_PRESS_MS     3000
#define HEADLIGHT_PULSE_MS       120

// --------------------------
// Input shaping
// --------------------------
#define THROTTLE_DEADBAND        0x40

// --------------------------
// WiFi AP
// --------------------------
const char* AP_SSID = "LightBoard404";
const char* AP_PASS = "";

// --------------------------
// Persistent mapping keys
// --------------------------
const char* PREF_NS = "lb404";
const char* PREF_K_HL_PWR = "hl_pwr";
const char* PREF_K_HL_TGL = "hl_tgl";
const char* PREF_K_BL_L   = "bl_l";
const char* PREF_K_BL_R   = "bl_r";
const char* PREF_K_HORN   = "horn";

// --------------------------
// Peripherals
// --------------------------
Adafruit_MCP23X17 mcp;
Adafruit_ADS1115 ads0;
Adafruit_ADS1115 ads1;
Adafruit_VEML7700 veml;
Preferences prefs;
WebServer server(80);

bool mcp_ok = false;
bool ads0_ok = false;
bool ads1_ok = false;
bool veml_ok = false;

// --------------------------
// Runtime state
// --------------------------
uint16_t can400_bits = 0;
uint32_t lastCan400Ms = 0;
uint32_t lastCan405Ms = 0;
uint32_t lastCan407Ms = 0;

uint8_t throttleRaw = 0;

bool btnA = false;
bool btnB = false;
bool btnA_prev = false;
bool btnB_prev = false;
bool btnA_press_event = false;  // Transient flag cleared after CAN tx
bool btnB_press_event = false;  // Transient flag cleared after CAN tx

bool reverseMode = false;
bool regenMode = false;

bool headlightPowerEnabled = true;

uint32_t lastControlTxMs = 0;
uint32_t lastSensorReadMs = 0;
uint32_t lastVemlReadMs = 0;
uint32_t lastDebugMs = 0;

int16_t currentMa[8] = {0};
float ambientLux = 0.0f;

bool manualMode = false;
bool manualOutput[8] = {false, false, false, false, false, false, false, false};
bool autoOutput[8] = {false, false, false, false, false, false, false, false};
bool appliedOutput[8] = {false, false, false, false, false, false, false, false};

// Configurable MOSFET channel roles (persisted)
uint8_t mapHeadlightToggle = 0;
uint8_t mapBlinkRight = 2;
uint8_t mapBlinkLeft = 3;
uint8_t mapHorn = 4;
uint8_t mapHeadlightPower = 5;

// Web diagnostics buffers
#define SERIAL_LOG_LINES_MAX 120
#define CAN_MONITOR_MAX_IDS  64

String serialLogLines[SERIAL_LOG_LINES_MAX];
uint16_t serialLogHead = 0;
uint16_t serialLogCount = 0;

struct CanLatestFrame {
  bool valid;
  uint32_t id;
  bool rx;
  uint8_t dlc;
  uint8_t data[8];
  uint32_t lastMs;
  uint32_t seenCount;
};

CanLatestFrame canLatest[CAN_MONITOR_MAX_IDS];

// --------------------------
// ADS channel map
// --------------------------
struct AdsChanMap {
  Adafruit_ADS1115* ads;
  uint8_t chan;
  bool* ok;
};
AdsChanMap adsChanMap[8];

// --------------------------
// Helpers
// --------------------------
void appendSerialLogLine(const String& line) {
  serialLogLines[serialLogHead] = line;
  serialLogHead = (serialLogHead + 1) % SERIAL_LOG_LINES_MAX;
  if (serialLogCount < SERIAL_LOG_LINES_MAX) {
    serialLogCount++;
  }
}

void logLine(const String& line) {
  Serial.println(line);
  appendSerialLogLine(line);
}

void logf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.println(buf);
  appendSerialLogLine(String(buf));
}

int findOrAllocateCanSlot(uint32_t id) {
  int freeIdx = -1;
  for (int i = 0; i < CAN_MONITOR_MAX_IDS; i++) {
    if (canLatest[i].valid && canLatest[i].id == id) return i;
    if (!canLatest[i].valid && freeIdx < 0) freeIdx = i;
  }
  if (freeIdx >= 0) return freeIdx;

  // Replace the oldest entry when the table is full.
  uint32_t oldestMs = canLatest[0].lastMs;
  int oldestIdx = 0;
  for (int i = 1; i < CAN_MONITOR_MAX_IDS; i++) {
    if (canLatest[i].lastMs < oldestMs) {
      oldestMs = canLatest[i].lastMs;
      oldestIdx = i;
    }
  }
  return oldestIdx;
}

void trackCanFrame(uint32_t id, const uint8_t* data, uint8_t dlc, bool rx) {
  int idx = findOrAllocateCanSlot(id);
  canLatest[idx].valid = true;
  canLatest[idx].id = id;
  canLatest[idx].rx = rx;
  canLatest[idx].dlc = dlc > 8 ? 8 : dlc;
  for (uint8_t i = 0; i < canLatest[idx].dlc; i++) {
    canLatest[idx].data[i] = data[i];
  }
  canLatest[idx].lastMs = millis();
  canLatest[idx].seenCount++;
}

String hexByte(uint8_t v) {
  char b[3];
  snprintf(b, sizeof(b), "%02X", v);
  return String(b);
}

String canDataHex(const CanLatestFrame& e) {
  String out;
  for (uint8_t i = 0; i < e.dlc; i++) {
    if (i) out += " ";
    out += hexByte(e.data[i]);
  }
  return out;
}

String serialLogText() {
  String out;
  uint16_t start = (serialLogHead + SERIAL_LOG_LINES_MAX - serialLogCount) % SERIAL_LOG_LINES_MAX;
  for (uint16_t i = 0; i < serialLogCount; i++) {
    uint16_t idx = (start + i) % SERIAL_LOG_LINES_MAX;
    out += serialLogLines[idx];
    out += "\n";
  }
  return out;
}

String canMonitorJson() {
  int indices[CAN_MONITOR_MAX_IDS];
  int count = 0;
  for (int i = 0; i < CAN_MONITOR_MAX_IDS; i++) {
    if (canLatest[i].valid) {
      indices[count++] = i;
    }
  }

  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (canLatest[indices[j]].id < canLatest[indices[i]].id) {
        int t = indices[i];
        indices[i] = indices[j];
        indices[j] = t;
      }
    }
  }

  String out = "[";
  uint32_t now = millis();
  for (int i = 0; i < count; i++) {
    const CanLatestFrame& e = canLatest[indices[i]];
    if (i) out += ",";
    out += "{";
    out += "\"id\":" + String(e.id) + ",";
    out += "\"id_hex\":\"0x" + String(e.id, HEX) + "\",";
    out += "\"dir\":\"" + String(e.rx ? "RX" : "TX") + "\",";
    out += "\"dlc\":" + String(e.dlc) + ",";
    out += "\"data\":\"" + canDataHex(e) + "\",";
    out += "\"age_ms\":" + String(now - e.lastMs) + ",";
    out += "\"count\":" + String(e.seenCount);
    out += "}";
  }
  out += "]";
  return out;
}

int16_t scaleThrottleToInt16(uint8_t raw) {
  return (int16_t)(((uint32_t)raw * 32767UL) / 255UL);
}

bool validChannel(int v) {
  return (v >= 0 && v <= 7);
}

void setAllOutputsLow() {
  for (int ch = 0; ch < 8; ch++) {
    autoOutput[ch] = false;
  }
}

void persistMapping() {
  prefs.begin(PREF_NS, false);
  prefs.putUChar(PREF_K_HL_TGL, mapHeadlightToggle);
  prefs.putUChar(PREF_K_BL_R, mapBlinkRight);
  prefs.putUChar(PREF_K_BL_L, mapBlinkLeft);
  prefs.putUChar(PREF_K_HORN, mapHorn);
  prefs.putUChar(PREF_K_HL_PWR, mapHeadlightPower);
  prefs.end();
}

void loadMapping() {
  prefs.begin(PREF_NS, true);
  mapHeadlightToggle = prefs.getUChar(PREF_K_HL_TGL, 0);
  mapBlinkRight = prefs.getUChar(PREF_K_BL_R, 2);
  mapBlinkLeft = prefs.getUChar(PREF_K_BL_L, 3);
  mapHorn = prefs.getUChar(PREF_K_HORN, 4);
  mapHeadlightPower = prefs.getUChar(PREF_K_HL_PWR, 5);
  prefs.end();

  if (!validChannel(mapHeadlightToggle)) mapHeadlightToggle = 0;
  if (!validChannel(mapBlinkRight)) mapBlinkRight = 2;
  if (!validChannel(mapBlinkLeft)) mapBlinkLeft = 3;
  if (!validChannel(mapHorn)) mapHorn = 4;
  if (!validChannel(mapHeadlightPower)) mapHeadlightPower = 5;
}

void applyOutputsToMcp() {
  if (!mcp_ok) return;

  for (int ch = 0; ch < 8; ch++) {
    bool desired = manualMode ? manualOutput[ch] : autoOutput[ch];
    if (desired != appliedOutput[ch]) {
      mcp.digitalWrite(kMosfetChannelToMcpPin[ch], desired ? HIGH : LOW);
      appliedOutput[ch] = desired;
    }
  }
}

String i2cScanJson() {
  String out = "[";
  bool first = true;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      if (!first) out += ",";
      out += String(addr);
      first = false;
    }
  }
  out += "]";
  return out;
}

void setupGPIO() {
  pinMode(PIN_CAN_S, OUTPUT);
  digitalWrite(PIN_CAN_S, LOW);
}

void setupCAN() {
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    logLine("[CAN] Driver installed");
  } else {
    logLine("[CAN] Driver install failed");
  }

  if (twai_start() == ESP_OK) {
    logLine("[CAN] Started @ 1 Mbps");
  } else {
    logLine("[CAN] Start failed");
  }
}

void setupMCP() {
  if (!mcp.begin_I2C(MCP_ADDR)) {
    logLine("[MCP] 0x20 not found");
    mcp_ok = false;
    return;
  }

  mcp_ok = true;
  logLine("[MCP] 0x20 OK");

  for (int p = 8; p <= 15; p++) {
    mcp.pinMode(p, OUTPUT);
    mcp.digitalWrite(p, LOW);
  }

  for (int p = 0; p <= 7; p++) {
    mcp.pinMode(p, OUTPUT);
    mcp.digitalWrite(p, LOW);
  }
}

void setupADS() {
  if (ads0.begin(0x48)) {
    ads0_ok = true;
    ads0.setGain(GAIN_SIXTEEN);
    ads0.setDataRate(RATE_ADS1115_860SPS);
    logLine("[ADS] 0x48 OK");
  } else {
    logLine("[ADS] 0x48 not found");
  }

  if (ads1.begin(0x49)) {
    ads1_ok = true;
    ads1.setGain(GAIN_SIXTEEN);
    ads1.setDataRate(RATE_ADS1115_860SPS);
    logLine("[ADS] 0x49 OK");
  } else {
    logLine("[ADS] 0x49 not found");
  }

  adsChanMap[0] = {&ads0, 3, &ads0_ok};
  adsChanMap[1] = {&ads0, 2, &ads0_ok};
  adsChanMap[2] = {&ads0, 1, &ads0_ok};
  adsChanMap[3] = {&ads0, 0, &ads0_ok};

  adsChanMap[4] = {&ads1, 3, &ads1_ok};
  adsChanMap[5] = {&ads1, 2, &ads1_ok};
  adsChanMap[6] = {&ads1, 1, &ads1_ok};
  adsChanMap[7] = {&ads1, 0, &ads1_ok};
}

void setupVEML() {
  if (veml.begin()) {
    veml_ok = true;
    veml.setGain(VEML7700_GAIN_1);
    veml.setIntegrationTime(VEML7700_IT_100MS);
    logLine("[VEML] 0x10 OK");
  } else {
    logLine("[VEML] 0x10 not found");
  }
}

void readSensors() {
  uint32_t now = millis();
  if (now - lastSensorReadMs < SENSOR_READ_INTERVAL_MS) return;
  lastSensorReadMs = now;

  for (uint8_t i = 0; i < 8; i++) {
    AdsChanMap& m = adsChanMap[i];
    if (m.ads != NULL && *(m.ok)) {
      int16_t raw = m.ads->readADC_SingleEnded(m.chan);
      float voltageMv = raw * 0.0078125f;
      currentMa[i] = (int16_t)voltageMv;
    }
  }

  if (veml_ok && (now - lastVemlReadMs >= VEML_READ_INTERVAL_MS)) {
    lastVemlReadMs = now;
    float lux = veml.readLux();
    if (isnan(lux) || isinf(lux) || lux < 0.0f) {
      ambientLux = 0.0f;
    } else {
      ambientLux = lux;
    }
  }
}

void handleButtonLogic() {
  // Button A: rising edge toggles reverse mode
  if (btnA && !btnA_prev) {
    btnA_press_event = true;  // Flag for CAN broadcast (one packet)
    reverseMode = !reverseMode;
    // If entering reverse, force accelerator mode (regen blocked)
    if (reverseMode) {
      regenMode = false;
    }
    logf("[BTN] A press -> Reverse mode=%d", reverseMode ? 1 : 0);
  }

  // Button B: rising edge toggles regen/accelerator mode
  if (btnB && !btnB_prev) {
    btnB_press_event = true;  // Flag for CAN broadcast (one packet)
    // Only allow regen toggle if not in reverse mode
    if (!reverseMode) {
      regenMode = !regenMode;
      logf("[BTN] B press -> Regen mode=%d", regenMode ? 1 : 0);
    } else {
      logLine("[BTN] B press ignored (reverse mode blocks regen)");
    }
  }

  btnA_prev = btnA;
  btnB_prev = btnB;
}

void synthesizeAutoMosfetOutputs() {
  setAllOutputsLow();

  bool canActive = (millis() - lastCan400Ms) <= CAN_CMD_TIMEOUT_MS;
  if (!canActive) {
    can400_bits = 0;
  }

  bool hornOn = canActive && ((can400_bits >> BIT_HORN) & 0x1);
  bool leftOn = canActive && ((can400_bits >> BIT_LEFT_BLINK) & 0x1);
  bool rightOn = canActive && ((can400_bits >> BIT_RIGHT_BLINK) & 0x1);

  // Headlight power: always on
  autoOutput[mapHeadlightPower] = true;
  
  // Headlight toggle: no longer used (power always on)
  autoOutput[mapHeadlightToggle] = false;

  // Horn and blinkers from CAN
  autoOutput[mapHorn] = hornOn;
  autoOutput[mapBlinkLeft] = leftOn;
  autoOutput[mapBlinkRight] = rightOn;
}

void parseCanButtons(const twai_message_t& msg) {
  if (msg.data_length_code < 1) return;
  uint8_t b0 = msg.data[0];
  btnA = (b0 & BTN_A_MASK) != 0;
  btnB = (b0 & BTN_B_MASK) != 0;
  lastCan405Ms = millis();
}

void parseCanThrottle(const twai_message_t& msg) {
  if (msg.data_length_code < 1) return;
  throttleRaw = msg.data[0];
  lastCan407Ms = millis();
}

void parseCanLightControl(const twai_message_t& msg) {
  if (msg.data_length_code < 2) return;
  can400_bits = (uint16_t)(msg.data[0] | (msg.data[1] << 8));
  lastCan400Ms = millis();
}

void readCAN() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    trackCanFrame(msg.identifier, msg.data, msg.data_length_code, true);
    switch (msg.identifier) {
      case CAN_ID_LIGHT_CONTROL:
        parseCanLightControl(msg);
        break;
      case CAN_ID_BUTTONS:
        parseCanButtons(msg);
        break;
      case CAN_ID_THROTTLE:
        parseCanThrottle(msg);
        break;
      default:
        break;
    }
  }
}

bool sendCanMessage(uint32_t id, const uint8_t* data, uint8_t dlc) {
  twai_message_t msg;
  msg.identifier = id;
  msg.extd = 0;
  msg.rtr = 0;
  msg.ss = 1;
  msg.self = 0;
  msg.dlc_non_comp = 0;
  msg.data_length_code = dlc;

  for (uint8_t i = 0; i < dlc; i++) {
    msg.data[i] = data[i];
  }

  bool ok = twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK;
  if (ok) {
    trackCanFrame(id, data, dlc, false);
  }
  return ok;
}

void sendControl5FF() {
  uint32_t now = millis();
  if (now - lastControlTxMs < CAN_TX_5FF_MS) return;
  lastControlTxMs = now;

  int16_t level1 = 0;   // Accelerator input
  int16_t level2 = 0;   // Regen braking input
  int16_t level3 = 0;    // Reserved/unused field

  int16_t scaled = scaleThrottleToInt16(throttleRaw);
  if (throttleRaw <= THROTTLE_DEADBAND) {
    scaled = 0;
  }

  // Choose which level gets the throttle based on mode.
  // If in reverse, force accelerator mode (regen blocked).
  bool effectiveRegenMode = regenMode && !reverseMode;
  if (effectiveRegenMode) {
    level1 = 0;
    level2 = scaled;
  } else {
    level1 = scaled;
    level2 = 0;
  }

  uint8_t digital = 0;
  // Bit 0: reverse mode state
  if (reverseMode) {
    digital |= (1 << 0);
  }
  // Bit 1: regen mode state
  if (regenMode) {
    digital |= (1 << 1);
  }
  // Bit 3: button B press event (regen button)
  if (btnB_press_event) {
    digital |= (1 << 3);
  }
  // Bit 4: button A press event (reverse button)
  if (btnA_press_event) {
    digital |= (1 << 4);
  }

  uint8_t payload[8];
  payload[0] = (uint8_t)(level1 & 0xFF);
  payload[1] = (uint8_t)((level1 >> 8) & 0xFF);
  payload[2] = (uint8_t)(level2 & 0xFF);
  payload[3] = (uint8_t)((level2 >> 8) & 0xFF);
  payload[4] = (uint8_t)(level3 & 0xFF);
  payload[5] = (uint8_t)((level3 >> 8) & 0xFF);
  payload[6] = digital;
  payload[7] = 0x00;  // Padding byte

  sendCanMessage(CAN_ID_CONTROL_OUT, payload, 8);

  // Clear press event flags after transmission
  btnA_press_event = false;
  btnB_press_event = false;
}

String jsonStatus() {
  String s = "{";

  s += "\"uptime_ms\":" + String(millis()) + ",";
  s += "\"mcp_ok\":" + String(mcp_ok ? "true" : "false") + ",";
  s += "\"ads0_ok\":" + String(ads0_ok ? "true" : "false") + ",";
  s += "\"ads1_ok\":" + String(ads1_ok ? "true" : "false") + ",";
  s += "\"veml_ok\":" + String(veml_ok ? "true" : "false") + ",";

  s += "\"throttle_raw\":" + String(throttleRaw) + ",";
  s += "\"regen_mode\":" + String(regenMode ? "true" : "false") + ",";
  s += "\"reverse_mode\":" + String(reverseMode ? "true" : "false") + ",";
  s += "\"manual_mode\":" + String(manualMode ? "true" : "false") + ",";

  s += "\"can_age_400_ms\":" + String(millis() - lastCan400Ms) + ",";
  s += "\"can_age_405_ms\":" + String(millis() - lastCan405Ms) + ",";
  s += "\"can_age_407_ms\":" + String(millis() - lastCan407Ms) + ",";

  s += "\"ambient_lux\":" + String(ambientLux, 2) + ",";

  s += "\"current_ma\":[";
  for (int i = 0; i < 8; i++) {
    if (i) s += ",";
    s += String(currentMa[i]);
  }
  s += "],";

  s += "\"output_applied\":[";
  for (int i = 0; i < 8; i++) {
    if (i) s += ",";
    s += (appliedOutput[i] ? "1" : "0");
  }
  s += "],";

  s += "\"mapping\":{";
  s += "\"headlight_power\":" + String(mapHeadlightPower) + ",";
  s += "\"headlight_toggle\":" + String(mapHeadlightToggle) + ",";
  s += "\"blink_left\":" + String(mapBlinkLeft) + ",";
  s += "\"blink_right\":" + String(mapBlinkRight) + ",";
  s += "\"horn\":" + String(mapHorn);
  s += "},";

  s += "\"i2c_scan\":" + i2cScanJson() + ",";
  s += "\"can_ids\":" + canMonitorJson();

  s += "}";
  return s;
}

void handleRoot() {
  String html;
  html.reserve(10000);
  html += "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>LightBoard 404</title>";
  html += "<style>body{font-family:Verdana,Arial,sans-serif;margin:16px;background:#f4f7fb;color:#14213d}";
  html += ".card{background:#fff;border-radius:12px;padding:14px;margin-bottom:14px;box-shadow:0 3px 14px rgba(0,0,0,.08)}";
  html += "button,input,select{padding:8px;margin:4px;border-radius:8px;border:1px solid #9aa7bf}";
  html += "pre{white-space:pre-wrap;word-break:break-word;max-height:320px;overflow:auto;background:#f7f9fc;border:1px solid #d3dbe8;padding:10px;border-radius:8px}";
  html += "table{border-collapse:collapse;width:100%;font-size:14px}th,td{border:1px solid #d3dbe8;padding:6px;text-align:left}";
  html += "h1{margin:0 0 10px 0}</style></head><body>";
  html += "<h1>Mainboard404 Emergency</h1>";

  html += "<div class='card'><b>Status</b><pre id='st'>loading...</pre><button onclick='refresh()'>Refresh</button></div>";

  html += "<div class='card'><b>CAN Monitor (latest frame per ID)</b><div id='canTbl'>loading...</div></div>";
  html += "<div class='card'><b>Serial Log</b><pre id='ser'>loading...</pre></div>";

  html += "<div class='card'><b>Manual MOSFET</b><br>";
  html += "<label>Manual mode <input type='checkbox' id='manual' onchange='setManual()'></label><br>";
  for (int i = 0; i < 8; i++) {
    html += "CH" + String(i) + " <button onclick='setCh(" + String(i) + ",1)'>ON</button><button onclick='setCh(" + String(i) + ",0)'>OFF</button><br>";
  }
  html += "</div>";

  html += "<div class='card'><b>Mapping</b><br>";
  html += "Headlight Toggle <input id='hlt' type='number' min='0' max='7'><br>";
  html += "Right Blinker <input id='blr' type='number' min='0' max='7'><br>";
  html += "Left Blinker <input id='bll' type='number' min='0' max='7'><br>";
  html += "Horn <input id='hrn' type='number' min='0' max='7'><br>";
  html += "Headlight Power <input id='hlp' type='number' min='0' max='7'><br>";
  html += "<button onclick='saveMap()'>Save Mapping</button></div>";

  html += "<script>";
  html += "function renderCanTable(rows){if(!rows||!rows.length){document.getElementById('canTbl').innerHTML='No frames yet';return;}";
  html += "let h='<table><tr><th>ID</th><th>Dir</th><th>DLC</th><th>Data</th><th>Age(ms)</th><th>Count</th></tr>';";
  html += "for(const r of rows){h+='<tr><td>'+r.id_hex.toUpperCase()+'</td><td>'+r.dir+'</td><td>'+r.dlc+'</td><td>'+r.data+'</td><td>'+r.age_ms+'</td><td>'+r.count+'</td></tr>'; }";
  html += "h+='</table>';document.getElementById('canTbl').innerHTML=h;}";

  html += "async function refresh(){";
  html += "let rs=await fetch('/api/status');let js=await rs.json();";
  html += "document.getElementById('st').textContent=JSON.stringify(js,null,2);";
  html += "document.getElementById('manual').checked=!!js.manual_mode;";
  html += "document.getElementById('hlt').value=js.mapping.headlight_toggle;";
  html += "document.getElementById('blr').value=js.mapping.blink_right;";
  html += "document.getElementById('bll').value=js.mapping.blink_left;";
  html += "document.getElementById('hrn').value=js.mapping.horn;";
  html += "document.getElementById('hlp').value=js.mapping.headlight_power;";

  html += "let rc=await fetch('/api/can');let jc=await rc.json();renderCanTable(jc);";
  html += "let rl=await fetch('/api/serial');let tl=await rl.text();document.getElementById('ser').textContent=tl;";
  html += "}";
  html += "async function setManual(){let e=document.getElementById('manual').checked?1:0;await fetch('/api/manual?enabled='+e);} ";
  html += "async function setCh(ch,v){await fetch('/api/setMosfet?ch='+ch+'&v='+v);} ";
  html += "async function saveMap(){";
  html += "let q='?headlightToggle='+hlt.value+'&rightBlinker='+blr.value+'&leftBlinker='+bll.value+'&horn='+hrn.value+'&headlightPower='+hlp.value;";
  html += "await fetch('/api/setMapping'+q);refresh();}";
  html += "setInterval(refresh,1000);refresh();";
  html += "</script></body></html>";

  server.send(200, "text/html", html);
}

void handleApiStatus() {
  server.send(200, "application/json", jsonStatus());
}

void handleApiCan() {
  server.send(200, "application/json", canMonitorJson());
}

void handleApiSerial() {
  server.send(200, "text/plain", serialLogText());
}

void handleApiManual() {
  if (!server.hasArg("enabled")) {
    server.send(400, "text/plain", "missing arg: enabled");
    return;
  }

  manualMode = server.arg("enabled").toInt() != 0;
  server.send(200, "text/plain", manualMode ? "manual=1" : "manual=0");
}

void handleApiSetMosfet() {
  if (!server.hasArg("ch") || !server.hasArg("v")) {
    server.send(400, "text/plain", "missing args: ch,v");
    return;
  }

  int ch = server.arg("ch").toInt();
  int v = server.arg("v").toInt();

  if (!validChannel(ch) || (v != 0 && v != 1)) {
    server.send(400, "text/plain", "invalid args");
    return;
  }

  manualOutput[ch] = (v == 1);
  server.send(200, "text/plain", "ok");
}

void handleApiSetMapping() {
  if (!server.hasArg("headlightPower") || !server.hasArg("headlightToggle") ||
      !server.hasArg("leftBlinker") || !server.hasArg("rightBlinker") || !server.hasArg("horn")) {
    server.send(400, "text/plain", "missing mapping args");
    return;
  }

  int hlp = server.arg("headlightPower").toInt();
  int hlt = server.arg("headlightToggle").toInt();
  int bll = server.arg("leftBlinker").toInt();
  int blr = server.arg("rightBlinker").toInt();
  int hrn = server.arg("horn").toInt();

  if (!validChannel(hlp) || !validChannel(hlt) || !validChannel(bll) ||
      !validChannel(blr) || !validChannel(hrn)) {
    server.send(400, "text/plain", "mapping range must be 0..7");
    return;
  }

  mapHeadlightPower = (uint8_t)hlp;
  mapHeadlightToggle = (uint8_t)hlt;
  mapBlinkLeft = (uint8_t)bll;
  mapBlinkRight = (uint8_t)blr;
  mapHorn = (uint8_t)hrn;
  persistMapping();

  server.send(200, "text/plain", "mapping saved");
}

void setupWeb() {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  if (ok) {
    logf("[WIFI] AP started: %s, IP=%s", AP_SSID, WiFi.softAPIP().toString().c_str());
  } else {
    logLine("[WIFI] AP start failed");
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/can", HTTP_GET, handleApiCan);
  server.on("/api/serial", HTTP_GET, handleApiSerial);
  server.on("/api/manual", HTTP_GET, handleApiManual);
  server.on("/api/setMosfet", HTTP_GET, handleApiSetMosfet);
  server.on("/api/setMapping", HTTP_GET, handleApiSetMapping);

  server.begin();
  logLine("[WEB] HTTP server started");
}

void printDebug() {
  uint32_t now = millis();
  if (now - lastDebugMs < DEBUG_PRINT_INTERVAL_MS) return;
  lastDebugMs = now;

  logf("[DBG] thr=%u regen=%d rev=%d bA=%d bB=%d can400_age=%lu manual=%d",
       throttleRaw,
       regenMode ? 1 : 0,
       reverseMode ? 1 : 0,
       btnA ? 1 : 0,
       btnB ? 1 : 0,
       (unsigned long)(now - lastCan400Ms),
       manualMode ? 1 : 0);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  logLine("\n========================================");
  logLine(" Mainboard404 Emergency Firmware");
  logLine("========================================");

  Wire.begin(PIN_SDA, PIN_SCL);

  loadMapping();
  setupGPIO();
  setupCAN();
  setupMCP();
  setupADS();
  setupVEML();
  setupWeb();

  lastCan400Ms = millis();
  lastCan405Ms = millis();
  lastCan407Ms = millis();

  logLine("[BOOT] Ready");
}

void loop() {
  readCAN();
  handleButtonLogic();
  readSensors();

  synthesizeAutoMosfetOutputs();
  applyOutputsToMcp();

  sendControl5FF();

  server.handleClient();
  printDebug();

  delay(LOOP_DELAY_MS);
}
