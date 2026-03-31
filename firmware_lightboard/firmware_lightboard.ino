/*
 * Velion Lightboard Firmware — Universal (Front / Rear)
 *
 * Single firmware for both boards. All MOSFET gate mappings, CAN signal
 * sources, button config, WiFi SSID, and 0x5FF broadcast are configurable
 * via the on-board WiFi AP web interface and persisted in NVS.
 *
 * Hardware: ESP32-C3 + MCP23017 + ADS1115 x2 + VEML7700 + CAN (TWAI)
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_VEML7700.h>
#include "driver/twai.h"
#include <stdarg.h>

// ── ESP32-C3 pin assignments ────────────────────────────────────────
static const gpio_num_t PIN_CAN_TX = GPIO_NUM_5;
static const gpio_num_t PIN_CAN_RX = GPIO_NUM_6;
static const uint8_t PIN_CAN_S  = 7;
static const uint8_t PIN_SDA    = 3;
static const uint8_t PIN_SCL    = 4;

// MCP23017 gate index 0..7 → MCP pin
static const uint8_t kGateToMcpPin[8] = {15, 14, 13, 12, 11, 10, 9, 8};

// ── CAN IDs ─────────────────────────────────────────────────────────
static const uint16_t CAN_ID_LIGHT_CONTROL    = 0x400;
static const uint16_t CAN_ID_FILOVELOX_RAW    = 0x405;
static const uint16_t CAN_ID_THROTTLE         = 0x407;
static const uint16_t CAN_ID_LYNX_STATUS      = 0x600;
static const uint16_t CAN_ID_SILIXCON_CONTROL = 0x5FF;

// ── Timing constants ────────────────────────────────────────────────
static const uint32_t CAN_5FF_INTERVAL_MS            = 50;
static const uint32_t THROTTLE_STALE_MS               = 500;
static const uint32_t CONTROL_STALE_MS                 = 1000;
static const uint32_t CAN_LOSS_TIMEOUT_MS              = 10000;
static const uint32_t CAN_LOSS_BLINK_HALF_PERIOD_MS    = 500;
static const uint32_t BUTTON_LONG_PRESS_MS             = 1000;
static const uint16_t THROTTLE_MAX_LEVEL               = 32500;

// ── Output function enum ────────────────────────────────────────────
enum OutputFunction : uint8_t {
  FUNC_OFF = 0,  FUNC_ON,  FUNC_BRAKE,  FUNC_DRL,
  FUNC_LEFT,  FUNC_RIGHT,  FUNC_HORN,  FUNC_FOG,
  FUNC_DRIVING,  FUNC_PASSING,  FUNC_TAIL,  FUNC_REVERSE,
  FUNC_HAZARD,  FUNC_TAIL_BRAKE,
  FUNC_MAX = FUNC_TAIL_BRAKE
};

static const char* kOutputFunctionNames[] = {
  "OFF","ON","BRAKE","DRL","LEFT_BLINK","RIGHT_BLINK",
  "HORN","FOG","DRIVING","PASSING","TAIL","REVERSE","HAZARD","TAIL_BRAKE"
};

// ── Command signal enum ─────────────────────────────────────────────
enum CommandSignal : uint8_t {
  SIG_PASSING = 0, SIG_DRIVING, SIG_LEFT, SIG_RIGHT,
  SIG_HORN, SIG_BRAKE, SIG_TAIL, SIG_DRL, SIG_FOG,
  SIG_MAX = SIG_FOG
};

static const char* kCommandSignalNames[] = {
  "PASSING","DRIVING","LEFT_BLINK","RIGHT_BLINK","HORN","BRAKE","TAIL","DRL","FOG"
};

// ── CAN-to-signal mapping ───────────────────────────────────────────
struct CanSignalMap {
  uint16_t canId;
  uint8_t  bit;
};

// Defaults (user-editable via web, persisted in NVS)
static const CanSignalMap kDefaultSignalMap[SIG_MAX + 1] = {
  {CAN_ID_LIGHT_CONTROL, 1},   // SIG_PASSING  — 0x400 bit 1
  {CAN_ID_FILOVELOX_RAW, 4},   // SIG_DRIVING  — 0x405 bit 4 (sw1)
  {CAN_ID_LIGHT_CONTROL, 3},   // SIG_LEFT     — 0x400 bit 3
  {CAN_ID_LIGHT_CONTROL, 4},   // SIG_RIGHT    — 0x400 bit 4
  {CAN_ID_LIGHT_CONTROL, 7},   // SIG_HORN     — 0x400 bit 7
  {CAN_ID_FILOVELOX_RAW, 7},   // SIG_BRAKE    — 0x405 bit 7
  {CAN_ID_LIGHT_CONTROL, 5},   // SIG_TAIL     — 0x400 bit 5
  {CAN_ID_LIGHT_CONTROL, 0},   // SIG_DRL      — 0x400 bit 0
  {CAN_ID_LIGHT_CONTROL, 6}    // SIG_FOG      — 0x400 bit 6
};

// Default gate→function maps for front and rear boards
static const uint8_t kDefaultFrontMap[8] = {
  FUNC_DRL, FUNC_PASSING, FUNC_LEFT, FUNC_RIGHT,
  FUNC_HORN, FUNC_ON, FUNC_DRIVING, FUNC_OFF
};
static const uint8_t kDefaultRearMap[8] = {
  FUNC_RIGHT, FUNC_OFF, FUNC_TAIL, FUNC_LEFT,
  FUNC_OFF, FUNC_BRAKE, FUNC_OFF, FUNC_OFF
};

// ── Peripheral objects ──────────────────────────────────────────────
WebServer        webServer(80);
Preferences      preferences;
Adafruit_MCP23X17 mcp;
Adafruit_ADS1115  ads48;
Adafruit_ADS1115  ads49;
Adafruit_VEML7700 veml;

bool mcpOk   = false;
bool ads48Ok = false;
bool ads49Ok = false;
bool vemlOk  = false;
bool canOk   = false;

// ── Persistent configuration (loaded from NVS at boot) ──────────────
char     apSsid[33] = "lightboard_unconfigured";
char     apPass[33] = "VelionDebug";
uint8_t  gateFunctionMap[8];
CanSignalMap signalMap[SIG_MAX + 1];
uint16_t btnCanId     = CAN_ID_FILOVELOX_RAW;  // long/short press button source
uint8_t  btnBit       = 3;                      // sw4 = bit 3 of 0x405
bool     can5ffEnabled = false;

// ── Runtime state ───────────────────────────────────────────────────
bool webAutoMode = true;
bool manualGateState[8]      = {};
bool lastAppliedGateState[8] = {};

float   senseCurrentA[8] = {};
int16_t senseRaw[8]      = {};
float   luxValue = 0.0f;

uint16_t lightWord      = 0;
bool     lightWordValid = false;
uint32_t lastLightWordMs = 0;

uint8_t  throttleByte   = 0;
bool     throttleValid  = false;
uint32_t lastThrottleMs = 0;

uint8_t lynxMode       = 0;
uint8_t currentPowerMap = 0;

// Decoded command signals
bool cmdPassing   = false;
bool cmdDriving   = false;
bool cmdLeftBlink = false;
bool cmdRightBlink= false;
bool cmdHorn      = false;
bool cmdBrake     = false;
bool cmdTail      = false;
bool cmdDrl       = false;
bool cmdFog       = false;
uint32_t cmdLastUpdateMs[SIG_MAX + 1] = {};
bool     signalRawState[SIG_MAX + 1]  = {};

// Button state (long / short press from configurable CAN source)
bool     buttonPrev         = false;
uint32_t buttonPressStartMs = 0;
bool     reverseLatched     = false;
bool     mappingPulsePending= false;

// 0x5FF TX state
uint32_t last5ffTxMs   = 0;
bool     last5ffValid  = false;
uint8_t  last5ffData[8]= {};
uint32_t last5ffSentMs = 0;

// Timing
uint32_t lastSensorReadMs = 0;
uint32_t lastAnyCanRxMs   = 0;

// CAN frame tracker (for web dashboard)
static const uint8_t CAN_TRACK_MAX = 64;
struct CanTrack { bool used; uint32_t id; uint8_t dlc; uint8_t data[8]; uint32_t lastMs; };
CanTrack canTrack[CAN_TRACK_MAX] = {};

// Serial log ring buffer
static const uint8_t SERIAL_LOG_MAX = 40;
String  serialLog[SERIAL_LOG_MAX];
uint8_t serialLogHead  = 0;
uint8_t serialLogCount = 0;

// ═════════════════════════════════════════════════════════════════════
//  HELPERS
// ═════════════════════════════════════════════════════════════════════

String hexByte(uint8_t b) {
  const char* d = "0123456789ABCDEF";
  String s; s.reserve(2);
  s += d[(b >> 4) & 0x0F];
  s += d[b & 0x0F];
  return s;
}

void serialLogPush(const String& line) {
  serialLog[serialLogHead] = line;
  serialLogHead = (uint8_t)((serialLogHead + 1) % SERIAL_LOG_MAX);
  if (serialLogCount < SERIAL_LOG_MAX) serialLogCount++;
}

void logLine(const String& line) {
  Serial.println(line);
  serialLogPush(line);
}

void logf(const char* fmt, ...) {
  char buf[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  logLine(String(buf));
}

const char* signalName(uint8_t s) {
  return (s <= SIG_MAX) ? kCommandSignalNames[s] : "?";
}
const char* functionName(uint8_t f) {
  return (f <= FUNC_MAX) ? kOutputFunctionNames[f] : "?";
}

const char* lynxModeName(uint8_t mode) {
  switch (mode) {
    case   0: return "STOP";          case   2: return "INVALID_INPUTS";
    case   6: return "BMS_ERROR";     case   9: return "IDENT";
    case  10: return "INIT";          case  20: return "CAN_TIMEOUT";
    case  30: return "WELDING";       case  65: return "CHARGE_EXT_FOREVER";
    case  68: return "CHARGE_EXT";    case  70: return "CHARGE";
    case  79: return "SLAVE";         case  80: return "OVR";
    case  90: return "LOCK";          case  95: return "SEAT_SWITCH";
    case 100: return "STANDBY";       case 101: return "BRAKE";
    case 102: return "ACCELERATE";    case 104: return "KICKBACK";
    case 105: return "ASSIST";        case 110: return "CRUISE";
    case 120: return "DISARMED_FOREVER";
    default:  return "UNKNOWN";
  }
}

bool parseUIntArg(const String& key, long minV, long maxV, uint8_t& out) {
  if (!webServer.hasArg(key)) return false;
  String s = webServer.arg(key);
  char* endPtr = NULL;
  long v = strtol(s.c_str(), &endPtr, 10);
  if (endPtr == s.c_str() || *endPtr != '\0') return false;
  if (v < minV || v > maxV) return false;
  out = (uint8_t)v;
  return true;
}

// ═════════════════════════════════════════════════════════════════════
//  PERSISTENCE (NVS)
// ═════════════════════════════════════════════════════════════════════

void savePersistentConfig() {
  preferences.begin("velion", false);
  preferences.putString("ssid", String(apSsid));
  preferences.putString("pass", String(apPass));
  preferences.putBool("tx5en", can5ffEnabled);
  preferences.putUShort("btnCid", btnCanId);
  preferences.putUChar("btnBit", btnBit);
  for (uint8_t i = 0; i < 8; i++) {
    char key[4]; snprintf(key, sizeof(key), "m%u", (unsigned)i);
    preferences.putUChar(key, gateFunctionMap[i]);
  }
  for (uint8_t s = 0; s <= SIG_MAX; s++) {
    char kc[5], kb[5];
    snprintf(kc, sizeof(kc), "s%uc", (unsigned)s);
    snprintf(kb, sizeof(kb), "s%ub", (unsigned)s);
    preferences.putUShort(kc, signalMap[s].canId);
    preferences.putUChar(kb, signalMap[s].bit);
  }
  preferences.end();
}

void loadPersistentConfig() {
  // Start with compiled-in defaults
  for (uint8_t i = 0; i < 8; i++) gateFunctionMap[i] = kDefaultFrontMap[i];
  for (uint8_t s = 0; s <= SIG_MAX; s++) signalMap[s] = kDefaultSignalMap[s];

  preferences.begin("velion", true);
  String ssid = preferences.getString("ssid", "lightboard_unconfigured");
  ssid.toCharArray(apSsid, sizeof(apSsid));
  String pass = preferences.getString("pass", "VelionDebug");
  pass.toCharArray(apPass, sizeof(apPass));
  can5ffEnabled = preferences.getBool("tx5en", false);
  btnCanId = preferences.getUShort("btnCid", CAN_ID_FILOVELOX_RAW);
  btnBit   = preferences.getUChar("btnBit", 3);
  for (uint8_t i = 0; i < 8; i++) {
    char key[4]; snprintf(key, sizeof(key), "m%u", (unsigned)i);
    uint8_t v = preferences.getUChar(key, gateFunctionMap[i]);
    gateFunctionMap[i] = (v <= FUNC_MAX) ? v : kDefaultFrontMap[i];
  }
  for (uint8_t s = 0; s <= SIG_MAX; s++) {
    char kc[5], kb[5];
    snprintf(kc, sizeof(kc), "s%uc", (unsigned)s);
    snprintf(kb, sizeof(kb), "s%ub", (unsigned)s);
    signalMap[s].canId = preferences.getUShort(kc, kDefaultSignalMap[s].canId);
    signalMap[s].bit   = preferences.getUChar(kb, kDefaultSignalMap[s].bit);
  }
  preferences.end();
}

// ═════════════════════════════════════════════════════════════════════
//  MCP23017 — MOSFET GATE DRIVER
// ═════════════════════════════════════════════════════════════════════

void mcpWriteGate(uint8_t gate, bool on) {
  if (!mcpOk || gate > 7) return;
  if (on == lastAppliedGateState[gate]) return;   // ← FIX (a): skip I2C if no change
  mcp.digitalWrite(kGateToMcpPin[gate], on ? HIGH : LOW);
  lastAppliedGateState[gate] = on;
}

// ═════════════════════════════════════════════════════════════════════
//  CAN FRAME TRACKER (for web dashboard)
// ═════════════════════════════════════════════════════════════════════

void trackCan(const twai_message_t& msg) {
  int slot = -1, freeSlot = -1, oldest = 0;
  uint32_t oldestMs = UINT32_MAX;
  for (int i = 0; i < CAN_TRACK_MAX; i++) {
    if (canTrack[i].used && canTrack[i].id == msg.identifier) { slot = i; break; }
    if (!canTrack[i].used && freeSlot < 0) freeSlot = i;
    if (canTrack[i].lastMs < oldestMs) { oldestMs = canTrack[i].lastMs; oldest = i; }
  }
  if (slot < 0) slot = (freeSlot >= 0) ? freeSlot : oldest;
  canTrack[slot].used = true;
  canTrack[slot].id   = msg.identifier;
  canTrack[slot].dlc  = msg.data_length_code;
  uint8_t n = msg.data_length_code; if (n > 8) n = 8;
  for (uint8_t i = 0; i < n; i++) canTrack[slot].data[i] = msg.data[i];
  for (uint8_t i = n; i < 8; i++) canTrack[slot].data[i] = 0;
  canTrack[slot].lastMs = millis();
}

// ═════════════════════════════════════════════════════════════════════
//  SIGNAL DECODING + BUTTON HANDLING
// ═════════════════════════════════════════════════════════════════════

void setSignalState(uint8_t signal, bool state) {
  switch (signal) {
    case SIG_PASSING: cmdPassing   = state; break;
    case SIG_DRIVING: cmdDriving   = state; break;
    case SIG_LEFT:    cmdLeftBlink = state; break;
    case SIG_RIGHT:   cmdRightBlink= state; break;
    case SIG_HORN:    cmdHorn      = state; break;
    case SIG_BRAKE:   cmdBrake     = state; break;
    case SIG_TAIL:    cmdTail      = state; break;
    case SIG_DRL:     cmdDrl       = state; break;
    case SIG_FOG:     cmdFog       = state; break;
  }
}

void decodeMappedSignalsFromCan(const twai_message_t& msg) {
  uint8_t dlc = msg.data_length_code;
  if (dlc == 0) return;
  if (dlc > 8) dlc = 8;

  uint16_t word = msg.data[0];
  if (dlc >= 2) word |= (uint16_t)(msg.data[1] << 8);

  uint32_t now = millis();

  // Update each mapped signal that listens to this CAN ID
  for (uint8_t s = 0; s <= SIG_MAX; s++) {
    if (signalMap[s].canId != msg.identifier) continue;
    uint8_t bit = signalMap[s].bit;
    if (bit / 8 >= dlc || bit > 15) continue;
    bool state = ((word >> bit) & 0x01) != 0;
    signalRawState[s] = state;
    setSignalState(s, state);
    cmdLastUpdateMs[s] = now;
  }

  // ── Button handling (configurable CAN source) ── FIX (b)
  // Long press  → toggle reverse latch (ignore short presses while latched)
  // Short press → one-shot mapping pulse in next 0x5FF
  if (msg.identifier == btnCanId && btnCanId != 0xFFFF) {
    uint8_t byteIdx = btnBit / 8;
    if (byteIdx < dlc && btnBit <= 15) {
      bool buttonNow = ((word >> btnBit) & 0x01) != 0;

      if (buttonNow && !buttonPrev) {
        // Rising edge — record press start
        buttonPressStartMs = now;
      }

      if (!buttonNow && buttonPrev) {
        // Falling edge — classify as short or long press
        uint32_t heldMs = now - buttonPressStartMs;
        if (heldMs >= BUTTON_LONG_PRESS_MS) {
          reverseLatched = !reverseLatched;
          logf("[BTN] long press -> reverse=%d", reverseLatched ? 1 : 0);
        } else if (!reverseLatched) {
          mappingPulsePending = true;
          logf("[BTN] short press -> mapping pulse");
        }
        // Short presses while reverseLatched are silently ignored.
      }

      buttonPrev = buttonNow;
    }
  }
}

// ═════════════════════════════════════════════════════════════════════
//  CAN RX
// ═════════════════════════════════════════════════════════════════════

void readCAN() {
  if (!canOk) return;
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    lastAnyCanRxMs = millis();
    trackCan(msg);

    if (msg.identifier == CAN_ID_LIGHT_CONTROL && msg.data_length_code >= 1) {
      lightWord = (uint16_t)(msg.data[0]);
      if (msg.data_length_code >= 2) lightWord |= (uint16_t)(msg.data[1] << 8);
      lightWordValid  = true;
      lastLightWordMs = millis();
    }

    decodeMappedSignalsFromCan(msg);

    if (msg.identifier == CAN_ID_THROTTLE && msg.data_length_code >= 1) {
      throttleByte = msg.data[0];
      throttleValid  = true;
      lastThrottleMs = millis();
    } else if (msg.identifier == CAN_ID_LYNX_STATUS && msg.data_length_code >= 3) {
      lynxMode       = msg.data[1];
      currentPowerMap = msg.data[2];
    }
  }
}

void applyInputTimeouts() {
  uint32_t now = millis();
  if (throttleValid && (now - lastThrottleMs > THROTTLE_STALE_MS))
    throttleValid = false;
  if (lightWordValid && (now - lastLightWordMs > CONTROL_STALE_MS))
    lightWordValid = false;

  for (uint8_t s = 0; s <= SIG_MAX; s++) {
    if (cmdLastUpdateMs[s] == 0) continue;
    if (now - cmdLastUpdateMs[s] > CONTROL_STALE_MS) {
      signalRawState[s] = false;
      setSignalState(s, false);
      cmdLastUpdateMs[s] = 0;
    }
  }
}

// ═════════════════════════════════════════════════════════════════════
//  OUTPUT STAGE
// ═════════════════════════════════════════════════════════════════════

bool outputFunctionState(uint8_t fn) {
  switch (fn) {
    case FUNC_OFF:        return false;
    case FUNC_ON:         return true;
    case FUNC_BRAKE:      return cmdBrake;
    case FUNC_DRL:        return cmdDrl;
    case FUNC_LEFT:       return cmdLeftBlink;
    case FUNC_RIGHT:      return cmdRightBlink;
    case FUNC_HORN:       return cmdHorn;
    case FUNC_FOG:        return cmdFog;
    case FUNC_DRIVING:    return cmdDriving;
    case FUNC_PASSING:    return cmdPassing;
    case FUNC_TAIL:       return cmdTail;
    case FUNC_REVERSE:    return reverseLatched;
    case FUNC_HAZARD:     return cmdLeftBlink || cmdRightBlink;
    case FUNC_TAIL_BRAKE: return cmdTail || cmdBrake;
    default:              return false;
  }
}

void applyOutputs() {
  if (!mcpOk) return;

  uint32_t now = millis();

  // CAN-loss safety: blink all outputs
  bool canLost = (now - lastAnyCanRxMs) >= CAN_LOSS_TIMEOUT_MS;
  if (canLost) {
    bool allOn = ((now / CAN_LOSS_BLINK_HALF_PERIOD_MS) % 2U) == 0U;
    for (uint8_t i = 0; i < 8; i++) mcpWriteGate(i, allOn);
    return;
  }

  if (webAutoMode) {
    for (uint8_t i = 0; i < 8; i++)
      mcpWriteGate(i, outputFunctionState(gateFunctionMap[i]));
  } else {
    for (uint8_t i = 0; i < 8; i++)
      mcpWriteGate(i, manualGateState[i]);
  }
}

// ═════════════════════════════════════════════════════════════════════
//  0x5FF SILIXCON CONTROL TX
//
//  Byte 0-1  INT16  CAN Level 1  (accelerator, 0 when braking)
//  Byte 2-3  INT16  CAN Level 2  (regen, = throttle when braking)
//  Byte 4-5  INT16  CAN Level 3  (always 0)
//  Byte 6    Digital inputs:
//              bit 0 = mapping command (one-shot)
//              bit 1 = reverse
//              bit 2 = digital brake
//              bit 3 = 0
//  Byte 7    0x00
// ═════════════════════════════════════════════════════════════════════

uint16_t scaledThrottle() {
  if (!throttleValid) return 0;
  uint32_t scaled = ((uint32_t)throttleByte * (uint32_t)THROTTLE_MAX_LEVEL) / 255UL;
  if (scaled > THROTTLE_MAX_LEVEL) scaled = THROTTLE_MAX_LEVEL;
  return (uint16_t)scaled;
}

void send5ffIfEnabled() {
  if (!canOk || !can5ffEnabled) return;

  uint32_t now = millis();
  if (now - last5ffTxMs < CAN_5FF_INTERVAL_MS) return;
  last5ffTxMs = now;

  uint16_t throttle = scaledThrottle();
  uint16_t lvl1 = cmdBrake ? 0        : throttle;   // Accel
  uint16_t lvl2 = cmdBrake ? throttle : 0;           // Regen
  uint16_t lvl3 = 0;

  uint8_t byte6 = 0;
  if (mappingPulsePending) byte6 |= (1 << 0);
  if (reverseLatched)      byte6 |= (1 << 1);
  if (cmdBrake)            byte6 |= (1 << 2);

  twai_message_t msg;
  msg.identifier       = CAN_ID_SILIXCON_CONTROL;
  msg.extd             = 0;
  msg.rtr              = 0;
  msg.ss               = 0;
  msg.self             = 0;
  msg.dlc_non_comp     = 0;
  msg.data_length_code = 8;
  msg.data[0] = (uint8_t)(lvl1 & 0xFF);
  msg.data[1] = (uint8_t)((lvl1 >> 8) & 0xFF);
  msg.data[2] = (uint8_t)(lvl2 & 0xFF);
  msg.data[3] = (uint8_t)((lvl2 >> 8) & 0xFF);
  msg.data[4] = (uint8_t)(lvl3 & 0xFF);
  msg.data[5] = (uint8_t)((lvl3 >> 8) & 0xFF);
  msg.data[6] = byte6;
  msg.data[7] = 0x00;

  if (twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK) {
    last5ffValid  = true;
    last5ffSentMs = millis();
    for (uint8_t i = 0; i < 8; i++) last5ffData[i] = msg.data[i];
    if (mappingPulsePending) mappingPulsePending = false;
  }
}

// ═════════════════════════════════════════════════════════════════════
//  I2C SENSORS
// ═════════════════════════════════════════════════════════════════════

void readI2cSensors() {
  uint32_t now = millis();
  if (now - lastSensorReadMs < 200) return;
  lastSensorReadMs = now;

  if (ads48Ok) {
    senseRaw[3] = ads48.readADC_SingleEnded(0);
    senseRaw[2] = ads48.readADC_SingleEnded(1);
    senseRaw[1] = ads48.readADC_SingleEnded(2);
    senseRaw[0] = ads48.readADC_SingleEnded(3);
  }
  if (ads49Ok) {
    senseRaw[7] = ads49.readADC_SingleEnded(0);
    senseRaw[6] = ads49.readADC_SingleEnded(1);
    senseRaw[5] = ads49.readADC_SingleEnded(2);
    senseRaw[4] = ads49.readADC_SingleEnded(3);
  }
  for (uint8_t i = 0; i < 8; i++)
    senseCurrentA[i] = ((float)senseRaw[i]) * 0.1875f;

  if (vemlOk) luxValue = veml.readLux(VEML_LUX_NORMAL_NOWAIT);
}

// ═════════════════════════════════════════════════════════════════════
//  WEB UI
// ═════════════════════════════════════════════════════════════════════

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Velion Lightboard</title>
<style>
body{font-family:Consolas,monospace;background:#0c1118;color:#d9e6f2;margin:0;padding:14px}
h1,h2{margin:8px 0}
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:10px}
.card{background:#111b27;border:1px solid #2c425c;border-radius:8px;padding:10px}
table{width:100%;border-collapse:collapse}
th,td{border:1px solid #2c425c;padding:5px;font-size:12px;text-align:left}
th{background:#152638}
button{background:#1f3a56;color:#d9e6f2;border:1px solid #436487;padding:5px 8px;cursor:pointer;border-radius:4px;font-size:11px}
.pill{display:inline-block;padding:2px 6px;border:1px solid #436487;border-radius:999px;margin-left:4px}
.ok{color:#58d68d}.bad{color:#ff7f7f}
select,input{background:#0d1722;color:#d9e6f2;border:1px solid #436487;padding:3px;font-size:12px}
.term{background:#0a111a;border:1px solid #203042;max-height:220px;overflow:auto;padding:6px;white-space:pre-wrap;font-size:11px}
.sv{color:#58d68d;font-size:11px;margin-left:6px}
</style>
</head><body>
<h1>Velion Lightboard</h1>
<div class="cards">

<div class="card"><h2>Device</h2>
<table>
<tr><td>SSID</td><td><input id="ssid" size="18" maxlength="31"> <input id="pass" size="12" maxlength="31" placeholder="password"> <button onclick="saveSsid()">Save AP</button><span id="ssidS" class="sv"></span></td></tr>
<tr><td>Mode</td><td><button onclick="setMode('auto')">AUTO</button> <button onclick="setMode('manual')">MANUAL</button> <span id="modeV"></span></td></tr>
<tr><td>0x5FF TX</td><td><button onclick="setTx5ff(1)">Enable</button> <button onclick="setTx5ff(0)">Disable</button> <span id="txV"></span></td></tr>
</table>
<div id="info" style="margin-top:6px;font-size:11px"></div>
</div>

<div class="card"><h2>MOSFET Gate Mapping</h2>
<button onclick="resetMap('front')">Front Defaults</button> <button onclick="resetMap('rear')">Rear Defaults</button>
<div id="gates"></div></div>

<div class="card"><h2>CAN Signal Sources</h2><div id="sigcfg"></div></div>

<div class="card"><h2>Long/Short Button</h2>
<table><tr><td>CAN ID</td><td>0x<input id="bCid" type="text" size="4" maxlength="4"></td>
<td>Bit</td><td><input id="bBit" type="number" min="0" max="15" style="width:3em"></td>
<td><button onclick="saveBtn()">Save</button><span id="btnS" class="sv"></span></td></tr></table>
<div id="btnSt" style="margin-top:4px;font-size:11px"></div>
</div>

<div class="card"><h2>0x5FF Frame</h2><div id="tx5ff"></div></div>
<div class="card"><h2>Decoded Signals</h2><div id="decoded"></div></div>
<div class="card"><h2>I2C / Sensors</h2><div id="i2c"></div><div id="sens" style="margin-top:6px"></div></div>
<div class="card" style="grid-column:1/-1"><h2>Serial</h2><div id="serial" class="term"></div></div>
<div class="card" style="grid-column:1/-1"><h2>CAN Bus</h2><div id="can" class="term"></div></div>
</div>
<script>
let cfgDone=false;
function yn(v){return v?'<span class="ok">OK</span>':'<span class="bad">FAIL</span>';}
function sv(id){let e=document.getElementById(id);if(e){e.textContent='Saved!';setTimeout(()=>e.textContent='',2000);}}
async function setMode(m){await fetch('/api/mode?mode='+m);}
async function setGate(i,v){await fetch('/api/gate?idx='+i+'&val='+(v?1:0));}
async function setTx5ff(v){await fetch('/api/config?tx5ff='+(v?1:0));}
async function saveGateMap(g){await fetch('/api/config?gate='+g+'&fn='+document.getElementById('gm_'+g).value);sv('gs_'+g);}
async function saveSigMap(i){
  let c=document.getElementById('sc_'+i).value;
  let b=document.getElementById('sb_'+i).value;
  await fetch('/api/sigmap?sig='+i+'&canId='+c+'&bit='+b);sv('ss_'+i);
}
async function saveSsid(){
  let s=encodeURIComponent(document.getElementById('ssid').value);
  let p=encodeURIComponent(document.getElementById('pass').value);
  await fetch('/api/config?ssid='+s+'&pass='+p);sv('ssidS');
}
async function saveBtn(){
  let c=document.getElementById('bCid').value;
  let b=document.getElementById('bBit').value;
  await fetch('/api/config?btnCid='+c+'&btnBit='+b);sv('btnS');
}
async function resetMap(role){await fetch('/api/config?reset='+role);cfgDone=false;await poll();}

function gatesHtml(d){
  let h='<table><tr><th>Gate</th><th>Function</th><th></th><th>State</th><th>Manual</th></tr>';
  for(let i=0;i<8;i++){
    h+='<tr><td>'+i+'</td><td><select id="gm_'+i+'">';
    for(const o of d.fnOpt){h+='<option value="'+o.v+'"'+(o.v===d.map[i]?' selected':'')+'>'+o.n+'</option>';}
    h+='</select> <button onclick="saveGateMap('+i+')">Save</button><span id="gs_'+i+'" class="sv"></span></td>';
    h+='<td class="'+(d.applied[i]?'ok':'')+'">'+( d.applied[i]?'ON':'off')+'</td>';
    h+='<td><button onclick="setGate('+i+',1)">ON</button> <button onclick="setGate('+i+',0)">OFF</button></td></tr>';
  }
  return h+'</table>';
}

function sigHtml(d){
  let h='<table><tr><th>Signal</th><th>CAN ID (hex)</th><th>Bit</th><th>Raw</th><th>Val</th><th></th></tr>';
  for(let i=0;i<d.sig.length;i++){const s=d.sig[i];
    h+='<tr><td>'+s.n+'</td><td><input id="sc_'+i+'" size="4" maxlength="4" value="'+s.ch+'"></td>';
    h+='<td><input id="sb_'+i+'" type="number" min="0" max="15" style="width:3em" value="'+s.bit+'"></td>';
    h+='<td>'+(s.raw?'1':'0')+'</td><td>'+(s.v?'1':'0')+'</td>';
    h+='<td><button onclick="saveSigMap('+i+')">Save</button><span id="ss_'+i+'" class="sv"></span></td></tr>';}
  return h+'</table>';
}

async function poll(){try{
  const d=await(await fetch('/api/state')).json();
  const c=await(await fetch('/api/can')).json();
  const l=await(await fetch('/api/log')).json();

  document.getElementById('modeV').textContent=d.auto?'AUTO':'MANUAL';
  document.getElementById('txV').textContent=d.tx5en?'ON':'OFF';
  document.getElementById('info').innerHTML='AP: <b>'+d.ssid+'</b> | 0x400: 0x'+d.lwHex+(d.lwOk?' (ok)':' (stale)')
    +' | Throttle: '+(d.thrV>=0?d.thrV:'stale')+' | LYNX: '+d.lynx+' ('+d.lynxN+') | PwrMap: '+d.pmap;

  document.getElementById('btnSt').innerHTML='Reverse: <b>'+(d.rev?'ON':'off')+'</b> | MapPulse: '+(d.mpulse?'PENDING':'off');

  if(!cfgDone){
    document.getElementById('ssid').value=d.ssid;
    document.getElementById('pass').value=d.apPass;
    document.getElementById('bCid').value=d.bCidH;
    document.getElementById('bBit').value=d.bBit;
    document.getElementById('gates').innerHTML=gatesHtml(d);
    document.getElementById('sigcfg').innerHTML=sigHtml(d);
    cfgDone=true;
  }

  // 0x5FF card
  if(!d.txOk){document.getElementById('tx5ff').textContent='No TX yet';}
  else{let b='';for(let i=0;i<8;i++){let h=d.txD[i].toString(16).toUpperCase();if(h.length<2)h='0'+h;b+=h+(i<7?' ':'');}
    document.getElementById('tx5ff').innerHTML='Age: '+d.txAge+'ms<br>Data: <code>'+b+'</code>';}

  // Decoded
  let ds='';for(const k of['brake','tail','drl','left','right','horn','driving','passing','fog'])
    ds+=k+': <span class="pill">'+(d[k]?'1':'0')+'</span> ';
  document.getElementById('decoded').innerHTML=ds;

  // I2C
  document.getElementById('i2c').innerHTML='MCP:'+yn(d.mcp)+' ADS48:'+yn(d.a48)+' ADS49:'+yn(d.a49)+' VEML:'+yn(d.vml);
  let sh='<table><tr><th>Ch</th><th>Raw</th><th>A</th></tr>';
  for(let i=0;i<8;i++)sh+='<tr><td>'+i+'</td><td>'+d.sr[i]+'</td><td>'+d.sa[i].toFixed(2)+'</td></tr>';
  sh+='<tr><td>Lux</td><td colspan="2">'+d.lux.toFixed(1)+'</td></tr></table>';
  document.getElementById('sens').innerHTML=sh;

  document.getElementById('serial').textContent=l.lines?l.lines.join('\n'):'';
  if(!c.entries.length){document.getElementById('can').textContent='No packets';}
  else{let h='';for(const e of c.entries)h+=e.id+' | dlc='+e.dlc+' | '+e.data+' | '+e.age+'ms\n';
    document.getElementById('can').textContent=h;}
}catch(e){}}
setInterval(poll,1500);poll();
</script>
</body></html>
)rawliteral";
  webServer.send(200, "text/html", html);
}

// ═════════════════════════════════════════════════════════════════════
//  WEB API HANDLERS
// ═════════════════════════════════════════════════════════════════════

void handleApiMode() {
  if (!webServer.hasArg("mode")) { webServer.send(400,"application/json","{\"ok\":false}"); return; }
  String m = webServer.arg("mode");
  if      (m == "auto")   webAutoMode = true;
  else if (m == "manual") webAutoMode = false;
  else { webServer.send(400,"application/json","{\"ok\":false}"); return; }
  webServer.send(200,"application/json","{\"ok\":true}");
}

void handleApiGate() {
  uint8_t idx = 0;
  if (!parseUIntArg("idx", 0, 7, idx) || !webServer.hasArg("val")) {
    webServer.send(400,"application/json","{\"ok\":false}"); return;
  }
  manualGateState[idx] = (webServer.arg("val") == "1");
  webServer.send(200,"application/json","{\"ok\":true}");
}

void handleApiConfig() {
  bool changed = false;

  if (webServer.hasArg("ssid")) {
    String s = webServer.arg("ssid");
    if (s.length() > 0 && s.length() < 32) {
      s.toCharArray(apSsid, sizeof(apSsid));
      changed = true;
    }
  }
  if (webServer.hasArg("pass")) {
    String p = webServer.arg("pass");
    if (p.length() < 32) {
      p.toCharArray(apPass, sizeof(apPass));
      changed = true;
    }
  }
  if (webServer.hasArg("tx5ff")) {
    can5ffEnabled = (webServer.arg("tx5ff") == "1");
    changed = true;
    logf("[CFG] 0x5FF TX %s", can5ffEnabled ? "ON" : "OFF");
  }
  if (webServer.hasArg("btnCid")) {
    char* endPtr = NULL;
    String cidStr = webServer.arg("btnCid");
    long v = strtol(cidStr.c_str(), &endPtr, 16);
    if (endPtr != cidStr.c_str() && v >= 0 && v <= 0x7FF) {
      btnCanId = (uint16_t)v;
      changed = true;
    }
  }
  if (webServer.hasArg("btnBit")) {
    uint8_t b = 0;
    if (parseUIntArg("btnBit", 0, 15, b)) { btnBit = b; changed = true; }
  }
  if (webServer.hasArg("gate") && webServer.hasArg("fn")) {
    uint8_t gate = 0, fn = 0;
    if (parseUIntArg("gate", 0, 7, gate) && parseUIntArg("fn", 0, FUNC_MAX, fn)) {
      gateFunctionMap[gate] = fn;
      changed = true;
      logf("[CFG] gate %u -> %s", (unsigned)gate, functionName(fn));
    }
  }
  if (webServer.hasArg("reset")) {
    String r = webServer.arg("reset");
    const uint8_t* def = (r == "front") ? kDefaultFrontMap : kDefaultRearMap;
    for (uint8_t i = 0; i < 8; i++) gateFunctionMap[i] = def[i];
    changed = true;
    logf("[CFG] gate map reset to %s defaults", r.c_str());
  }

  if (changed) {
    savePersistentConfig();
    if (webServer.hasArg("ssid") || webServer.hasArg("pass")) {
      WiFi.softAPdisconnect(true);
      WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
      WiFi.softAP(apSsid, apPass);
      logf("[CFG] AP restarted: %s", apSsid);
    }
  }
  webServer.send(200,"application/json","{\"ok\":true}");
}

void handleApiSigMap() {
  if (!webServer.hasArg("sig") || !webServer.hasArg("canId") || !webServer.hasArg("bit")) {
    webServer.send(400,"application/json","{\"ok\":false}"); return;
  }
  uint8_t sig = 0;
  if (!parseUIntArg("sig", 0, SIG_MAX, sig)) {
    webServer.send(400,"application/json","{\"ok\":false}"); return;
  }
  char* endPtr = NULL;
  String cidStr = webServer.arg("canId");
  long cid = strtol(cidStr.c_str(), &endPtr, 16);
  if (endPtr == cidStr.c_str() || cid < 0 || cid > 0xFFFF) {
    webServer.send(400,"application/json","{\"ok\":false}"); return;
  }
  uint8_t bit = 0;
  if (!parseUIntArg("bit", 0, 15, bit)) {
    webServer.send(400,"application/json","{\"ok\":false}"); return;
  }
  signalMap[sig].canId = (uint16_t)cid;
  signalMap[sig].bit   = bit;
  savePersistentConfig();
  logf("[SIG] %s -> 0x%03X bit%d", signalName(sig), (int)cid, (int)bit);
  webServer.send(200,"application/json","{\"ok\":true}");
}

void handleApiState() {
  char lwHex[5]; snprintf(lwHex, sizeof(lwHex), "%04X", lightWord);
  char bCidH[5]; snprintf(bCidH, sizeof(bCidH), "%03X", btnCanId);

  String o = "{";
  o += "\"ssid\":\"" + String(apSsid) + "\",";
  o += "\"apPass\":\"" + String(apPass) + "\",";
  o += "\"auto\":" + String(webAutoMode ? "true" : "false") + ",";
  o += "\"lwHex\":\"" + String(lwHex) + "\",";
  o += "\"lwOk\":" + String(lightWordValid ? "true" : "false") + ",";
  o += "\"thrV\":" + String(throttleValid ? (int)throttleByte : -1) + ",";
  o += "\"lynx\":" + String((int)lynxMode) + ",";
  o += "\"lynxN\":\"" + String(lynxModeName(lynxMode)) + "\",";
  o += "\"pmap\":" + String((int)currentPowerMap) + ",";
  o += "\"tx5en\":" + String(can5ffEnabled ? "true" : "false") + ",";
  o += "\"rev\":" + String(reverseLatched ? "true" : "false") + ",";
  o += "\"mpulse\":" + String(mappingPulsePending ? "true" : "false") + ",";
  o += "\"txOk\":" + String(last5ffValid ? "true" : "false") + ",";
  o += "\"txAge\":" + String(last5ffValid ? (unsigned long)(millis() - last5ffSentMs) : 0UL) + ",";
  o += "\"mcp\":" + String(mcpOk ? "true" : "false") + ",";
  o += "\"a48\":" + String(ads48Ok ? "true" : "false") + ",";
  o += "\"a49\":" + String(ads49Ok ? "true" : "false") + ",";
  o += "\"vml\":" + String(vemlOk ? "true" : "false") + ",";
  o += "\"passing\":" + String(cmdPassing ? "true" : "false") + ",";
  o += "\"driving\":" + String(cmdDriving ? "true" : "false") + ",";
  o += "\"left\":" + String(cmdLeftBlink ? "true" : "false") + ",";
  o += "\"right\":" + String(cmdRightBlink ? "true" : "false") + ",";
  o += "\"horn\":" + String(cmdHorn ? "true" : "false") + ",";
  o += "\"brake\":" + String(cmdBrake ? "true" : "false") + ",";
  o += "\"tail\":" + String(cmdTail ? "true" : "false") + ",";
  o += "\"drl\":" + String(cmdDrl ? "true" : "false") + ",";
  o += "\"fog\":" + String(cmdFog ? "true" : "false") + ",";
  o += "\"bCidH\":\"" + String(bCidH) + "\",";
  o += "\"bBit\":" + String((int)btnBit) + ",";

  o += "\"manual\":[";
  for (uint8_t i = 0; i < 8; i++) { if (i) o += ","; o += manualGateState[i] ? "true" : "false"; }
  o += "],\"applied\":[";
  for (uint8_t i = 0; i < 8; i++) { if (i) o += ","; o += lastAppliedGateState[i] ? "true" : "false"; }
  o += "],\"map\":[";
  for (uint8_t i = 0; i < 8; i++) { if (i) o += ","; o += String((int)gateFunctionMap[i]); }
  o += "],\"fnOpt\":[";
  for (uint8_t f = 0; f <= FUNC_MAX; f++) { if (f) o += ","; o += "{\"v\":" + String((int)f) + ",\"n\":\"" + String(functionName(f)) + "\"}"; }

  o += "],\"sig\":[";
  for (uint8_t s = 0; s <= SIG_MAX; s++) {
    if (s) o += ",";
    bool v = false;
    switch (s) {
      case SIG_PASSING: v = cmdPassing;   break; case SIG_DRIVING: v = cmdDriving;   break;
      case SIG_LEFT:    v = cmdLeftBlink;  break; case SIG_RIGHT:   v = cmdRightBlink; break;
      case SIG_HORN:    v = cmdHorn;       break; case SIG_BRAKE:   v = cmdBrake;      break;
      case SIG_TAIL:    v = cmdTail;       break; case SIG_DRL:     v = cmdDrl;        break;
      case SIG_FOG:     v = cmdFog;        break;
    }
    char ch[5]; snprintf(ch, sizeof(ch), "%03X", signalMap[s].canId);
    o += "{\"n\":\"" + String(signalName(s)) + "\",\"ch\":\"" + String(ch) + "\",\"bit\":" + String((int)signalMap[s].bit);
    o += ",\"raw\":" + String(signalRawState[s] ? "true" : "false") + ",\"v\":" + String(v ? "true" : "false") + "}";
  }

  o += "],\"txD\":[";
  for (uint8_t i = 0; i < 8; i++) { if (i) o += ","; o += String((int)last5ffData[i]); }
  o += "],\"sr\":[";
  for (uint8_t i = 0; i < 8; i++) { if (i) o += ","; o += String((int)senseRaw[i]); }
  o += "],\"sa\":[";
  for (uint8_t i = 0; i < 8; i++) { if (i) o += ","; o += String(senseCurrentA[i], 3); }
  o += "],\"lux\":" + String(luxValue, 2);
  o += "}";
  webServer.send(200, "application/json", o);
}

void handleApiCan() {
  int idx[CAN_TRACK_MAX];
  int count = 0;
  for (int i = 0; i < CAN_TRACK_MAX; i++)
    if (canTrack[i].used) idx[count++] = i;
  // Sort by CAN ID ascending
  for (int i = 1; i < count; i++) {
    int key = idx[i]; int j = i - 1;
    while (j >= 0 && canTrack[idx[j]].id > canTrack[key].id) { idx[j+1] = idx[j]; j--; }
    idx[j+1] = key;
  }
  uint32_t now = millis();
  String o = "{\"entries\":[";
  for (int n = 0; n < count; n++) {
    if (n) o += ",";
    const CanTrack& e = canTrack[idx[n]];
    char ib[6]; snprintf(ib, sizeof(ib), "0x%03lX", (unsigned long)e.id);
    o += "{\"id\":\"" + String(ib) + "\",\"dlc\":" + String((int)e.dlc) + ",\"data\":\"";
    for (uint8_t b = 0; b < e.dlc && b < 8; b++) {
      o += hexByte(e.data[b]);
      if (b + 1 < e.dlc && b < 7) o += " ";
    }
    o += "\",\"age\":" + String((unsigned long)(now - e.lastMs)) + "}";
  }
  o += "]}";
  webServer.send(200, "application/json", o);
}

void handleApiLog() {
  String o = "{\"lines\":[";
  for (uint8_t i = 0; i < serialLogCount; i++) {
    if (i) o += ",";
    uint8_t idx = (uint8_t)((serialLogHead + SERIAL_LOG_MAX - serialLogCount + i) % SERIAL_LOG_MAX);
    String line = serialLog[idx];
    line.replace("\\", "\\\\");
    line.replace("\"", "\\\"");
    o += "\"" + line + "\"";
  }
  o += "]}";
  webServer.send(200, "application/json", o);
}

// ═════════════════════════════════════════════════════════════════════
//  HARDWARE SETUP
// ═════════════════════════════════════════════════════════════════════

void setupMcp() {
  mcpOk = mcp.begin_I2C(0x20);
  if (!mcpOk) { logLine("[I2C] MCP23017 0x20 not found"); return; }
  for (uint8_t g = 0; g < 8; g++) {
    mcp.pinMode(kGateToMcpPin[g], OUTPUT);
    mcp.digitalWrite(kGateToMcpPin[g], LOW);
    lastAppliedGateState[g] = false;
  }
}

void setupSensors() {
  ads48Ok = ads48.begin(0x48, &Wire);
  ads49Ok = ads49.begin(0x49, &Wire);
  if (ads48Ok) ads48.setGain(GAIN_TWOTHIRDS);
  if (ads49Ok) ads49.setGain(GAIN_TWOTHIRDS);
  vemlOk = veml.begin();
  if (vemlOk) {
    veml.setGain(VEML7700_GAIN_1_8);
    veml.setIntegrationTime(VEML7700_IT_100MS);
  }
}

void setupCAN() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) { logLine("[CAN] install failed"); canOk = false; return; }
  if (twai_start() != ESP_OK) { logLine("[CAN] start failed"); canOk = false; return; }
  canOk = true;
}

void setupWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  WiFi.softAP(apSsid, apPass);

  webServer.on("/",           handleRoot);
  webServer.on("/api/state",  handleApiState);
  webServer.on("/api/can",    handleApiCan);
  webServer.on("/api/mode",   handleApiMode);
  webServer.on("/api/gate",   handleApiGate);
  webServer.on("/api/config", handleApiConfig);
  webServer.on("/api/sigmap", handleApiSigMap);
  webServer.on("/api/log",    handleApiLog);
  webServer.begin();
}

// ═════════════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ═════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(PIN_CAN_S, OUTPUT);
  digitalWrite(PIN_CAN_S, LOW);
  serialLogPush("[BOOT] init");

  loadPersistentConfig();

  Wire.begin(PIN_SDA, PIN_SCL);
  lastAnyCanRxMs = millis();

  setupMcp();
  setupSensors();
  logf("[I2C] MCP=%s ADS48=%s ADS49=%s VEML=%s",
       mcpOk ? "OK" : "FAIL", ads48Ok ? "OK" : "FAIL",
       ads49Ok ? "OK" : "FAIL", vemlOk ? "OK" : "FAIL");
  setupCAN();
  setupWeb();

  logf("[BOOT] AP=%s  5FF=%s", apSsid, can5ffEnabled ? "ON" : "OFF");
}

void loop() {
  readCAN();
  applyInputTimeouts();
  readI2cSensors();
  applyOutputs();
  send5ffIfEnabled();
  webServer.handleClient();
  delay(1);
}
