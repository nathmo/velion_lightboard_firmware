#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_NeoPixel.h>

// ---------------- I2C devices ----------------
Adafruit_MCP23X17 mcp;     // 0x20
Adafruit_ADS1115 ads0;    // 0x48
Adafruit_ADS1115 ads1;    // 0x49
Adafruit_VEML7700 veml;   // 0x10 default

// ---------------- WS2812 ----------------
#define PIN_RGBLED_DIN 8
#define LED_COUNT 5
Adafruit_NeoPixel strip(LED_COUNT, PIN_RGBLED_DIN, NEO_GRB + NEO_KHZ800);

// ---------------- MOSFET mapping ----------------
// MCP23017 @ 0x20
// GPB0..7 = MOSFET_GATE_7..0
// GPA0..2 = EN_PIEZO_A/B/C

// ---------------- Timing ----------------
unsigned long lastMosfetSwitch = 0;
unsigned long lastCurrentPrint = 0;
unsigned long lastLuxPrint = 0;
unsigned long lastLedStep = 0;

int mosfetIndex = 0;
int ledState = 0;

// ---------------- Constants ----------------
const float SHUNT = 0.001f;     // 1 mOhm
const float ADS_LSB = 0.125e-3; // ADS1115 @ GAIN_ONE = 0.125 mV/bit

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Serial.println("test");
  Wire.begin(3, 4); // SDA=3, SCL=4
  Serial.println("Booted");
  for (int i = 0; i < 3; i++) {
    Serial.print("---- I2C scan #");
    Serial.print(i + 1);
    Serial.println(" ----");
    scanI2C();
    delay(1000);
  }

  // MCP23017
  if (!mcp.begin_I2C(0x20)) {
    Serial.println("MCP23017 not found!");
    while (1);
  }

  for (int i = 0; i < 8; i++) mcp.pinMode(8 + i, OUTPUT); // GPB0..7
  for (int i = 0; i < 3; i++) mcp.pinMode(i, OUTPUT);     // GPA0..2

  allMosfetsOff();

  // ADS1115
  /*
  if (!ads0.begin(0x48)) {
    Serial.println("ADS1115 (0x48) not found!");
    while (1);
  }

  if (!ads1.begin(0x49)) {
    Serial.println("ADS1115 (0x49) not found!");
    while (1);
  }

  ads0.setGain(GAIN_ONE);
  ads1.setGain(GAIN_ONE);
*/
// ---------------- VEML7700 ----------------
if (!veml.begin()) {
  Serial.println("VEML7700 not found at 0x10");
  while (1);
}

// Optional: make it explicit
veml.setGain(VEML7700_GAIN_1);
veml.setIntegrationTime(VEML7700_IT_100MS);

  // WS2812
  strip.begin();
  strip.show();
}

// ---------------- Helpers ----------------

void scanI2C() {
  Serial.println("I2C scan start");

  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }

  if (found == 0) Serial.println("  No I2C devices found");
  Serial.println("I2C scan end\n");
}


void allMosfetsOff() {
  for (int i = 0; i < 8; i++) mcp.digitalWrite(8 + i, LOW);
}

void setMosfet(int ch) {
  allMosfetsOff();
  // GPB7..0 = MOSFET 0..7
  int pin = 8 + (7 - ch);
  mcp.digitalWrite(pin, HIGH);
}

float readCurrent(int ch) {
  int16_t raw;
  if (ch < 4) raw = ads0.readADC_SingleEnded(ch);
  else raw = ads1.readADC_SingleEnded(ch - 4);

  float voltage = raw * ADS_LSB;   // volts
  return voltage / SHUNT;          // amps
}

void stepLed() {
  uint32_t color;
  if (ledState == 0) color = strip.Color(255, 0, 0);
  else if (ledState == 1) color = strip.Color(0, 255, 0);
  else color = strip.Color(0, 0, 255);

  for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, color);
  strip.show();

  ledState = (ledState + 1) % 3;
}

// ---------------- Loop ----------------
void loop() {
  unsigned long now = millis();
  bool led = 0;
  // ---- MOSFET sequencing ----
  if (now - lastMosfetSwitch >= 1000) {
    setMosfet(mosfetIndex);
    Serial.print("MOSFET ");
    Serial.print(mosfetIndex);
    Serial.println(" ON");

    mosfetIndex = (mosfetIndex + 1) % 8;
    lastMosfetSwitch = now;
  }

  // ---- Current printing ----
  /*
  if (now - lastCurrentPrint >= 500) {
    Serial.print("Currents [A]: ");
    for (int i = 0; i < 8; i++) {
      Serial.print(readCurrent(i), 3);
      if (i < 7) Serial.print(", ");
    }
    Serial.println();
    lastCurrentPrint = now;
  }

  // ---- Lux printing ----
  if (now - lastLuxPrint >= 1000) {
    float lux = veml.readLux();
    Serial.print("Lux: ");
    Serial.println(lux);
    lastLuxPrint = now;
  }
*/
  // ---- LED cycling ----
  if (now - lastLedStep >= 1000) {
    stepLed();
    //digitalWrite(LED_BUILTIN, led);  // turn the LED on (HIGH is the voltage level)
    led = !led;
    lastLedStep = now;
  }
  if (now - lastLuxPrint >= 1000) {
    float lux = veml.readLux();

    Serial.print("Ambient light: ");
    Serial.print(lux, 2);
    Serial.println(" lx");

    lastLuxPrint = now;
  }
}
