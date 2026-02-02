# velion_lightboard_firmware
This is the firmware that control the main board and control pannel of the vehicle.

# General logic
Use ESP32-C3 FreeRTOS on Arduino  IDE (ESP32 core 2.0.18)

there is a mainloop which first read the input, generate event. then iterrate over all state machine and feed them the first event untill all event are consumed. then there the output stage which run the PID step and the output monitoring.

INPUTS Loop (1kHz mini)
get luminosity sensor data
get current ADC data
get distance data from sonar A B C

LOGIC FSMs
execute each FSM with the event queue

OUTPUT 
apply the state to the mosfet output

# Pin mapping 
GPIO 0 : Analog input  (PIN_SONAR_ECHO_A)
GPIO 1 : Analog input  (PIN_SONAR_ECHO_B)
GPIO 2 : Analog input  (PIN_SONAR_ECHO_C)
GPIO 3 : SDA (has a pullup of 4k7 aldready )
GPIO 4 : SCL (has a pullup of 4k7 aldready )
GPIO 5 : CAN_TX
GPIO 6 : CAN_RX
GPIO 7 : CAN_S (TJA1051T-3)
GPIO 8 : WS2812 led strip (PIN_RGBLED_DIN)
GPIO 9 : digital out PIN_PIEZO_PWM_HALF_X
GPIO 10 : digital out PIN_PIEZO_PWM_HALF_Y

I have two ADC1115IDGS luminosity sensor on the I2C bus with address 1001000 -> 0x48 and 1001001 -> 0x49.
on the 0x48, AIN0 = SENSE_3, AIN1 = SENSE_2, AIN2 = SENSE_1,AIN3 = SENSE_0 

on the 0x49, AIN0 = SENSE_7, AIN1 = SENSE_6, AIN2 = SENSE_5,AIN3 = SENSE_4

for both ADC, they are mesuring the current trough a 1 milli Ohm resistor with the voltage difference accross it to GND.

I also have a VEML7700 with address 0x10 luminosity sensor on the I2C bus

I have one MCP23017_SO over I2C whose IO I also want to control like if it was a GPIO of the ESP32.

the one with address 0100000 = 0x20 has : 
GPB0 : MOSFET_GATE_7 (output)
GPB1 : MOSFET_GATE_6 (output)
GPB2 : MOSFET_GATE_5 (output)
GPB3 : MOSFET_GATE_4 (output)
GPB4 : MOSFET_GATE_3 (output)
GPB5 : MOSFET_GATE_2 (output)
GPB6 : MOSFET_GATE_1 (output)
GPB7 : MOSFET_GATE_0 (output)

GPA0 : EN_PIEZO_A
GPA1 : EN_PIEZO_B
GPA2 : EN_PIEZO_C

# CAN codes


This catalog defines **standard 11-bit CAN 2.0 IDs (0x000–0x7FF)** for the front (`0x17`) and rear (`0x16`) lightboards.  
Device addressing is handled by **distinct CAN IDs per board**, so no extra address byte is needed in payloads.

**All data is little-endian. DLC is the exact length used.**

---

# Front Lightboard (`0x17`) — CAN IDs

## Input (Control/Set)

| CAN ID | DLC | Purpose | Payload |
|--------|-----|---------|---------|
| `0x120` | 1 | Set all front outputs | **Byte 0:** 8-bit control (see Front Bit Mapping) |
| `0x121` | 5 | Set current thresholds for alert | **Byte 0:** Channel (0–7)<br>**Bytes 1–2:** Low threshold mA (0 = disable)<br>**Bytes 3–4:** High threshold mA (0 = disable) |
| Bit | Output |
|-----|--------|
| 0 | DRL |
| 1 | Position Left |
| 2 | Position Right |
| 3 | Left Blinker |
| 4 | Right Blinker |
| 5 | Low Beam |
| 6 | High Beam |
| 7 | Horn |

## Output (Status/Telemetry)

| CAN ID | DLC | Purpose | Payload |
|--------|-----|---------|---------|
| `0x131` | 1 | Front output status | **Byte 0:** 8-bit status (same bit mapping as control) |
| `0x132` | 5 | Report current thresholds | Same payload layout as `0x121` |
| `0x140`–`0x147` | 2 | Current per channel | **Bytes 0–1:** INT16 current in mA for channel 0–7 (CAN ID = `0x140 + channel`) |
| `0x150` | 3 | Front proximity | **Byte 0:** Left sensor cm<br>**Byte 1:** Center sensor cm<br>**Byte 2:** Right sensor cm |
| `0x151` | 2 | Ambient light | **Bytes 0–1:** Fixed-exponent lux (see encoding below) |

---

# Rear Lightboard (`0x16`) — CAN IDs

## Input (Control/Set)

| CAN ID | DLC | Purpose | Payload |
|--------|-----|---------|---------|
| `0x220` | 1 | Set all front outputs | **Byte 0:** 8-bit control (see Front Bit Mapping) |
| `0x221` | 5 | Set current thresholds for alert | **Byte 0:** Channel (0–7)<br>**Bytes 1–2:** Low threshold mA (0 = disable)<br>**Bytes 3–4:** High threshold mA (0 = disable) |

| Bit | Output |
|-----|--------|
| 0 | Brake |
| 1 | Left Blinker |
| 2 | Right Blinker |
| 3 | Rear Position |
| 4 | Fog |
| 5 | Reverse |
| 6 | License Plate |
| 7 | Reserved |

## Output (Status/Telemetry)

| CAN ID | DLC | Purpose | Payload |
|--------|-----|---------|---------|
| `0x231` | 1 | Front output status | **Byte 0:** 8-bit status (same bit mapping as control) |
| `0x232` | 5 | Report current thresholds | Same payload layout as `0x121` |
| `0x240`–`0x247` | 2 | Current per channel | **Bytes 0–1:** INT16 current in mA for channel 0–7 (CAN ID = `0x240 + channel`) |
| `0x250` | 3 | Front proximity | **Byte 0:** Left sensor cm<br>**Byte 1:** Center sensor cm<br>**Byte 2:** Right sensor cm |
| `0x251` | 2 | Ambient light | **Bytes 0–1:** Fixed-exponent lux (see encoding below) |

---

## Proximity Encoding (All Boards)
- **Byte value 0:** Sensor error / dirty
- **Byte value 255:** No object in range
- **Byte value 1–254:** Distance in cm

---

## Ambient Light Encoding (VEML7700 Fixed Exponent, 0.1–120k lux)

**16-bit value:**
- **Bits 15–12:** Exponent `E` (0–15)
- **Bits 11–0:** Mantissa `M` (0–4095)

**Lux calculation:**
```
lux = M × 2^E × 0.1
```

---

## Full Packet Examples
### Example 1 — Front: Turn DRL + Horn ON
| CAN ID | DLC | Data |
|--------|-----|------|
| `0x120` | `1` | `0x81` |

**Explanation:**  
`0x81 = 1000 0001b` → DRL ON (bit 0) + Horn ON (bit 7).

---

### Example 2 — Front: Report Current for Channel 3 (Left Blinker)
| CAN ID | DLC | Data |
|--------|-----|------|
| `0x143` | `2` | `0x2C` `0x01` |

**Explanation:**  
`0x012C = 300 mA` on channel 3.

---

### Example 3 — Rear: Proximity Telemetry
| CAN ID | DLC | Data |
|--------|-----|------|
| `0x250` | `3` | `0x32` `0xFF` `0x00` |

**Explanation:**  
- Left sensor = 50 cm  
- Center = no object (255)  
- Right = error/dirty (0)

---

### Example 4 — Front: Ambient Light = ~50,000 lux
| CAN ID | DLC | Data |
|--------|-----|------|
| `0x151` | `2` | `0xD2` `0x04` |

**Explanation:**  
- Encoded value = `0x04D2`  
- If `E = 1` and `M = 0x0D2 = 210` → `lux = 210 × 2^1 × 0.1 = 42 lux`  
- Adjust E/M for exact lux target (example shows decoding form).  

---



# Input event
current bellow threshold low for light 0-7
current above threshold high for light 0-7
proximity distance detect something
luminosity above threshold X
luminosity bellow threshold Y

# State machines
general rules about timer, if set while running, is start again at the set time but will not generate more than ONE timeout event.
we should have an event queue and a STATE table. the state table is for global state that must be present or absent for something to happen with an event. the event are for every state change.

# STATE MACHINES (FSM DEFINITIONS)

## GLOBAL FSM RULES

* FSMs consume events from a **global event queue**
* Each FSM has **its own timer(s)**

  * If a timer is started while already running, it **restarts**
  * A timer **emits only ONE timeout event**
* `ST_` = current state
* `EV_` = discrete event
* Transitions are **edge-driven only**
* Outputs are applied in the **output stage**, never in input logic

---

## blinker FSM

### States

```
ST_BLINKER_OFF
ST_BLINKER_WARNING_ON
ST_BLINKER_WARNING_OFF
ST_BLINKER_LEFT_ON
ST_BLINKER_LEFT_OFF
```

### Transitions

```

```

### Outputs

```

```

---

