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

there is a static flag I can set at compilation that tell if the board is the rear or front one. the code is almost the same, it just change the CAN id when broadcasting and the PIN mapping assignement

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

GPA0 : EN_PIEZO_A_X
GPA1 : EN_PIEZO_B_X
GPA2 : EN_PIEZO_C_X

GPA3 : EN_PIEZO_A_Y
GPA4 : EN_PIEZO_B_Y
GPA5 : EN_PIEZO_C_Y

# Piezo Sonar Driving

Each sonar channel (A, B, C) uses a piezoelectric transducer driven by two half-H bridges (X side and Y side).
GPIO 9 (PIN_PIEZO_PWM_HALF_X) and GPIO 10 (PIN_PIEZO_PWM_HALF_Y) provide the 40 kHz PWM excitation signals.
The MCP23017 GPA pins (EN_PIEZO_x_X, EN_PIEZO_x_Y) are enable lines for each half-H bridge:
- EN = HIGH: half-H bridge active (drives the piezo)
- EN = LOW: half-H bridge in High-Z (disconnected)

To transmit on channel A: set EN_PIEZO_A_X=HIGH, EN_PIEZO_A_Y=HIGH, drive complementary 40 kHz on PWM_HALF_X and PWM_HALF_Y.
To listen on channel A: set EN_PIEZO_A_X=LOW (High-Z one side), EN_PIEZO_A_Y=LOW, then read the analog echo on PIN_SONAR_ECHO_A.
The echo signal is filtered with a Goertzel algorithm (single-frequency DFT at 40 kHz) to detect the return pulse magnitude.
Distance is computed from the time-of-flight between excitation start and the echo threshold crossing.

**NOTE:** There is currently a hardware bug — the X and Y enable select lines are not independently routable
per channel, so sonar excitation is disabled in firmware. The code is implemented but commented out.
# CAN codes


This catalog defines **standard 11-bit CAN 2.0 IDs (0x000–0x7FF)** for the front (`0x17`) and rear (`0x16`) lightboards.  
Device addressing is handled by **distinct CAN IDs per board**, so no extra address byte is needed in payloads.

**All data is little-endian. DLC is the exact length used.**

---

# Front Lightboard (`0x17`) — CAN IDs

## Input (Control/Set)

| `0x121` | 5 | Set current thresholds for alert | **Byte 0:** Channel (0–7)<br>**Bytes 1–2:** Low threshold mA (0 = disable)<br>**Bytes 3–4:** High threshold mA (0 = disable) |

ID 0x400, 2 byte :   
0010 0001 0000 0000 -> rien
0010 0010 0000 0000 -> low beam
0010 0100 0000 0000 -> full beam
0010 0011 0000 0000 -> appelle de phare
1010 0001 0000 0000 -> horn
1010 1001 0000 0000 -> left blinker
1011 0001 0000 0000 -> right blinker

ID 0x405 2 byte :
1000 0000 4F -> button A
0100 0000 4F -> button B

ID 0x407, 1 byte -> analog value for the throttle





for the control of the light, we read packet with ID 0x400 made of two byte with the following structure :
| Bit | Output | Mosfet |
|-----|--------|--------|
| 0 | horn | MOSFET_GATE_4 (frontboard only) |
| 1 | brake | MOSFET_GATE_1 (rearboard only) |
| 2 | tail | MOSFET_GATE_0 (rearboard only) |
| 3 | right blinker | MOSFET_GATE_3  |
| 4 | left blinker | MOSFET_GATE_2  |
| 5 | high beam |MOSFET_GATE_1 READ NOTE (frontboard only) |
| 6 | Low Beam |MOSFET_GATE_1 READ NOTE (frontboard only) |
| 7 | Day (DRL) | MOSFET_GATE_0 (frontboard only) |
| 8 | flash |  MOSFET_GATE_7 (rearboard only) |
| 9 | fog |  MOSFET_GATE_6 (rearboard only) |
| 10 | reverse |  MOSFET_GATE_5 (rearboard only) |

we should reflect the bit state to the mosfet. if no command is received for more than 3 second, we turn off the output.


for the low beam and high beam, we have a "smart" light where rising EDGE toggle the state. the state machine is as follow :

(after BOOT) -> DRL only -(rising edge)> low beam -(rising edge)> high beam
high beam -(rising edge)> low beam

We need to implement it too and keep track of the state so that when low beam bit is active we switch to low beam, and when high beam bit is active we switch to high beam. if both are active, we stay / go to high beam. 

# Ouput CAN


0x481 / 0x491 should be 2 byte that 
| rearboard         | `0x481–0x490` |
| frontboard        | `0x491–0x500` |
## Output (Status/Telemetry)

| CAN ID | DLC | Purpose | Payload |
|--------|-----|---------|---------|
| `0x481/0x491` | 1 | mosfet output status | **Byte 0:** 8-bit status (same bit mapping as control) |
| `0x482/0x492` | 5 | Report current thresholds | Same payload layout as `0x121` |
| `0x483/0x493` | 2 | Current per channel | **Bytes 0–1:** INT16 current in mA for channel 0–7 (CAN ID offset = base + channel) |
| `0x484/0x494` | 3 | proximity | **Byte 0:** Left sensor cm<br>**Byte 1:** Center sensor cm<br>**Byte 2:** Right sensor cm |
| `0x485/0x495` | 2 | Ambient light | **Bytes 0–1:** Fixed-exponent lux (see encoding below) |


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

