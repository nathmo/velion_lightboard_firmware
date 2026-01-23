# velion_lightboard_firmware
This is the firmware that control the main board and control pannel of the vehicle.

# General logic
Use ESP32-C3 FreeRTOS on Arduino  IDE

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

I also have two ADC1115IDGS luminosity sensor on the I2C bus with address 1001000 -> 0x48 and 1001001 -> 0x49.
on the 0x48, AIN0 = SENSE_3, AIN1 = SENSE_2, AIN2 = SENSE_1,AIN3 = SENSE_0 

on the 0x49, AIN0 = SENSE_7, AIN1 = SENSE_6, AIN2 = SENSE_5,AIN3 = SENSE_4

for both ADC, they are mesuring the current trough a 1 milli Ohm resistor with the voltage difference accross it to GND.

I also have a VEML7700 luminosity sensor on the I2C bus


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

