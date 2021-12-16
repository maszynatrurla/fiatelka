# Fiatełka - christmas lights controller

Program for ATTINY microcontroller to switch on/off LED light chain automatically.

Lights are switched on/off based on photo-sensor and timer.


## Introduction

At the beginning of holiday season, supermarkets stock large amouts of light ornaments. I especially like warm-white "micro-LED" lights. They are battery-operated (taking 2 or 3 AA batteries) and have
from 20 to 100 small "tear-shaped" LEDs attached to thin wire. They are very cheap and are perfect to decorate small indoor items. 

LEDs are very efficient, but even so, batteries will run flat after a few days if you let them running. Timer-controlled LED lights are available, but I had bought "dumb" ones.

I wanted to create low cost (so I can upgrade several light chains for cheap), small footprint (so it fits in original battery compartment) and low power (save battery, not drain it) microcontroller
device to control lights. The idea was to use light detection to switch lights on when it gets dark. The length of time LEDs are kept lit to be limited by timer.

Eventually, I used the same controller in a beefed up version to control bigger LED chain powered from 5V wall adapter, not batteries. In that case energy saving was not much a concern, but
automatic on/off was still cool things to have.


## Hardware

MCU is ATTINY13A. 

```
                _____
            1 -|o    |- 8 vcc 
 (led)  pb3 2 -|     |- 
 (lux)  pb4 3 -|     |- 6 pb1 (button)
        gnd 4 -|_____|-       
                            
```
 
 * PB1 (INT0) is connected via pull-up resistor to Vcc and via button to ground.
 * PB3 is connected to switch turing on/off LED chain (in my case it goes to transistor gate through resistor).
 * PB4 goes to voltage divider, made to measure voltage on photoresistor.
 

## Usage

When powered on, it is checked if button is pressed. If so device enters configuration mode - more about that later. If button is not pressed at power-up, then device enters
"DAY" state. LEDs are off.

In the "DAY" state, voltage on photo resistor is checked every 2s (processor goes into deep sleep inbeetween). When it is "dark enough" state changes to "EVENING" and LEDs are turned on.
Fade-in effect is achieved by pulsing LED switch with gradually longer pulses. "EVENING" lasts some configurable amount of time. When time passes, device enters "NIGHT" state and LEDS
are turned off (with fade-out effect). "NIGHT" again consists of 2s sleep interrupted by photo sensor check. This time state advance requires it to be brighter than threshold value.
The next state is "DAY" and the cycle repeats.

Device will react to button presses that override state machine and will make LEDs go on or off regardless of light or timer value. In particular, clicking the button when lights are off
will make application jump into "EVENING" state. Clicking the button when LEDs are on will make application jump into "NIGHT" state. 

### Configuration mode

Configuration mode allows to calibrate photo sensor and change length of time LEDs will stay on.

When device starts in configuration mode, it will start blink LEDs to signal configuration setting that can be selected. Sequence of quick blinks is followed by longer, dark pause (2s).
During the pause user can click button to select configuration option.

 * 1 blink - exit configuration mode - return to normal operation;
 * 2 blinks - set light sensor threshold for "DAY"->"EVENING" transition ("dark" threshold);
 * 3 blinks - set light sensor threshold for "NIGHT"->"DAY" transition ("bright" threshold);
 * 4 blinks - show current setting of LED timer;
 * 5 blinks - set LED timer length;
 * 6 blinks - restore default settings.
 
Setting selection is acknowledged by another, identical sequence of blinks. What happens later depends on setting.

For calibration settings (2, 3), device will wait for another button press. The moment button is pressed, voltage measurement on photo resistor will be made. This measurement will be
stored in nonvolatile memory (EEPROM) as new threshold value.

The length of time LEDs will be kept on (5) is configured by clicking button a number of times. One click equals 15 minutes - so, for example 6 clicks is 1.5 h. At least one click must be made.
No clicks for 5 seconds mean the end of timer configuration. At that moment, device will acknowledge input by blinking LEDs the same number of times as number of button clicks it has
detected.

When setting is completed, application returns to configuration "menu".




## Build/Flash

Developed for gcc-avr. Compilation must be optimized. 

Microcontroller should be running from internal 128kHz oscillator (energy saving), because code assumes that it is so.
(ATTINY13 should have lower fuse byte set to 0x7B)

