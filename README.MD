# Wacom Ripoff BLE edition based on nRF51822

This is the ported code of my project Wac0m Ripoff to the nRF51822 Bluetooth capable MCU.  
With Arduino Core from [sandeepmistry](https://github.com/sandeepmistry/arduino-nRF5)

Required Libraries
=

1. [sandeepmistry's](https://github.com/sandeepmistry/arduino-BLEPeripheral) BLE peripheral library for all of Bluetooth 
2. My BLEDigitizer.cpp/h library ported from his BLEMouse.cpp/h code. (already included in same folder with the sketch).

Soft Device version and Low Freq Clock
=

Be sure to choose the version "S130". About the clock source. I'm using nRF51822 04 module (WT51822-S4AT),  
I need to select the RC oscillator. other option is not working and synthesized is not recommended.