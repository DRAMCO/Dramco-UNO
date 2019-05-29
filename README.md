# Dramco UNO

LoRaWAN-enabled Arduino board developed by [DRAMCO](www.dramco.be) featuring an accelerometer and a onewire connection optimized for low-power.

## Folder Structure

```
├───app
│   ├───Accelero-Test
│   ├───Dramco-UNO
│   └───Temprature-Sensor-Test
├───hardware
└───libraries
    └───arduino-lmic-master
```

- Example applications can be found in `app`
- Hardware folder includes the schematics 
- Altered libraries can be found in the `libraries`folder

## Description

- The device sends temperature, accelerometer and the battery level each MEASURE_INTERVAL.
- The (onewire) temperature sensor needs to be connected to the TEMP_BUS pin
- The (I2C) accelerometer with id ACC_ID is accessed through the Wire.h library
- The battery level is measured through the readVcc() function
- The measurements sent by the node are formatted with the Cayenne Low Power Payload (https://github.com/myDevicesIoT/cayenne-docs/blob/master/docs/LORA.md)

##  Dependencies:
- OneWire lib
- DallasTemperature lib
- Install arduino-lmic-master in lib folder of the Arduino IDE (for windows this is most likely here: C:\Program Files (x86)\Arduino\libraries)
    * If you do not use the lib provided in this repo, be sure to uncomment '#define DISABLE_JOIN' in src>lmic>config.h
- DeepSleepScheduler lib

- LoRaWAN keys see: https://www.thethingsnetwork.org/docs/devices/registration.html


## How to cite
```LaTex
@Article{s19030585,
AUTHOR = {Thoen, Bart and Callebaut, Gilles and Leenders, Guus and Wielandt, Stijn},
TITLE = {{A Deployable LPWAN Platform for Low-Cost and Energy-Constrained IoT Applications}},
JOURNAL = {Sensors},
VOLUME = {19},
YEAR = {2019},
NUMBER = {3},
ARTICLE-NUMBER = {585},
URL = {https://www.mdpi.com/1424-8220/19/3/585},
ISSN = {1424-8220},
DOI = {10.3390/s19030585}
}
```
