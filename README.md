# Dramco-UNO

LoRaWAN-enabled Arduino board developed by [DRAMCO](www.dramco.be) featuring an accelerometer and a onewire connection.

---
ONLY ABP IS CURRENTLY SUPPORTED!
---

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
