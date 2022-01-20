#include <Dramco-UNO.h>

LoraParam DevEUI = "70B3D57ED004B3EC";
LoraParam AppKey = "CB61E2552ABB2DF5D0F18803B694BDEF";

void setup() {
    DramcoUno.begin(DevEUI, AppKey);
}

void loop() {
    DramcoUno.blink();
    DramcoUno.addTemperature();
    DramcoUno.send();
    DramcoUno.delayUntilButtonPress();
}