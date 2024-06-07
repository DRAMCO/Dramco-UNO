#include <Arduino.h>
#include <Dramco-UNO.h>

LoraParam DevEUI = "";
LoraParam AppKey = "";


void setup() {
    DramcoUno.begin(DevEUI, AppKey);
}

void loop() {
    DramcoUno.blink();
    DramcoUno.delayUntilButtonPress(1800000); // 30 minutes: 1800000
    if(DramcoUno.getWakeReason() == DRAMCO_UNO_WAKE_REASON_TIMER || (DramcoUno.getWakeReason() == DRAMCO_UNO_WAKE_REASON_BUTTON && DramcoUno.getButtonCounter() > 0 && DramcoUno.getButtonCounter() % 10 == 0)){
      DramcoUno.addButtonCounter();
      DramcoUno.send();
    }
}