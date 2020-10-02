#include <Dramco-UNO.h>

LoraParam Device_EUI = "001449031C31FF8D";
LoraParam Application_EUI = "70B3D57ED000EFD3";
LoraParam App_Key = "1697F5AFE2242FFBA53E8383ED0ABB2D";

DramcoUno board;

void setup() {
    board.begin(Device_EUI, Application_EUI, App_Key);
}

void loop() {
    board.blink();
    board.addTemperature();
    board.addLuminosity();
    board.send();
    board.delay(60000);
}