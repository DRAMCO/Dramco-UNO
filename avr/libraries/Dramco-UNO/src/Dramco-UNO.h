#ifndef __Dramco_UNO
#define __Dramco_UNO


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define DRAMCO_UNO_LMIC_NSS_PIN 6
#define DRAMCO_UNO_LMIC_RST_PIN 5
#define DRAMCO_UNO_LMIC_DIO1_PIN 2
#define DRAMCO_UNO_LMIC_DIO2_PIN 3
#define DRAMCO_UNO_LMIC_DIO3_PIN 4

#define DRAMCO_UNO_LORA_ENABLE_PIN 8

#define LORA_EUI_SIZE  8
#define LORA_KEY_SIZE  16

typedef const char * LoraParam;

class DramcoUno {
	public:
		void begin(LoraParam deveui, LoraParam appeui, LoraParam appkey);
		void send(char * buffer);
		void loop();
//	private:
};

#endif//__Dramco_UNO
