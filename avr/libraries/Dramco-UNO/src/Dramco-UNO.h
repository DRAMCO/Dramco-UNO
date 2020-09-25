//#ifndef __Dramco_UNO
//#define __Dramco_UNO

//#define ARDUINO_LMIC_PROJECT_CONFIG_H ../../../Dramco-UNO/src/lmic_project_config.h

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define LORA_PARAMETER (static const u1_t PROGMEM)

#define DRAMCO_UNO_LMIC_NSS_PIN 6
#define DRAMCO_UNO_LMIC_RST_PIN 5
#define DRAMCO_UNO_LMIC_DIO1_PIN 2
#define DRAMCO_UNO_LMIC_DIO2_PIN 3
#define DRAMCO_UNO_LMIC_DIO3_PIN 4

#define DRAMCO_UNO_LORA_ENABLE_PIN 8

class DramcoUno {
	public:
		void begin(const u1_t* deveui, const u1_t* appeui, const u1_t* appkey);
		void send(char * buffer);
		void loop();
//	private:
};

//#endif __Dramco_UNO