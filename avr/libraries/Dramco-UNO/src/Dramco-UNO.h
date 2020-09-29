#ifndef __Dramco_UNO
#define __Dramco_UNO


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LoRaWAN LMIC constants
#define DRAMCO_UNO_LMIC_NSS_PIN 6
#define DRAMCO_UNO_LMIC_RST_PIN 5
#define DRAMCO_UNO_LMIC_DIO1_PIN 2
#define DRAMCO_UNO_LMIC_DIO2_PIN 3
#define DRAMCO_UNO_LMIC_DIO3_PIN 4

#define DRAMCO_UNO_LORA_ENABLE_PIN 8
#define DRAMCO_UNO_LORA_EUI_SIZE  8
#define DRAMCO_UNO_LORA_KEY_SIZE  16

#define DRAMCO_UNO_BUFFER_SIZE 20 // Should be enough for temp, lumin and accelerometer

// Sensor constants
#define DRAMCO_UNO_LIGHT_SENSOR_PIN A0
#define DRAMCO_UNO_TEMPERATURE_SENSOR_PIN A1
#define DRAMCO_UNO_TEMPERATURE_AVERAGE 50
#define DRAMCO_UNO_TEMPERATURE_CALIBRATE 3.27

// Low power payload constants
#define DRAMCO_UNO_LPP_DIGITAL_INPUT               0     // 1 byte
#define DRAMCO_UNO_LPP_ANALOG_INPUT                2     // 2 bytes, 0.01 signed
#define DRAMCO_UNO_LPP_GENERIC_SENSOR              100   // 4 bytes, unsigned
#define DRAMCO_UNO_LPP_LUMINOSITY                  101   // 2 bytes, 1 lux unsigned
#define DRAMCO_UNO_LPP_TEMPERATURE                 103   // 2 bytes, 0.1°C signed
#define DRAMCO_UNO_LPP_ACCELEROMETER               113   // 2 bytes per axis, 0.001G

#define DRAMCO_UNO_LPP_DIGITAL_INPUT_SIZE          1
#define DRAMCO_UNO_LPP_ANALOG_INPUT_SIZE           2
#define DRAMCO_UNO_LPP_GENERIC_SENSOR_SIZE         4
#define DRAMCO_UNO_LPP_LUMINOSITY_SIZE             2
#define DRAMCO_UNO_LPP_TEMPERATURE_SIZE            2
#define DRAMCO_UNO_LPP_ACCELEROMETER_SIZE          6

#define DRAMCO_UNO_LPP_DIGITAL_INPUT_MULT          1
#define DRAMCO_UNO_LPP_ANALOG_INPUT_MULT           100
#define DRAMCO_UNO_LPP_GENERIC_SENSOR_MULT         1
#define DRAMCO_UNO_LPP_LUMINOSITY_MULT             1
#define DRAMCO_UNO_LPP_TEMPERATURE_MULT            10
#define DRAMCO_UNO_LPP_ACCELEROMETER_MULT          1000


typedef const char * LoraParam;

class DramcoUno {
	public:
		void begin(LoraParam deveui, LoraParam appeui, LoraParam appkey);
		void send();
		void loop();

		float readTemperature();
		float readLuminosity();

		void sendTemperature();
		void addTemperature();
		void addTemperature(float temperature);
		void addTemperatureToMessage();
		void addTemperatureToMessage(float temperature);

		void sendLuminosity();
		void addLuminosity();
		void addLuminosity(float temperature);
		void addLuminosityToMessage();
		void addLuminosityToMessage(float temperature);
		
	private:
		void _lppAddToBuffer(float val, uint8_t channel, uint8_t type, uint8_t size, uint16_t mult);

};

#endif//__Dramco_UNO
