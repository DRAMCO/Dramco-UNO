#ifndef __Dramco_UNO
#define __Dramco_UNO


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <avr/sleep.h>
#include <avr/wdt.h>

#include "DeepSleep_avr_definition.h"

#define DRAMCO_UNO_LED_NAME A3
#define DRAMCO_UNO_LED_PORT PORTC
#define DRAMCO_UNO_LED_PIN 3

#define DRAMCO_UNO_BLINK_ON 100 // Time on in ms

#define DRAMCO_UNO_3V3_ENABLE_PIN 8

// LoRaWAN LMIC constants
#define DRAMCO_UNO_LMIC_NSS_PIN 6
#define DRAMCO_UNO_LMIC_RST_PIN 5
#define DRAMCO_UNO_LMIC_DIO1_PIN 2
#define DRAMCO_UNO_LMIC_DIO2_PIN 3
#define DRAMCO_UNO_LMIC_DIO3_PIN 4

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

#define DEBUG

typedef const char * LoraParam;

class DramcoUno {
	public:
		void begin(LoraParam deveui, LoraParam appeui, LoraParam appkey);
		void send();					// Made blocking
		void sendWithOS();				// Only with OS loop
		void loop();
		void delay(uint32_t d);

		void startBlink(); 				// Only with OS loop
		void startBlink(uint32_t d);	// Only with OS loop
		void stopBlink();				// Only with OS loop
		void blink();

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

		void sleep(uint32_t d);

		static void _isrWdt(); 
		static void _sleep(unsigned long maxWaitTimeMillis);
		static unsigned long _wdtEnableForSleep(const unsigned long maxWaitTimeMillis);	
		static void _wdtEnableInterrupt();
	private:
		void _lppAddToBuffer(float val, uint8_t channel, uint8_t type, uint8_t size, uint16_t mult);

		



};

#endif//__Dramco_UNO
