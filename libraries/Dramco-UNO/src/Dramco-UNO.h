#ifndef __Dramco_UNO
#define __Dramco_UNO


#include "LMIC/lmic.h"
#include "LMIC/hal/hal.h"
#include <SPI.h>

#include <avr/sleep.h>
#include <avr/wdt.h>

#include <Wire.h>
#include "Deepsleep/DeepSleep_avr_definition.h"

#include "LIS2DW12/LIS2DW12.h"

#define DRAMCO_UNO_LED_NAME 4
#define DRAMCO_UNO_LED_PORT PORTD
#define DRAMCO_UNO_LED_PIN 4

#define DRAMCO_UNO_BLINK_ON 100 // Time on in ms

#define DRAMCO_UNO_3V3_ENABLE_PIN 8


#define DRAMCO_UNO_ERROR_LORA_JOIN 3
#define DRAMCO_UNO_ERROR_BUFFER 4

// LoRaWAN LMIC constants
#define DRAMCO_UNO_LMIC_NSS_PIN 6
#define DRAMCO_UNO_LMIC_RST_PIN LMIC_UNUSED_PIN
#define DRAMCO_UNO_LMIC_DIO0_PIN 2
#define DRAMCO_UNO_LMIC_DIO1_PIN 3
#define DRAMCO_UNO_LMIC_DIO2_PIN LMIC_UNUSED_PIN

#define DRAMCO_UNO_LORA_EUI_SIZE  8
#define DRAMCO_UNO_LORA_KEY_SIZE  16

#define DRAMCO_UNO_BUFFER_SIZE 20 // Should be enough for temp, lumin and accelerometer

// Sensor constants
#define DRAMCO_UNO_LIGHT_SENSOR_PIN A0
#define DRAMCO_UNO_TEMPERATURE_SENSOR_ENABLE_PIN 7
#define DRAMCO_UNO_TEMPERATURE_SENSOR_PIN A1
#define DRAMCO_UNO_TEMPERATURE_AVERAGE 50
#define DRAMCO_UNO_TEMPERATURE_CALIBRATE 3.27

#define DRAMCO_UNO_ACCELEROMTER_INT_PIN 9
#define DRAMCO_UNO_ACCELEROMTER_INT_NAME PINB1
#define DRAMCO_UNO_ACCELEROMTER_INT_PORT PINB

#define DRAMCO_UNO_BUTTON_PIN 10

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

#define DRAMCO_UNO_ACCLEROMETER_ACTION_WAKE		   1
#define DRAMCO_UNO_ACCLEROMETER_ACTION_SEND		   2
typedef const char * LoraParam;

class DramcoUno {
	public:
		void begin(LoraParam deveui, LoraParam appeui, LoraParam appkey);
		
		// --- Utils ---
		void loop();
		void delay(uint32_t d);
		static void blink();
		static void error(uint8_t errorcode);

		// --- Message ---
		static void send();					// Made blocking
		void sendWithOS();				// Only with OS loop
		void clearMessage();

		// --- Sensors ---
		// - Temperature
		void calibrateTemperature();
		float readTemperature();		// Gets temperature in degrees C
		void sendTemperature();			
		void addTemperature();
		void addTemperature(float temperature);
		void addTemperatureToMessage();
		void addTemperatureToMessage(float temperature);
		
		// - Luminosity
		float readLuminosity();			// Gets luminosity in % (not calibrated in lux)
		void sendLuminosity();
		void addLuminosity();
		void addLuminosity(float temperature);
		void addLuminosityToMessage();
		void addLuminosityToMessage(float temperature);

		// - Accelerometer
		float readAccelerationX();		// Gets motion in g
		float readAccelerationY();
		float readAccelerationZ();
		float readTemperatureAccelerometer();	// Gets temperature in degrees C of accelerometer

		static void addAcceleration();
		void addAccelerationToMessage();
		static void sendAcceleration();

		void delayUntilShake();
		void delayUntilFall();
		void delayUntilFreeFall();
		void delayUntilMotion();
		void delayUntilMovement();

		void sendAccelerationOnShake();
		void sendAccelerationOnFall();
		void sendAccelerationOnFreeFall();
		void sendAccelerationOnMotion();
		void sendAccelerationOnMovement();
		
		// --- Sleep ---
		void sleep(uint32_t d);
		static void _isrWdt(); 
		static void _sleep(unsigned long maxWaitTimeMillis);
		static unsigned long _wdtEnableForSleep(const unsigned long maxWaitTimeMillis);	
		static void _wdtEnableInterrupt();
	private:
		static void _lppAddToBuffer(float val, uint8_t channel, uint8_t type, uint8_t size, uint16_t mult);
		static void _lppAddAcceleration(uint8_t channel, float x, float y, float z);
};

#endif//__Dramco_UNO
