/*******************************************************************************
 * Copytight (c) 2016 Maarten Westenberg based on work of
 * Thomas Telkamp and Matthijs Kooijman porting the LMIC stack to Arduino IDE
 * and Gerben den Hartog for his tiny stack implementation with the AES library
 * that we used in the LMIC stack.
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with sensor values read.
 * If no sensor is connected the payload is '{"Hello":"World"}', that
 * will be processed by The Things Network server.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1, 
 *  0.1% in g2). 
 *
 * Change DevAddr to a unique address for your node 
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h, default is:
 *   #define CFG_sx1272_radio 1
 * for SX1272 and RFM92, but change to:
 *   #define CFG_sx1276_radio 1
 * for SX1276 and RFM95.
 *
 *******************************************************************************/

#define WAIT_SECS 60


#if defined(__AVR__)
#include <avr/pgmspace.h>
#include <Arduino.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP.h>
#elif defined(__MKL26Z64__)
#include <Arduino.h>
#else
#error Unknown architecture in aes.cpp
#endif

#include "lmic.h"
#include "hal/hal.h"
#include <SPI.h>

#define OTAA 1

//---------------------------------------------------------
// Sensor declarations
//---------------------------------------------------------

// Frame Counter
int count=0;

// LoRaWAN Application identifier (AppEUI)
// Not used in this example
///////static const u1_t APPEUI[8] PROGMEM = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x00, 0xEF, 0xD3 };

// LoRaWAN DevEUI, unique device ID (LSBF)
// Not used in this example
///////static const u1_t DEVEUI[8] PROGMEM  = { 0x00, 0xFB, 0xB8, 0x99, 0x33, 0x6C, 0xF7, 0x99 };

// LoRaWAN NwkSKey, network session key 
// Use this key for The Things Network
unsigned char NwkSkey[16] = 		{ 0xEB, 0xD4, 0x3C, 0x3D, 0x5E, 0x32, 0xCD, 0xFF, 0x41, 0x65, 0xBB, 0x60, 0xCC, 0xD8, 0xD8, 0xE2 };

// LoRaWAN AppSKey, application session key
// Use this key to get your data decrypted by The Things Network
unsigned char AppSkey[16] =		{ 0xF2, 0xB0, 0xE4, 0xBF, 0x12, 0x8D, 0xAB, 0x8C, 0xD5, 0xB2, 0x58, 0x53, 0x3F, 0x9C, 0xB1, 0x2B };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace

#define msbf4_read(p)   (u4_t)((u4_t)(p)[0]<<24 | (u4_t)(p)[1]<<16 | (p)[2]<<8 | (p)[3])
unsigned char DevAddr[4] =  { 0x26, 0x01, 0x13, 0x43 };


// ----------------------------------------------------------------------------
// APPLICATION CALLBACKS
// ----------------------------------------------------------------------------


// provide application router ID (8 bytes, LSBF)
//void os_getArtEui (u1_t* buf) {
//    memcpy(buf, APPEUI, 8);
//}
//
//// provide device ID (8 bytes, LSBF)
//void os_getDevEui (u1_t* buf) {
//    memcpy(buf, DEVEUI, 8);
//}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, NwkSkey, 16);
}

int debug=1;
uint8_t mydata[64];
static osjob_t sendjob;

// Pin mapping
// These settings should be set to the GPIO pins of the device
// you want to run the LMIC stack on.
//
lmic_pinmap pins = {
  .nss = 6,			// Connected to pin D10
  .rxtx = 0, 			// For placeholder only, Do not connected on RFM92/RFM95
  .rst = 5,  			// Needed on RFM92/RFM95? (probably not)
  .dio = {2, 3, 4},		// Specify pin numbers for DIO0, 1, 2
						// connected to D4, D5, D7 
};

void onEvent (ev_t ev) {
    //debug_event(ev);

    switch(ev) {
      // scheduled data sent (optionally data received)
      // note: this includes the receive window!
      case EV_TXCOMPLETE:
          // use this event to keep track of actual transmissions
          Serial.print("EV_TXCOMPLETE, time: ");
          Serial.println(millis() / 1000);
          if(LMIC.dataLen) { // data received in rx slot after tx
              //debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              Serial.println("Data Received");
          }
          break;
       default:
          break;
    }
}

void do_send(osjob_t* j){
	  delay(1);													// XXX delay is added for Serial
      Serial.print("Time: ");
      Serial.println(millis() / 1000);
      // Show TX channel (channel numbers are local to LMIC)
      Serial.print("Send, txCnhl: ");
      Serial.println(LMIC.txChnl);
      Serial.print("Opmode check: ");
      // Check if there is not a current TX/RX job running
    if (LMIC.opmode & (1 << 7)) {
      Serial.println("OP_TXRXPEND, not sending");
    } 
	else {
	
	  //Serial.print("ready to send: ");
	  strcpy((char *) mydata,"{\"Hello\":\"World\"}"); 
	  LMIC_setTxData2(1, mydata, strlen((char *)mydata), 0);
    }
    // Schedule a timed job to run at the given timestamp (absolute system time)
    os_setTimedCallback(j, os_getTime()+sec2osticks(WAIT_SECS), do_send);
         
}

// ====================================================================
// SETUP
//
void setup() {
  Serial.begin(115200);

  // Enable LoRa Module 
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters. Instead of dynamically establishing a session 
  // by joining the network, precomputed session parameters are be provided.
  ////////LMIC_setSession (0x1, msbf4_read(DevAddr), (uint8_t*)NwkSkey, (uint8_t*)AppSkey);
  // Disable data rate adaptation
  LMIC_setAdrMode(0);
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // Disable beacon tracking
  LMIC_disableTracking ();
  // Stop listening for downstream data (periodical reception)
  LMIC_stopPingable();
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF11,14);

  LMIC_disableChannel(6);
  
  //
#if defined(__AVR__)
	Serial.println("AVR arch");
#elif defined(ARDUINO_ARCH_ESP8266)
	Serial.println("ESP arch");
#elif defined(__MKL26Z64__)
	Serial.println("Teensy arch");
#else
	Serial.println("WARNING. Unknown Arch");
#endif

}

// ================================================================
// LOOP
//
void loop() {

	do_send(&sendjob);
	while(1) {
		os_runloop_once(); 
		delay(100);
	}
}

