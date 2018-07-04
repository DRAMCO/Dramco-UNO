/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/
//LoRa Moduele
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//Temperature sensors
#include <OneWire.h> 
#include <DallasTemperature.h>

// Data wire is plugged into pin 2 on the Arduino 
#define ONE_WIRE_BUS 7 

/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


#define DEVICE 4

#if DEVICE == 1
  // LoRaWAN NwkSKey, network session key
  static const PROGMEM u1_t NWKSKEY[16] = { 0xEB, 0xD4, 0x3C, 0x3D, 0x5E, 0x32, 0xCD, 0xFF, 0x41, 0x65, 0xBB, 0x60, 0xCC, 0xD8, 0xD8, 0xE2 };
  // LoRaWAN AppSKey, application session key
  static const u1_t PROGMEM APPSKEY[16] = { 0xF2, 0xB0, 0xE4, 0xBF, 0x12, 0x8D, 0xAB, 0x8C, 0xD5, 0xB2, 0x58, 0x53, 0x3F, 0x9C, 0xB1, 0x2B };
  // LoRaWAN end-device address (DevAddr)
  static const u4_t DEVADDR = 0x26011343 ;
#elif DEVICE == 2
  // LoRaWAN NwkSKey, network session key
  static const PROGMEM u1_t NWKSKEY[16] = { 0x1B, 0xC1, 0xDF, 0xBF, 0xF3, 0x8A, 0x71, 0x50, 0x5C, 0x2D, 0xE4, 0x9A, 0xF0, 0x30, 0xBD, 0x23 };
  // LoRaWAN AppSKey, application session key
  static const u1_t PROGMEM APPSKEY[16] = { 0xE5, 0x7F, 0xF1, 0x89, 0x8C, 0x95, 0xEC, 0x42, 0x72, 0x73, 0x0C, 0x54, 0x7A, 0xA0, 0x54, 0x40 };
  // LoRaWAN end-device address (DevAddr)
  static const u4_t DEVADDR = 0x260119E7 ;
#elif DEVICE == 3
  // LoRaWAN NwkSKey, network session key
  static const PROGMEM u1_t NWKSKEY[16] = { 0x5C, 0xE3, 0x62, 0x82, 0x6B, 0xC4, 0xE2, 0x6B, 0xA9, 0xE8, 0x5C, 0xA8, 0x4C, 0xA3, 0x66, 0x68 };
  // LoRaWAN AppSKey, application session key
  static const u1_t PROGMEM APPSKEY[16] = { 0xFC, 0xC0, 0x35, 0x91, 0x5B, 0x10, 0xE9, 0xB8, 0x81, 0xC9, 0x5A, 0x79, 0xA2, 0xAC, 0x43, 0xB8 };
  // LoRaWAN end-device address (DevAddr)
  static const u4_t DEVADDR = 0x26011638 ;
#elif DEVICE == 4
  // LoRaWAN NwkSKey, network session key
  static const PROGMEM u1_t NWKSKEY[16] = { 0xEA, 0x9C, 0x58, 0x94, 0xD6, 0x01, 0x68, 0xE2, 0x97, 0x87, 0x51, 0xFE, 0x6C, 0x64, 0x4B, 0xDB };
  // LoRaWAN AppSKey, application session key
  static const u1_t PROGMEM APPSKEY[16] = { 0x59, 0xE9, 0x64, 0x97, 0x1C, 0x4E, 0x23, 0x9C, 0x8B, 0xB9, 0xF5, 0xFF, 0xB6, 0x57, 0x74, 0xCE };
  // LoRaWAN end-device address (DevAddr)
  static const u4_t DEVADDR = 0x26011A6D ;
#endif
 

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "OK";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            Serial.println(os_getTime()+sec2osticks(TX_INTERVAL));
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    pinMode(8, OUTPUT);
    digitalWrite(8, HIGH);
    
    sensors.begin(); 
    
    delay(1000);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF11,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
     Serial.print(" Requesting temperatures..."); 
     sensors.requestTemperatures(); // Send the command to get temperature readings 
     Serial.println("DONE"); 
    /********************************************************************/
     Serial.print("Temperature is: "); 
     Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"?  
}
