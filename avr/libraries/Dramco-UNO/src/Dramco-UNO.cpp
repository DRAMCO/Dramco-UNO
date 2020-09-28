#include <Dramco-UNO.h>

// ------------------------ LMIC STUFF ------------------------
const lmic_pinmap lmic_pins = {
  .nss = DRAMCO_UNO_LMIC_NSS_PIN,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = DRAMCO_UNO_LMIC_RST_PIN,
  .dio = {DRAMCO_UNO_LMIC_DIO1_PIN, DRAMCO_UNO_LMIC_DIO2_PIN, DRAMCO_UNO_LMIC_DIO3_PIN},
};

static u1_t _appeui[LORA_EUI_SIZE];
static u1_t _deveui[LORA_EUI_SIZE];
static u1_t _appkey[LORA_KEY_SIZE];

static osjob_t sendjob;
static uint8_t mydata[] = "Hello, world!";

const unsigned TX_INTERVAL = 60;

void os_getArtEui (u1_t* buf) { // LMIC expects reverse from TTN
  for(byte i = 8; i>0; i--){
    buf[8-i] = _appeui[i-1];
  }
}

void os_getDevEui (u1_t* buf) { // LMIC expects reverse from TTN
  for(byte i = 8; i>0; i--){
    buf[8-i] = _deveui[i-1];
  }
}

void os_getDevKey (u1_t* buf) {  // no reverse here
	memcpy(buf, _appkey, 16);
} 

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
	#ifdef DEBUG
    Serial.print(os_getTime());
    Serial.print(": ");
    #endif 
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            #ifdef DEBUG
        	Serial.println(F("EV_SCAN_TIMEOUT"));
        	#endif
            break;
        case EV_BEACON_FOUND:
        	#ifdef DEBUG
            Serial.println(F("EV_BEACON_FOUND"));
            #endif
            break;
        case EV_BEACON_MISSED:
        	#ifdef DEBUG
            Serial.println(F("EV_BEACON_MISSED"));
            #endif
            break;
        case EV_BEACON_TRACKED:
        	#ifdef DEBUG
            Serial.println(F("EV_BEACON_TRACKED"));
            #endif
            break;
        case EV_JOINING:
        	#ifdef DEBUG
            Serial.println(F("EV_JOINING"));
            #endif
            break;
        case EV_JOINED:
        	#ifdef DEBUG
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
              
            }
            #endif
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
        	#ifdef DEBUG
            Serial.println(F("EV_JOIN_FAILED"));
            #endif
            break;
        case EV_REJOIN_FAILED:
        	#ifdef DEBUG
            Serial.println(F("EV_REJOIN_FAILED"));
            #endif
            break;
        case EV_TXCOMPLETE:
        	#ifdef DEBUG
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            #endif
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
       		#ifdef DEBUG
            Serial.println(F("EV_LOST_TSYNC"));
            #endif
            break;
        case EV_RESET:
        	#ifdef DEBUG
            Serial.println(F("EV_RESET"));
            #endif
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
       		#ifdef DEBUG
            Serial.println(F("EV_RXCOMPLETE"));
            #endif
            break;
        case EV_LINK_DEAD:
        	#ifdef DEBUG
            Serial.println(F("EV_LINK_DEAD"));
            #endif
            break;
        case EV_LINK_ALIVE:
        	#ifdef DEBUG
            Serial.println(F("EV_LINK_ALIVE"));
            #endif
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
        	#ifdef DEBUG
            Serial.println(F("EV_TXSTART"));
            #endif
            break;
        case EV_TXCANCELED:
        	#ifdef DEBUG
            Serial.println(F("EV_TXCANCELED"));
            #endif
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
        	#ifdef DEBUG
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            #endif
            break;

        default:
        	#ifdef DEBUG
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            #endif
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

// ------------------------ DRAMCO UNO LIB ------------------------
void DramcoUno::begin(LoraParam deveui, LoraParam appeui, LoraParam appkey){
	Serial.println("hi");
	// copy and convert string (aka char *) to byte array
	char tempStr[3] = {0x00, 0x00, 0x00};
	// -> deveui
	for(uint8_t i=0; i<LORA_EUI_SIZE; i++){
		tempStr[0] = *(deveui+(i*2));
		tempStr[1] = *(deveui+(i*2)+1);
		*(_deveui+i) = (u1_t)strtol(tempStr, NULL, 16);
	}
	// -> appeui
	for(uint8_t i=0; i<LORA_EUI_SIZE; i++){
		tempStr[0] = *(appeui+(i*2));
		tempStr[1] = *(appeui+(i*2)+1);
		*(_appeui+i) = (u1_t)strtol(tempStr, NULL, 16);
	}
	// -> appkey
	for(uint8_t i=0; i<LORA_KEY_SIZE; i++){
		tempStr[0] = *(appkey+(i*2));
		tempStr[1] = *(appkey+(i*2)+1);
		*(_appkey+i) = (u1_t)strtol(tempStr, NULL, 16);
	}

	pinMode(DRAMCO_UNO_LORA_ENABLE_PIN, OUTPUT);
    digitalWrite(DRAMCO_UNO_LORA_ENABLE_PIN, HIGH);

    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
	LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
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

	// Disable link check validation
	LMIC_setLinkCheckMode(0);

	// required for downlink
	LMIC.dn2Dr = SF9;

	// Set data rate and transmit power (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(DR_SF7, 14);
}

void DramcoUno::send(char * buffer){
	do_send(&sendjob);
}

void DramcoUno::loop(){
	os_runloop_once();
}

// TODO: Sleep, accelerometer, light
