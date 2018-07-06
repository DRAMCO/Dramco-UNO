//Accelerometer
#include <Wire.h>
//Temperature sensors
#include <OneWire.h> 
#include <DallasTemperature.h>
//LoRa Module
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//Sleep
#include <DeepSleepScheduler.h>

//Accelerometer parameters
#define ACC_ID          0x53  //ADXL345 Device ID
#define ACC_POWER_CTL   0x2D  //Power Control Register
#define ACC_DATA_FORMAT 0x31
#define ACC_START_BYTE  0x32  //X-Axis Data 0
#define ACC_BYTES       0x06  //Number of databytes
#define ACC_GAIN        0.00376390  //Convertion factor to g

byte acc_buffer[ACC_BYTES];

//Temperature sensor parameters
#define TEMP_BUS  7     //Onewire pin
#define TEMP_SENSORS 3  //Number of temperature sensors 

OneWire oneWire(TEMP_BUS);                  //Setup onwire on selected pin
DallasTemperature temp_sensors(&oneWire);   //Setup temperature sensor 

float temp_buffer[TEMP_SENSORS]; 

//LoRa module parameters
#define DEVICE 5

#define POWER_ENABLE_PIN 8
#define MEASURE_INTERVAL 900000   //Measurement interval time in ms  each quarter

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
#elif DEVICE == 5  //Boom Bart
  // LoRaWAN NwkSKey, network session key
  static const PROGMEM u1_t NWKSKEY[16] = { 0xF5, 0xA3, 0x98, 0xCA, 0x0B, 0xC5, 0x56, 0x51, 0x87, 0xC0, 0xE2, 0x4E, 0xC0, 0xEB, 0x4C, 0xCD };
  // LoRaWAN AppSKey, application session key
  static const u1_t PROGMEM APPSKEY[16] = { 0x6F, 0x28, 0x82, 0x08, 0x0E, 0x1D, 0x4D, 0xF8, 0x0A, 0x5B, 0xDE, 0x3B, 0xCF, 0xA6, 0x38, 0x2A };
  // LoRaWAN end-device address (DevAddr)
  static const u4_t DEVADDR = 0x260110C5 ;
#endif
 

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

#define LORA_TEMP 0x67
#define LORA_ACC  0x71
static uint8_t lora_stream[8 + TEMP_SENSORS * 4];
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


void setup()
{
  //Enable Serial Port
  Serial.begin(115200); 
  
  //Enable Pin 3V3 LDO
  pinMode(POWER_ENABLE_PIN, OUTPUT);
  
  //Start measurement scheduler
  scheduler.schedule(measure);
}
void measure() {
  uint8_t i;
  //Enable 3V3 LDO
  digitalWrite(POWER_ENABLE_PIN, HIGH);

  //Start up Accelerometer
  Wire.begin();        // join i2c bus (address optional for master)
  
   //Put the ADXL345 into +/- 2G range by writing the value 0x01 to the DATA_FORMAT register.
  writeTo(ACC_DATA_FORMAT, 0x00);
   //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeTo(ACC_POWER_CTL, 0x08);

  //Start up temperature sensors and read number of sensors
  temp_sensors.begin(); 
  Serial.println(temp_sensors.getDeviceCount());
  temp_sensors.setResolution(12); 
  temp_sensors.requestTemperatures(); // Send the command to get temperature readings 
  for(i = 0; i < TEMP_SENSORS; i++){
    temp_buffer[i] = temp_sensors.getTempCByIndex(i);

    lora_stream[0 + 4*i] = i;
    lora_stream[1 + 4*i] = LORA_TEMP;
    lora_stream[2 + 4*i] = (uint8_t)((int16_t)(temp_buffer[i]*10) >> 8);
    lora_stream[3 + 4*i] = (uint8_t)((int16_t)(temp_buffer[i]*10) & 0xFF);
  
    Serial.print("Temp ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(temp_buffer[i]);  
  }
  Serial.println();
  
  //Read accellerometer data
  readFrom(ACC_START_BYTE, ACC_BYTES, acc_buffer); //read the acceleration data from the ADXL345
    // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  int acc_x = (int)(((((int)acc_buffer[1]) << 8) | acc_buffer[0]) * 3.76390);   
  int acc_y = (int)(((((int)acc_buffer[3]) << 8) | acc_buffer[2]) * 3.76390);
  int acc_z = (int)(((((int)acc_buffer[5]) << 8) | acc_buffer[4]) * 3.76390);

  lora_stream[4*TEMP_SENSORS + 0] = 3;
  lora_stream[4*TEMP_SENSORS + 1] = LORA_ACC;
  lora_stream[4*TEMP_SENSORS + 2] = acc_x >> 8;
  lora_stream[4*TEMP_SENSORS + 3] = acc_x & 0xFF;
  lora_stream[4*TEMP_SENSORS + 4] = acc_y >> 8;
  lora_stream[4*TEMP_SENSORS + 5] = acc_y & 0xFF;
  lora_stream[4*TEMP_SENSORS + 6] = acc_z >> 8;
  lora_stream[4*TEMP_SENSORS + 7] = acc_z & 0xFF;

  Serial.print("x: ");
  Serial.print(acc_x);
  Serial.print(" y: ");
  Serial.print(acc_y);
  Serial.print(" z: ");
  Serial.println(acc_z);
  
  //Start Lora

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

  
  os_runloop_once();



  delay(1000); // only read every 0,5 seconds
  
  //Sleep
  digitalWrite(POWER_ENABLE_PIN, LOW);
  Serial.print("Sleep");
  //Serial.flush();
  scheduler.scheduleDelayed(measure, MEASURE_INTERVAL);
}

void loop()
{
  scheduler.execute();
}


//LoRa Module Functions
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
        LMIC_setTxData2(1, lora_stream, sizeof(lora_stream), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}
//Accelero Functions
void writeTo(byte address, byte val){
  Wire.beginTransmission(ACC_ID); // start transmission to device 
  Wire.write(address);            // send register address
  Wire.write(val);                // send value to write
  Wire.endTransmission();         // end transmission
}

void readFrom(byte address, int num, byte _buff[]) {
  int i = 0;
  
  Wire.beginTransmission(ACC_ID); // start transmission to device 
  Wire.write(address);            // sends address to read from
  Wire.endTransmission();         // end transmission

  Wire.beginTransmission(ACC_ID); // start transmission to device
  Wire.requestFrom(ACC_ID, num);  // request 6 bytes from device

  while(Wire.available()){        // device may send less than requested (abnormal)
    acc_buffer[i] = Wire.read();  // receive a byte
    i++;
  }
  Wire.endTransmission();         // end transmission
}
