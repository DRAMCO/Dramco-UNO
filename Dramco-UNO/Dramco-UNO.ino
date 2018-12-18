//IMPORTANT NOTICE: set DISABLE_JOIN in config.h in the lmic library

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
//EEPROM to save temperature sensor addresses
#include <EEPROM.h>

//#define VERBOSE

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
#define TEMP_SENSORS 2  //Number of temperature sensors 

OneWire oneWire(TEMP_BUS);                  //Setup onwire on selected pin
DallasTemperature temp_sensors(&oneWire);   //Setup temperature sensor
DeviceAddress savedTempSensors[TEMP_SENSORS];
DeviceAddress clearValue = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

//LoRa module parameters
#define DEVICE 1

#define POWER_ENABLE_PIN 8
//#define MEASURE_INTERVAL 900000   //Measurement interval time in ms each quarter
#define MEASURE_INTERVAL 20000   //Measurement interval time in ms each quarter

#define LORA_LPP_TEMP       0x67
#define LORA_LPP_ACC        0x71
#define LORA_LPP_ANALOG_OUT 0x03
uint8_t temp_sensor_channel[3] = {2, 1, 0};

#define LORA_PACKET_SIZE (8 + TEMP_SENSORS * 4 + 4)

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
#endif

static uint8_t lora_stream[LORA_PACKET_SIZE];
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
// function to compare device addresses
uint8_t bulkCompareAddresses(DeviceAddress deviceAddress[TEMP_SENSORS], DeviceAddress deviceAddressComp)
{
  uint8_t result = 0;
  for (uint8_t i = 0; i < TEMP_SENSORS; i++) {

    if (compareAddresses(deviceAddress[i], deviceAddressComp)) {
      result |= 1 << i;
    }
  }
  return result;
}
void setup()
{
  DeviceAddress tempSensors[TEMP_SENSORS];
  uint8_t detectedTempSensors;
  uint8_t outdoorPos;
  uint8_t treePos;
  uint8_t temp;
  int i;

  //Enable Serial Port
#ifdef VERBOSE
  Serial.begin(115200);
#endif

  //Enable Pin 3V3 LDO
  pinMode(POWER_ENABLE_PIN, OUTPUT);
#ifdef VERBOSE
  Serial.println(F("Dramco-UNO"));

  //Start up temperature sensors
  Serial.println(F("Known temperature sensors"));
#endif
  for (i = 0; i < TEMP_SENSORS; i++) {
    readAddressFromEEPROM(savedTempSensors[i], i);
    printAddress(savedTempSensors[i]);
  }
  temp_sensors.begin();
  detectedTempSensors = temp_sensors.getDeviceCount();
#ifdef VERBOSE
  Serial.print(detectedTempSensors);
  Serial.println(F(" temperature sensor(s) detected"));
#endif
  for (i = 0; i < detectedTempSensors; i++) {
    temp_sensors.getAddress(tempSensors[i], i);
  }
#ifdef VERBOSE
  switch (detectedTempSensors) {
    case 0:
      Serial.println(F("Known temperature sensors cleared"));
      for (i = 0; i < TEMP_SENSORS; i++) {
        saveAddressToEEPROM(clearValue, i);
      }
      break;
    case 1:
      if (compareAddresses(savedTempSensors[0], clearValue)) {
        Serial.println(F("Outdoor sensor added to list"));
        saveAddressToEEPROM(tempSensors[0], 0);
      }
      else if (compareAddresses(savedTempSensors[0], tempSensors[0])) {
        Serial.println(F("Sensor found: Outdoor"));
      }
      else {
        Serial.println(F("Outdoor sensor mismatch -> disconnect sensors and start over"));
      }
      break;
    case 2:
      outdoorPos = bulkCompareAddresses(tempSensors, savedTempSensors[0]);
      if (outdoorPos > 0) {
        Serial.println(F("Sensor found: Outdoor"));
        if (compareAddresses(savedTempSensors[1], clearValue)) {
          Serial.println(F("Tree sensor added to list"));
          saveAddressToEEPROM(tempSensors[findMissingSensor(outdoorPos, 2)], 1);
        }
        else if (compareAddresses(tempSensors[findMissingSensor(outdoorPos, 2)], savedTempSensors[1])) {
          Serial.println(F("Sensor found: Tree"));
        }
        else {
          Serial.println(F("Tree sensor mismatch  -> disconnect sensors and start over"));
        }
      } else {
        Serial.println(F("Outdoor sensor not found  -> disconnect sensors and start over"));
      }
      break;
    case 3:
      outdoorPos = bulkCompareAddresses(tempSensors, savedTempSensors[0]);
      treePos = bulkCompareAddresses(tempSensors, savedTempSensors[1]);
      if (outdoorPos > 0 && treePos > 0) {
        Serial.println(F("Sensor found: Outdoor"));
        Serial.println(F("Sensor found: Tree"));
        if (compareAddresses(savedTempSensors[2], clearValue)) {
          Serial.println(F("Aux sensor added to list"));
          saveAddressToEEPROM(tempSensors[findMissingSensor(outdoorPos | treePos, 3)], 2);
        }
        else if (compareAddresses(tempSensors[findMissingSensor(outdoorPos | treePos, 3)], savedTempSensors[2])) {
          Serial.println(F("Sensor found: Aux"));
        }
        else {
          Serial.println(F("Aux sensor mismatch  -> disconnect sensors and start over"));
        }
      } else {
        Serial.println(F("Outdoor or tree sensor not found  -> disconnect sensors and start over"));
      }
      break;
    default:
      Serial.println("To many temperature sensors");
      break;
  }
#else
  switch (detectedTempSensors) {
    case 0:
      for (i = 0; i < TEMP_SENSORS; i++) {
        saveAddressToEEPROM(clearValue, i);
      }
      break;
    case 1:
      if (compareAddresses(savedTempSensors[0], clearValue)) {
        saveAddressToEEPROM(tempSensors[0], 0);
      }
      break;
    case 2:
      outdoorPos = bulkCompareAddresses(tempSensors, savedTempSensors[0]);
      if (outdoorPos > 0) {
        if (compareAddresses(savedTempSensors[1], clearValue)) {
          saveAddressToEEPROM(tempSensors[findMissingSensor(outdoorPos, 2)], 1);
        }
      }
      break;
    case 3:
      outdoorPos = bulkCompareAddresses(tempSensors, savedTempSensors[0]);
      treePos = bulkCompareAddresses(tempSensors, savedTempSensors[1]);
      if (outdoorPos > 0 && treePos > 0) {
        if (compareAddresses(savedTempSensors[2], clearValue)) {
          saveAddressToEEPROM(tempSensors[findMissingSensor(outdoorPos | treePos, 3)], 2);
        }
      }
      break;
  }
#endif
  for (i = 0; i < TEMP_SENSORS; i++) {
    readAddressFromEEPROM(savedTempSensors[i], i);
#ifdef VERBOSE
    printAddress(savedTempSensors[i]);
#endif
  }

  temp_sensors.setResolution(10);   //10 bit -> conversion time 187.5 ms

  //Start measurement scheduler
  scheduler.schedule(measure);
}

// function to find a missing sensor in the list
uint8_t findMissingSensor(uint8_t sensors, uint8_t numberOfsensors)
{
  uint8_t result = 0;
  for (uint8_t i = 0; i < numberOfsensors; i++) {
    if ((sensors & 1 << i) == 0) {
      return i;
    }
  }
  return 255;
}
// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}

// function to compare device addresses
bool compareAddresses(DeviceAddress deviceAddress1, DeviceAddress deviceAddress2)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress1[i] != deviceAddress2[i]) {
      return false;
    }
  }
  return true;
}

void saveAddressToEEPROM(DeviceAddress deviceAddress, uint8_t number)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    EEPROM.write(i + 8 * number, deviceAddress[i]);
  }
}
void readAddressFromEEPROM(uint8_t *deviceAddress, uint8_t number)
{
  uint8_t val;
  for (uint8_t i = 0; i < 8; i++)
  {
    val = EEPROM.read(i + 8 * number);
    deviceAddress[i] = val;
  }
}
void measure() {
  uint8_t detectedTempSensors;
  float temp_buffer[TEMP_SENSORS];
  //Enable 3V3 LDO
  digitalWrite(POWER_ENABLE_PIN, HIGH);

  //Start Lora

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  //LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  

  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x34, DEVADDR, nwkskey, appskey);

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

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  //  Disable ADR
  //LMIC_setAdrMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF11, 14);

  

  //Start up Accelerometer
  Wire.begin();        // join i2c bus (address optional for master)

  //Put the ADXL345 into +/- 2G range by writing the value 0x01 to the DATA_FORMAT register.
  writeTo(ACC_DATA_FORMAT, 0x00);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeTo(ACC_POWER_CTL, 0x08);

  temp_sensors.requestTemperatures(); // Send the command to get temperature readings
  for (uint8_t i = 0; i < TEMP_SENSORS; i++) {
    temp_buffer[i] = temp_sensors.getTempC(savedTempSensors[i]);

    lora_stream[0 + 4 * i] = temp_sensor_channel[i];
    lora_stream[1 + 4 * i] = LORA_LPP_TEMP;
    lora_stream[2 + 4 * i] = (uint8_t)((int16_t)(temp_buffer[i] * 10) >> 8);
    lora_stream[3 + 4 * i] = (uint8_t)((int16_t)(temp_buffer[i] * 10) & 0xFF);
#ifdef VERBOSE
    Serial.print("Temp ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(temp_buffer[i]);
#endif
  }
#ifdef VERBOSE
  Serial.println();
#endif

  //Read accellerometer data (Doesn't work without Temp sensor)
  readFrom(ACC_START_BYTE, ACC_BYTES, acc_buffer); //read the acceleration data from the ADXL345
  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  int acc_x = (int)(((((int)acc_buffer[1]) << 8) | acc_buffer[0]) * 3.76390);
  int acc_y = (int)(((((int)acc_buffer[3]) << 8) | acc_buffer[2]) * 3.76390);
  int acc_z = (int)(((((int)acc_buffer[5]) << 8) | acc_buffer[4]) * 3.76390);

  lora_stream[4 * TEMP_SENSORS + 0] = 3;
  lora_stream[4 * TEMP_SENSORS + 1] = LORA_LPP_ACC;
  lora_stream[4 * TEMP_SENSORS + 2] = acc_x >> 8;
  lora_stream[4 * TEMP_SENSORS + 3] = acc_x & 0xFF;
  lora_stream[4 * TEMP_SENSORS + 4] = acc_y >> 8;
  lora_stream[4 * TEMP_SENSORS + 5] = acc_y & 0xFF;
  lora_stream[4 * TEMP_SENSORS + 6] = acc_z >> 8;
  lora_stream[4 * TEMP_SENSORS + 7] = acc_z & 0xFF;
#ifdef VERBOSE
  Serial.print(F("x: "));
  Serial.print(acc_x);
  Serial.print(F(" y: "));
  Serial.print(acc_y);
  Serial.print(F(" z: "));
  Serial.println(acc_z);
#endif

  //Read Voltage
  long batteryVoltage = readVcc();
#ifdef VERBOSE
  Serial.print(F("Vbatt: "));
  Serial.println(batteryVoltage, DEC);
#endif
  lora_stream[4 * TEMP_SENSORS + 8] = 3;
  lora_stream[4 * TEMP_SENSORS + 9] = LORA_LPP_ANALOG_OUT;
  lora_stream[4 * TEMP_SENSORS + 10] = (uint8_t)((int16_t)(batteryVoltage / 10) >> 8);
  lora_stream[4 * TEMP_SENSORS + 11] = (uint8_t)((int16_t)(batteryVoltage / 10) & 0xFF);

  // Start job
  do_send(&sendjob);

  os_runloop_once();

  delay(2000); // only read every 0,5 seconds

  //Sleep
  digitalWrite(POWER_ENABLE_PIN, LOW);
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);
  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);
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
  switch (ev) {
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
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      Serial.println(os_getTime() + sec2osticks(TX_INTERVAL));
      break;
    default:
      Serial.println(F("LoRa someting happened"));
      break;
  }
}

void do_send(osjob_t* j) {
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
void writeTo(byte address, byte val) {
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

  while (Wire.available()) {      // device may send less than requested (abnormal)
    acc_buffer[i] = Wire.read();  // receive a byte
    i++;
  }
  Wire.endTransmission();         // end transmission
}
long readVcc() {
  long result; // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}
