/******************************************************************************
LIS2DW12.cpp
LIS2DW12 Arduino Driver

Ion Bold 
October 15, 2018

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used

Development environment specifics:
Arduino IDE 1.8.5

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

//See LIS2DW12.h for additional topology notes.

#include "LIS2DW12.h"
#include "stdint.h"
#include "Arduino.h"
#include "../Wire/Wire.h"

//****************************************************************************//
//
//  LIS2DW12Core functions.
//
//  Construction arguments:
//  ( uint8_t busType, uint8_t inputArg ),
//
//    where inputArg is address for I2C_MODE and chip select pin
//    number for SPI_MODE
//
//  For SPI, construct LIS2DW12Core myIMU(SPI_MODE, 10, SPISettings(250000, MSBFIRST, SPI_MODE3));
//  For I2C, construct LIS2DW12Core myIMU(I2C_MODE, 0x19); -> if SA0/SDO = VCC and myIMU(I2C_MODE, 0x18); -> if SA0/SDO = GND
//
//  Default construction is I2C mode, I2CAddress = 0x19.
//
//****************************************************************************//

void error(uint8_t errorcode); // Prototype of error function

LIS2DW12Core::LIS2DW12Core(void){

}

status_t LIS2DW12Core::beginCore(void){
	Wire.begin();
	
	//Spin for a few ms
	volatile uint8_t temp = 0;
	for( uint16_t i = 0; i < 10000; i++ ){
		temp++;
	}

	//Check the ID register to determine if the operation was a success.
	uint8_t readCheck;
	readRegister(&readCheck, LIS2DW12_WHO_AM_I);
	if(readCheck  != 0x44 ){
		return IMU_HW_ERROR;
	}

	return IMU_SUCCESS;

}

//****************************************************************************//
//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//
//  Note:  Does not know if the target memory space is an array or not, or
//    if there is the array is big enough.  if the variable passed is only
//    two bytes long and 3 bytes are requested, this will over-write some
//    other memory!
//
//****************************************************************************//
status_t LIS2DW12Core::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length){
	status_t returnError = IMU_SUCCESS;

	//define pointer that will point to the external space
	uint8_t i = 0;
	uint8_t c = 0;
	uint8_t tempFFCounter = 0;

	Wire.beginTransmission(I2CAddress);
	Wire.write(offset);
	if( Wire.endTransmission() != 0 ){
		returnError = IMU_HW_ERROR;
	}
	else{//OK, all worked, keep going
		// request 6 bytes from slave device
		Wire.requestFrom((uint8_t)I2CAddress, (uint8_t)length);
		unsigned long startMillis = millis();
		while ( (Wire.available()) && (i < length) && millis() - startMillis < I2C_TIMEOUT)  // slave may send less than requested
		{
			c = Wire.read(); // receive a byte as character
			*outputPointer = c;
			outputPointer++;
			i++;
		}
		if (i==0){
			Serial.println("error");
			error(2);
		}
	}

	return returnError;
}

//****************************************************************************//
//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t LIS2DW12Core::readRegister(uint8_t* outputPointer, uint8_t offset) {
	//Return value
	uint8_t result =0;
	uint8_t numBytes = 1;
	status_t returnError = IMU_SUCCESS;

	Wire.beginTransmission(I2CAddress);
	Wire.write(offset);
	if( Wire.endTransmission() != 0 ){
		returnError = IMU_HW_ERROR;
	}
	Wire.requestFrom((uint8_t)I2CAddress, numBytes);
	unsigned long startMillis = millis();
	while ( Wire.available() && millis() - startMillis < I2C_TIMEOUT){ // slave may send less than requested
		result = Wire.read(); // receive a byte as a proper uint8_t
	}
	if(result == 0){
		Serial.println("error");
		error(2);
	}
	
	*outputPointer = result;
	return returnError;
}

//****************************************************************************//
//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t LIS2DW12Core::readRegisterInt16( int16_t* outputPointer, uint8_t offset ){
	uint8_t myBuffer[2];
	status_t returnError = readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	// Changed to allow uint8_t/int16_t conversion, only in 12 bit mode
	int16_t output = (((myBuffer[0] | myBuffer[1] << 8)<<((sizeof(int)-2)*8)) >> ((sizeof(int)-2)*8));
	#if settings_lpMode == 1
		output = output >> 4;
	#else
		output = output >> 2;
	#endif
	*outputPointer = output;
	return returnError;
}

//****************************************************************************//
//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//
//****************************************************************************//
status_t LIS2DW12Core::writeRegister(uint8_t offset, uint8_t dataToWrite) {
	//Write the byte
	Wire.beginTransmission(I2CAddress);
	Wire.write(offset);
	Wire.write(dataToWrite);
	if( Wire.endTransmission() != 0 ){
		return IMU_HW_ERROR;
	}
		
	return IMU_SUCCESS;
}

//****************************************************************************//
//
//  Main user class -- wrapper for the core class + maths
//
//  Construct with same rules as the core ( uint8_t busType, uint8_t inputArg )
//
//****************************************************************************//
LIS2DW12::LIS2DW12(void ) {
	mode				= 0;		// 0 = low power, 1 = high performance, 2 = single data conversion
}

//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "myIMU.settings.commInterface = SPI_MODE;" or
//  "myIMU.settings.accelEnabled = 1;" to configure before calling .begin();
//
//****************************************************************************//
status_t LIS2DW12::begin(){
	 Wire.setTimeout( 1000 );

	//Check the settings structure values to determine how to setup the device
	uint8_t dataToWrite = 0;  //Temporary variable

	//Begin the inherited core.  This gets the physical wires connected
	status_t returnError = beginCore();

	//Setup CTRL1 register******************************
	dataToWrite = 0; //Start Fresh!
	//CTRL1 mode
	switch (mode) {
	default:  //set default low power mode
	case 0:
		dataToWrite |= LIS2DW12_MODE_LOW_POWER;
		break;
	case 1:
		dataToWrite |= LIS2DW12_MODE_HIGH_PERF;
		break;
	case 2:
		dataToWrite |= LIS2DW12_MODE_SINGLE_CONV;
		break;
	}

	//Next, set low power mode
	#if settings_lpMode == 1
		dataToWrite |= LIS2DW12_LP_MODE_1;
	#endif
	#if settings_lpMode == 2
		dataToWrite |= LIS2DW12_LP_MODE_2;
		break;
	#endif
	#if settings_lpMode == 3
		dataToWrite |= LIS2DW12_LP_MODE_3;
		break;
	#endif
	#if settings_lpMode == 4
		dataToWrite |= LIS2DW12_LP_MODE_4;
	#endif

	//Lastly, patch in ODR
	#if settings_odr == 0
		dataToWrite |= LIS2DW12_ODR_POWER_DOWN;
	#endif
	#if settings_odr == 2
		dataToWrite |= LIS2DW12_ODR_12_5_1_6HZ;
	#endif
	#if settings_odr == 13
		dataToWrite |= LIS2DW12_ODR_12_5Hz;
	#endif
	#if settings_odr == 25
		dataToWrite |= LIS2DW12_ODR_25Hz;
	#endif
	#if settings_odr == 50
		dataToWrite |= LIS2DW12_ODR_50Hz;
	#endif
	#if settings_odr == 100
		dataToWrite |= LIS2DW12_ODR_100Hz;
	#endif
	#if settings_odr == 200
		dataToWrite |= LIS2DW12_ODR_200Hz;
	#endif
	#if settings_odr == 400
		dataToWrite |= LIS2DW12_ODR_400_200Hz;
	#endif
	#if settings_odr == 800
		dataToWrite |= LIS2DW12_ODR_800_200Hz;
	#endif
	#if settings_odr == 1600
		dataToWrite |= LIS2DW12_ODR_1600_200Hz;
	#endif

	//Now, write the patched together data if it's different from default value
	if(dataToWrite != 0) {
		writeRegister(LIS2DW12_CTRL1, dataToWrite);
	}

	//Setup CTRL2 register******************************
	dataToWrite = 4; //Start Fresh! (this is CTRL2 default value)
	//CTRL2 CS_PU_DISC
	#if settings_csPuDisc == 0
		dataToWrite |= LIS2DW12_CS_PU_DISC_CONNECT;
	#endif
	#if settings_csPuDisc == 1
		dataToWrite |= LIS2DW12_CS_PU_DISC_DISCONNECT;
	#endif

	//Next, set i2c disable
	#if settings_i2cDisable == 0
		dataToWrite |= LIS2DW12_I2C_ENABLE_I2C_AND_SPI;
	#endif
	#if settings_i2cDisable == 1
		dataToWrite |= LIS2DW12_I2C_ENABLE_SPI_ONLY;
	#endif

	//Now, write the patched together data if it's defferent from default value
	if(dataToWrite != 4) {
		writeRegister(LIS2DW12_CTRL2, dataToWrite);
	}
	
	//Setup CTRL3 register******************************
	dataToWrite = 0; //Start Fresh!
	//Next, set pull-up/open-drain
	#if settings_ppOd == 0
		dataToWrite |= LIS2DW12_PP_OD_PUSH_PULL;
	#endif
	#if settings_ppOd == 1
		dataToWrite |= LIS2DW12_PP_OD_OPEN_DRAIN;
	#endif

	//Next, set pull-up/open-drain
	#if settings_lir == 0
		dataToWrite |= LIS2DW12_LIR_NOT_LATCHED;
	#endif
	#if settings_lir == 1
		dataToWrite |= LIS2DW12_LIR_LATCHED;
	#endif

	//Next, set high active
	#if settings_hiActive == 0
		dataToWrite |= LIS2DW12_H_LACTIVE_HIGH;
	#endif
	#if settings_hiActive == 1
		dataToWrite |= LIS2DW12_H_LACTIVE_LOW;
	#endif

	//Now, write the patched together data if it's different from default value
	if(dataToWrite != 0) {
		writeRegister(LIS2DW12_CTRL3, dataToWrite);
		dataToWrite = 0; //Start Fresh!
	}

	//Setup CTRL6 register******************************
	//CTRL6 set full scale

	#if settings_fs == 2
		dataToWrite |= LIS2DW12_FS_2G;
	#endif
	#if settings_fs == 4
		dataToWrite |= LIS2DW12_FS_4G;
	#endif
	#if settings_fs == 8
		dataToWrite |= LIS2DW12_FS_8G;
	#endif
	#if settings_fs == 16
		dataToWrite |= LIS2DW12_FS_16G;
	#endif

	//Next, set low noise
	#if settings_lowNoise == 0
		dataToWrite |= LIS2DW12_LOW_NOISE_DISABLE;
	#endif
	#if settings_lowNoise == 0
		dataToWrite |= LIS2DW12_LOW_NOISE_ENABLE;
	#endif

	//Now, write the patched together data if it's different from default value
	if(dataToWrite != 0) {
		writeRegister(LIS2DW12_CTRL6, dataToWrite);
	}

	//Return WHO AM I reg  //Not no mo!
	uint8_t result;
	readRegister(&result, LIS2DW12_WHO_AM_I);

	return returnError;
}

//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
int16_t LIS2DW12::readRawAccelX( void ){
	int16_t output;
	readRegisterInt16( &output, LIS2DW12_OUT_X_L );
	return output;
}

float LIS2DW12::readFloatAccelX( void ){
	return calcAccel(readRawAccelX());
}

int16_t LIS2DW12::readRawAccelY( void ){
	int16_t output;
	readRegisterInt16( &output, LIS2DW12_OUT_Y_L );
	return output;
}
float LIS2DW12::readFloatAccelY( void ){
	return calcAccel(readRawAccelY());
}

int16_t LIS2DW12::readRawAccelZ( void ){
	int16_t output;
	readRegisterInt16( &output, LIS2DW12_OUT_Z_L );
	return output;
}
float LIS2DW12::readFloatAccelZ( void ){
	return calcAccel(readRawAccelZ());
}

float LIS2DW12::calcAccel( int16_t input ){
	return (float)input  * accelSensitivity;
}

//****************************************************************************//
//
//  Temperature section
//
//****************************************************************************//
int16_t LIS2DW12::readRawTemp( void ){
	int16_t output;
	
	readRegisterInt16( &output, LIS2DW12_OUT_T_L );
	
	return output;
}  

int8_t LIS2DW12::readRawTempLowRes( void ){
	uint8_t output;
	
	readRegister( &output, LIS2DW12_OUT_T );
	
	return (int8_t)output;
}

float LIS2DW12::readTempC( void ){
	float output = (float)readRawTemp() / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset

	return output;
}


//****************************************************************************//
//
//  TAP detection section
//
//****************************************************************************//
uint8_t LIS2DW12::initDoubleTap( uint8_t axis ) {// 0: only X axis, 1: only Y axis, 2: only Z axis, >2: all axis
	//Error accumulation variable
	uint8_t errorAccumulator = 0;

	uint8_t dataToWrite;  //Temporary variable

	// Set bit INT1_TAP in CTRL4 rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |=  LIS2DW12_INT1_TAP_ENABLE;

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_CTRL4_INT1_PAD_CTRL, dataToWrite);
	
	// set X axis TH if x axis or all axis are on
	if(axis == 0 || axis > 2){
		// Set threshold on X in TAP_THS_X rgister
		dataToWrite = 0;  // Start fresh!
		dataToWrite |=  settings_tapTh;

		// //Now, write the patched together data
		errorAccumulator += writeRegister(LIS2DW12_TAP_THS_X, dataToWrite);
	}
	
	dataToWrite = 0;  // Start fresh!

	if(axis == 1 || axis > 2){
		// Set threshold on Y and axis priority (zyx) in TAP_THS_Y rgister
		dataToWrite |=  settings_tapTh;
	}
	
	switch(axis){
		case 0:			// only x axis
			dataToWrite |= LIS2DW12_TAP_PRIOR_XYZ1;
		break;
		case 1:			// only y axis
			dataToWrite |= LIS2DW12_TAP_PRIOR_YXZ1;
		break;
		case 2:			// only z axis
			dataToWrite |= LIS2DW12_TAP_PRIOR_ZYX1;
		break;
		default:		// all axis
			dataToWrite |= LIS2DW12_TAP_PRIOR_ZXY2;
		break;
	}

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_TAP_THS_Y, dataToWrite);

	dataToWrite = 0;  // Start fresh!
	
	if(axis == 2 || axis > 2){
		// Set threshold on Z and enable tap on all axis in TAP_THS_Z rgister
		dataToWrite |=  settings_tapTh;
	}
	switch(axis)
	{
		case 0:
			dataToWrite |= LIS2DW12_TAP_X_EN_ENABLE;
		break;
		case 1:
			dataToWrite |= LIS2DW12_TAP_Y_EN_ENABLE;
		break;
		case 2:
			dataToWrite |= LIS2DW12_TAP_Z_EN_ENABLE;
		break;
		default:
			dataToWrite |= LIS2DW12_TAP_X_EN_ENABLE;
			dataToWrite |= LIS2DW12_TAP_Y_EN_ENABLE;
			dataToWrite |= LIS2DW12_TAP_Z_EN_ENABLE;
		break;
	}

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_TAP_THS_Z, dataToWrite);

	// Set desired latency, shock time window and quiet time window in INT_DUR rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |= settings_latency;
	dataToWrite |= settings_quiet;
	dataToWrite |= settings_shock;

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_INT_DUR, dataToWrite);

	// Enable double tap recognition in WAKE_UP_THS rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |=  LIS2DW12_SINGLE_DOUBLE_TAP_BOTH_EN;

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_WAKE_UP_THS, dataToWrite);

	delay(10);
	// Enable interrupts in CTRL7 rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |=  LIS2DW12_INTERRUPTS_ENABLE_ENABLE;
	//dataToWrite |=  LIS2DW12_INT2_ON_INT1_ENABLE;

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_CTRL_REG7, dataToWrite);

	return errorAccumulator;
}

uint8_t LIS2DW12::initSingleTap( uint8_t axis ) {// 0: only X axis, 1: only Y axis, 2: only Z axis, >2: all axis
	//Error accumulation variable
	uint8_t errorAccumulator = 0;

	uint8_t dataToWrite;  //Temporary variable

	// Set bit INT1_TAP in CTRL4 rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |=  LIS2DW12_INT1_SINGLE_TAP_ENABLE;

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_CTRL4_INT1_PAD_CTRL, dataToWrite);
	
	// set X axis TH if x axis or all axis are on
	if(axis == 0 || axis > 2){
		// Set threshold on X in TAP_THS_X rgister
		dataToWrite = 0;  // Start fresh!
		dataToWrite |=  settings_tapTh;

		// //Now, write the patched together data
		errorAccumulator += writeRegister(LIS2DW12_TAP_THS_X, dataToWrite);
	}
	
	dataToWrite = 0;  // Start fresh!
	
	if(axis == 1 || axis > 2){
		// Set threshold on Y and axis priority (zyx) in TAP_THS_Y rgister
		dataToWrite |=  settings_tapTh;
	}
	
	switch(axis){
		case 0:			// only x axis
			dataToWrite |= LIS2DW12_TAP_PRIOR_XYZ1;
		break;
		case 1:			// only y axis
			dataToWrite |= LIS2DW12_TAP_PRIOR_YXZ1;
		break;
		case 2:			// only z axis
			dataToWrite |= LIS2DW12_TAP_PRIOR_ZYX1;
		break;
		default:		// all axis
			dataToWrite |= LIS2DW12_TAP_PRIOR_ZXY2;
		break;
	}

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_TAP_THS_Y, dataToWrite);

	dataToWrite = 0;  // Start fresh!
	
	if(axis == 2 || axis > 2){
		// Set threshold on Z and enable tap on all axis in TAP_THS_Z rgister
		dataToWrite |=  settings_tapTh;
	}
	switch(axis){
		case 0:
			dataToWrite |= LIS2DW12_TAP_X_EN_ENABLE;
		break;
		case 1:
			dataToWrite |= LIS2DW12_TAP_Y_EN_ENABLE;
		break;
		case 2:
			dataToWrite |= LIS2DW12_TAP_Z_EN_ENABLE;
		break;
		default:
			dataToWrite |= LIS2DW12_TAP_X_EN_ENABLE;
			dataToWrite |= LIS2DW12_TAP_Y_EN_ENABLE;
			dataToWrite |= LIS2DW12_TAP_Z_EN_ENABLE;
		break;
	}

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_TAP_THS_Z, dataToWrite);

	// Set desired shock time window and quiet time window in INT_DUR rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |= settings_quiet;
	dataToWrite |= settings_shock;

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_INT_DUR, dataToWrite);

	delay(10);
	// Enable interrupts in CTRL7 rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |=  LIS2DW12_INTERRUPTS_ENABLE_ENABLE;

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_CTRL_REG7, dataToWrite);

	return errorAccumulator;
}

uint8_t LIS2DW12::initFreefall( ) {
	//Error accumulation variable
	uint8_t errorAccumulator = 0;

	uint8_t dataToWrite;  //Temporary variable

	// Set bit INT1_TAP in CTRL4 rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |=  LIS2DW12_INT1_FF_ENABLE;

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_CTRL4_INT1_PAD_CTRL, dataToWrite);
	
	// Enable interrupts in CTRL7 rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |=  LIS2DW12_INTERRUPTS_ENABLE_ENABLE;

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_CTRL_REG7, dataToWrite);

	return errorAccumulator;
}


uint8_t LIS2DW12::initWakeUp( ) {
	//Error accumulation variable
	uint8_t errorAccumulator = 0;

	uint8_t dataToWrite;  //Temporary variable

	// Set bit INT1_TAP in CTRL4 rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |=  LIS2DW12_INT1_WU_ENABLE;

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_CTRL4_INT1_PAD_CTRL, dataToWrite);
	
	writeRegister(LIS2DW12_WAKE_UP_THS, settings_wakeTh);
	writeRegister(LIS2DW12_WAKE_UP_DUR, settings_wakeDur);

	// Enable interrupts in CTRL7 rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |=  LIS2DW12_INTERRUPTS_ENABLE_ENABLE;

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_CTRL_REG7, dataToWrite);

	return errorAccumulator;
}


void LIS2DW12::printStatus(){
	uint8_t status = 0;
	readRegister(&status, LIS2DW12_STATUS);
	Serial.println(status, BIN);
}