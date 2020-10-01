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

#include "Wire.h"
#include "SPI.h"


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
LIS2DW12Core::LIS2DW12Core( uint8_t busType, uint8_t inputArg, SPISettings settingArg) : commInterface(I2C_MODE), I2CAddress(0x19), chipSelectPin(10), COMMSettings(250000, MSBFIRST, SPI_MODE3)
{
	commInterface = busType;
	if( commInterface == I2C_MODE )
	{
		I2CAddress = inputArg;
	}
	if( commInterface == SPI_MODE )
	{
		chipSelectPin = inputArg;
		COMMSettings = settingArg;
	}

}

status_t LIS2DW12Core::beginCore(void)
{
	status_t returnError = IMU_SUCCESS;

	switch (commInterface) {

	case I2C_MODE:
		Wire.begin();
		break;

	case SPI_MODE:
		// start the SPI library:
		SPI.begin();
		
		pinMode(chipSelectPin, OUTPUT);
		digitalWrite(chipSelectPin, HIGH);
		break;
	default:
		break;
	}

	//Spin for a few ms
	volatile uint8_t temp = 0;
	for( uint16_t i = 0; i < 10000; i++ )
	{
		temp++;
	}

	//Check the ID register to determine if the operation was a success.
	uint8_t readCheck;
	readRegister(&readCheck, LIS2DW12_WHO_AM_I);
	if( readCheck != 0x44 )
	{
		returnError = IMU_HW_ERROR;
	}

	return returnError;

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
status_t LIS2DW12Core::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	status_t returnError = IMU_SUCCESS;

	//define pointer that will point to the external space
	uint8_t i = 0;
	uint8_t c = 0;
	uint8_t tempFFCounter = 0;

	switch (commInterface) {

	case I2C_MODE:
		Wire.beginTransmission(I2CAddress);
		Wire.write(offset);
		if( Wire.endTransmission() != 0 )
		{
			returnError = IMU_HW_ERROR;
		}
		else  //OK, all worked, keep going
		{
			// request 6 bytes from slave device
			Wire.requestFrom(I2CAddress, length);
			while ( (Wire.available()) && (i < length))  // slave may send less than requested
			{
				c = Wire.read(); // receive a byte as character
				*outputPointer = c;
				outputPointer++;
				i++;
			}
		}
		break;

	case SPI_MODE:
		SPI.beginTransaction(COMMSettings);
		// take the chip select low to select the device:
		digitalWrite(chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset | 0x80);  //Ored with "read request" bit
		while ( i < length ) // slave may send less than requested
		{
			c = SPI.transfer(0x00); // receive a byte as character
			if( c == 0xFF )
			{
				//May have problem
				tempFFCounter++;
			}
			*outputPointer = c;
			outputPointer++;
			i++;
		}
		if( tempFFCounter == i )
		{
			//Ok, we've recieved all ones, report
			returnError = IMU_ALL_ONES_WARNING;
		}
		// take the chip select high to de-select:
		digitalWrite(chipSelectPin, HIGH);
		SPI.endTransaction();
		break;

	default:
		break;
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
	uint8_t result;
	uint8_t numBytes = 1;
	status_t returnError = IMU_SUCCESS;

	switch (commInterface) {

	case I2C_MODE:
		Wire.beginTransmission(I2CAddress);
		Wire.write(offset);
		if( Wire.endTransmission() != 0 )
		{
			returnError = IMU_HW_ERROR;
		}
		Wire.requestFrom(I2CAddress, numBytes);
		while ( Wire.available() ) // slave may send less than requested
		{
			result = Wire.read(); // receive a byte as a proper uint8_t
		}
		break;

	case SPI_MODE:
		SPI.beginTransaction(COMMSettings);
		// take the chip select low to select the device:
		digitalWrite(chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset | 0x80);  //Ored with "read request" bit
		// send a value of 0 to read the first byte returned:
		result = SPI.transfer(0x00);
		// take the chip select high to de-select:
		digitalWrite(chipSelectPin, HIGH);
		SPI.endTransaction();
		
		if( result == 0xFF )
		{
			//we've recieved all ones, report
			returnError = IMU_ALL_ONES_WARNING;
		}
		break;

	default:
		break;
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
status_t LIS2DW12Core::readRegisterInt16( int16_t* outputPointer, uint8_t offset )
{
	uint8_t myBuffer[2];
	status_t returnError = readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);
	
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
	status_t returnError = IMU_SUCCESS;
	switch (commInterface) {
	case I2C_MODE:
		//Write the byte
		Wire.beginTransmission(I2CAddress);
		Wire.write(offset);
		Wire.write(dataToWrite);
		if( Wire.endTransmission() != 0 )
		{
			returnError = IMU_HW_ERROR;
		}
		break;

	case SPI_MODE:
		SPI.beginTransaction(COMMSettings);
		// take the chip select low to select the device:
		digitalWrite(chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset);
		// send a value of 0 to read the first byte returned:
		SPI.transfer(dataToWrite);
		// decrement the number of bytes left to read:
		// take the chip select high to de-select:
		digitalWrite(chipSelectPin, HIGH);
		SPI.endTransaction();
		break;
		
		//No way to check error on this write (Except to read back but that's not reliable)

	default:
		break;
	}

	return returnError;
}

//****************************************************************************//
//
//  Main user class -- wrapper for the core class + maths
//
//  Construct with same rules as the core ( uint8_t busType, uint8_t inputArg )
//
//****************************************************************************//
LIS2DW12::LIS2DW12( uint8_t busType, uint8_t inputArg, SPISettings settingArg ) : LIS2DW12Core( busType, inputArg, settingArg )
{
	//Construct with these default settings
	
	// CTRL1
	settings.mode				= 0;		// 0 = low power, 1 = high performance, 2 = single data conversion
	settings.lpMode				= 1;		// 1 = lp mode 1 (12 bit), 2 = lp mode 2 (14 bit) ...
	settings.odr				= 200;		// Hz. Default is 0 = power down
	
	// CTRL2
	settings.csPuDisc			= 0;		// 0 = pull-up connected to CS pin
	settings.i2cDisable			= 1;		// 0 = i2c enable, 1 = i2c disable
	
	// CTRL3
	settings.ppOd				= 0;		// 0 = push-pull, 1 = open-drain
	settings.lir				= 1;		// 0 = interrupt not latched, 1 = interrupt signal latched
	settings.hiActive			= 1;		// 0 = active high, 1 = active low
	
	// CTRL6
	settings.fs					= 2;		// 2g, 4g, 8g, 16g
	settings.lowNoise			= 1;		// 1 = low noise enabled
	
	settings.tapTh				= 0x0C;		// threshold for tap detection
	settings.latency			= 0x30;		// latency for double tap detection ((0x40 >> 4) * 32 / ODR)
	settings.quiet				= 0x08;		// quiet time window for double tap detection ((0x08 >> 2) * 4 / ODR)
	settings.shock				= 0x02;		// shock time window for double tap detection (0x02 * 8 / ODR)
	
	settings.accelSensitivity	= 0.244;	// set correct sensitivity from LIS2DW12 Application Notes (FS = 2g, resolution = 14bit)
											// this is a function of full scale setting (FS) and resolution (12 or 14 bit format)

	allOnesCounter 				= 0;
	nonSuccessCounter 			= 0;

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
status_t LIS2DW12::begin()
{
	//Check the settings structure values to determine how to setup the device
	uint8_t dataToWrite = 0;  //Temporary variable

	//Begin the inherited core.  This gets the physical wires connected
	status_t returnError = beginCore();

	//Setup CTRL1 register******************************
	dataToWrite = 0; //Start Fresh!
	//CTRL1 mode
	switch (settings.mode) {
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
	switch (settings.lpMode) {
	default:  //set mode to low power mode 1
	case 1:
		dataToWrite |= LIS2DW12_LP_MODE_1;
		break;
	case 2:
		dataToWrite |= LIS2DW12_LP_MODE_2;
		break;
	case 3:
		dataToWrite |= LIS2DW12_LP_MODE_3;
		break;
	case 4:
		dataToWrite |= LIS2DW12_LP_MODE_4;
		break;
	}
	//Lastly, patch in ODR
	switch (settings.odr) {
	case 0:
		dataToWrite |= LIS2DW12_ODR_POWER_DOWN;
		break;
	case 2:
		dataToWrite |= LIS2DW12_ODR_12_5_1_6HZ;
		break;
	case 13:
		dataToWrite |= LIS2DW12_ODR_12_5Hz;
		break;
	case 25:
		dataToWrite |= LIS2DW12_ODR_25Hz;
		break;
	case 50:
		dataToWrite |= LIS2DW12_ODR_50Hz;
		break;
	case 100:
		dataToWrite |= LIS2DW12_ODR_100Hz;
		break;
	case 200:
		dataToWrite |= LIS2DW12_ODR_200Hz;
		break;
	default:  //Set default to 400
	case 400:
		dataToWrite |= LIS2DW12_ODR_400_200Hz;
		break;
	case 800:
		dataToWrite |= LIS2DW12_ODR_800_200Hz;
		break;
	case 1600:
		dataToWrite |= LIS2DW12_ODR_1600_200Hz;
		break;
	}

	//Now, write the patched together data if it's different from default value
	if(dataToWrite != 0) {
		writeRegister(LIS2DW12_CTRL1, dataToWrite);
	}

	//Setup CTRL2 register******************************
	dataToWrite = 4; //Start Fresh! (this is CTRL2 default value)
	//CTRL2 CS_PU_DISC
	switch (settings.csPuDisc) {
	default:  //set default CS pull-up conected
	case 0:
		dataToWrite |= LIS2DW12_CS_PU_DISC_CONNECT;
		break;
	case 1:
		dataToWrite |= LIS2DW12_CS_PU_DISC_DISCONNECT;
		break;
	}
	//Next, set i2c disable
	switch (settings.i2cDisable) {
	default:  //set mode i2c and spi enabled
	case 0:
		dataToWrite |= LIS2DW12_I2C_ENABLE_I2C_AND_SPI;
		break;
	case 1:
		dataToWrite |= LIS2DW12_I2C_ENABLE_SPI_ONLY;
		break;
	}

	//Now, write the patched together data if it's defferent from default value
	if(dataToWrite != 4) {
		writeRegister(LIS2DW12_CTRL2, dataToWrite);
	}
	
	//Setup CTRL3 register******************************
	dataToWrite = 0; //Start Fresh!
	//Next, set pull-up/open-drain
	switch (settings.ppOd) {
	default:  //set mode to push-pull
	case 0:
		dataToWrite |= LIS2DW12_PP_OD_PUSH_PULL;
		break;
	case 1:
		dataToWrite |= LIS2DW12_PP_OD_OPEN_DRAIN;
		break;
	}
	//Next, set pull-up/open-drain
	switch (settings.lir) {
	default:  //set latched interrupt to not latched
	case 0:
		dataToWrite |= LIS2DW12_LIR_NOT_LATCHED;
		break;
	case 1:
		dataToWrite |= LIS2DW12_LIR_LATCHED;
		break;
	}
	//Next, set high active
	switch (settings.hiActive) {
	default:  //set high active mode
	case 0:
		dataToWrite |= LIS2DW12_H_LACTIVE_HIGH;
		break;
	case 1:
		dataToWrite |= LIS2DW12_H_LACTIVE_LOW;
		break;
	}

	//Now, write the patched together data if it's different from default value
	if(dataToWrite != 0) {
		writeRegister(LIS2DW12_CTRL3, dataToWrite);
		dataToWrite = 0; //Start Fresh!
	}

	//Setup CTRL6 register******************************
	//CTRL6 set full scale
	switch (settings.fs) {
	default:  //set full scale to 2g
	case 2:
		dataToWrite |= LIS2DW12_FS_2G;
		break;
	case 4:
		dataToWrite |= LIS2DW12_FS_4G;
		break;
	case 8:
		dataToWrite |= LIS2DW12_FS_8G;
		break;
	case 16:
		dataToWrite |= LIS2DW12_FS_16G;
		break;
	}
	//Next, set low noise
	switch (settings.lowNoise) {
	default:  //set low noise disable
	case 0:
		dataToWrite |= LIS2DW12_LOW_NOISE_DISABLE;
		break;
	case 1:
		dataToWrite |= LIS2DW12_LOW_NOISE_ENABLE;
		break;
	}

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
int16_t LIS2DW12::readRawAccelX( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LIS2DW12_OUT_X_L );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LIS2DW12::readFloatAccelX( void )
{
	float output = calcAccel(readRawAccelX());
	return output;
}

int16_t LIS2DW12::readRawAccelY( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LIS2DW12_OUT_Y_L );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LIS2DW12::readFloatAccelY( void )
{
	float output = calcAccel(readRawAccelY());
	return output;
}

int16_t LIS2DW12::readRawAccelZ( void )
{
	int16_t output;
	status_t errorLevel = readRegisterInt16( &output, LIS2DW12_OUT_Z_L );
	if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}
	return output;
}
float LIS2DW12::readFloatAccelZ( void )
{
	float output = calcAccel(readRawAccelZ());
	return output;
}

float LIS2DW12::calcAccel( int16_t input )
{
	float output = (float)input / 4 * settings.accelSensitivity;
	return output;
}

//****************************************************************************//
//
//  Temperature section
//
//****************************************************************************//
int16_t LIS2DW12::readRawTemp( void )
{
	int16_t output;
	
	readRegisterInt16( &output, LIS2DW12_OUT_T_L );
	
	return output;
}  

int8_t LIS2DW12::readRawTempLowRes( void )
{
	uint8_t output;
	
	readRegister( &output, LIS2DW12_OUT_T );
	
	return (int8_t)output;
}

float LIS2DW12::readTempC( void )
{
	float output = (float)readRawTemp() / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset

	return output;
}

int8_t LIS2DW12::readTempCLowRes(  void )
{
	int8_t output = readRawTempLowRes();
	output += 25; // Add 25 degrees to remove offset
	
	return output;
}

float LIS2DW12::readTempF( void )
{
	float output = (float)readRawTemp() / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset
	output = (output * 9) / 5 + 32;

	return output;
}

int8_t LIS2DW12::readTempFLowRes( void )
{
	int8_t output = readRawTempLowRes();
	output += 25;
	output = (output * 9) / 5 + 32;
	
	return output;
}

//****************************************************************************//
//
//  TAP detection section
//
//****************************************************************************//
uint8_t LIS2DW12::initDoubleTap( uint8_t axis ) {
	//Error accumulation variable
	uint8_t errorAccumulator = 0;

	uint8_t dataToWrite;  //Temporary variable

	// Set bit INT1_TAP in CTRL4 rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |=  LIS2DW12_INT1_TAP_ENABLE;

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_CTRL4_INT1_PAD_CTRL, dataToWrite);
	
	// set X axis TH if x axis or all axis are on
	if(axis == 0 || axis > 2)
	{
		// Set threshold on X in TAP_THS_X rgister
		dataToWrite = 0;  // Start fresh!
		dataToWrite |=  settings.tapTh;

		// //Now, write the patched together data
		errorAccumulator += writeRegister(LIS2DW12_TAP_THS_X, dataToWrite);
	}
	
	dataToWrite = 0;  // Start fresh!

	if(axis == 1 || axis > 2)
	{
		// Set threshold on Y and axis priority (zyx) in TAP_THS_Y rgister
		dataToWrite |=  settings.tapTh;
	}
	
	switch(axis)
	{
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
	
	if(axis == 2 || axis > 2)
	{
		// Set threshold on Z and enable tap on all axis in TAP_THS_Z rgister
		dataToWrite |=  settings.tapTh;
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
	dataToWrite |= settings.latency;
	dataToWrite |= settings.quiet;
	dataToWrite |= settings.shock;

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

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_CTRL_REG7, dataToWrite);

	return errorAccumulator;
}

uint8_t LIS2DW12::initSingleTap( uint8_t axis ) {
	//Error accumulation variable
	uint8_t errorAccumulator = 0;

	uint8_t dataToWrite;  //Temporary variable

	// Set bit INT1_TAP in CTRL4 rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |=  LIS2DW12_INT1_SINGLE_TAP_ENABLE;

	// //Now, write the patched together data
	errorAccumulator += writeRegister(LIS2DW12_CTRL4_INT1_PAD_CTRL, dataToWrite);
	
	// set X axis TH if x axis or all axis are on
	if(axis == 0 || axis > 2)
	{
		// Set threshold on X in TAP_THS_X rgister
		dataToWrite = 0;  // Start fresh!
		dataToWrite |=  settings.tapTh;

		// //Now, write the patched together data
		errorAccumulator += writeRegister(LIS2DW12_TAP_THS_X, dataToWrite);
	}
	
	dataToWrite = 0;  // Start fresh!
	
	if(axis == 1 || axis > 2)
	{
		// Set threshold on Y and axis priority (zyx) in TAP_THS_Y rgister
		dataToWrite |=  settings.tapTh;
	}
	
	switch(axis)
	{
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
	
	if(axis == 2 || axis > 2)
	{
		// Set threshold on Z and enable tap on all axis in TAP_THS_Z rgister
		dataToWrite |=  settings.tapTh;
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

	// Set desired shock time window and quiet time window in INT_DUR rgister
	dataToWrite = 0;  // Start fresh!
	dataToWrite |= settings.quiet;
	dataToWrite |= settings.shock;

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
