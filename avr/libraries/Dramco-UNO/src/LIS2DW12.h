/******************************************************************************
LIS2DW12.h
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

Please review the LICENSE.md file included with this example.

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __LIS2DW12_H__
#define __LIS2DW12_H__

#include "stdint.h"
#include "SPI.h"

#define I2C_MODE 0
#define SPI_MODE 1

// Return values 
typedef enum
{
	IMU_SUCCESS,
	IMU_HW_ERROR,
	IMU_NOT_SUPPORTED,
	IMU_GENERIC_ERROR,
	IMU_OUT_OF_BOUNDS,
	IMU_ALL_ONES_WARNING,
	//...
} status_t;

//This is the core operational class of the driver.
//  LIS2DW12Core contains only read and write operations towards the IMU (Inertial Measurement Unit).
//  To use the higher level functions, use the class LIS2DW12 which inherits
//  this class.

class LIS2DW12Core
{
public:
	LIS2DW12Core( uint8_t );
	LIS2DW12Core( uint8_t, uint8_t );
	LIS2DW12Core( uint8_t, uint8_t, SPISettings);
	~LIS2DW12Core() = default;
	
	status_t beginCore( void );
	
	//The following utilities read and write to the IMU

	//ReadRegisterRegion takes a uint8 array address as input and reads
	//  a chunk of memory into that array.
	status_t readRegisterRegion(uint8_t*, uint8_t, uint8_t );
	
	//readRegister reads one 8-bit register
	status_t readRegister(uint8_t*, uint8_t);
	
	//Reads two 8-bit regs, LSByte then MSByte order, and concatenates them.
	//  Acts as a 16-bit read operation
	status_t readRegisterInt16(int16_t*, uint8_t offset );
	
	//Writes an 8-bit byte;
	status_t writeRegister(uint8_t, uint8_t);
	
	//Change to embedded page
	status_t embeddedPage( void );
	
	//Change to base page
	status_t basePage( void );
	
private:
	
	//Communication stuff
	uint8_t commInterface;
	uint8_t I2CAddress;
	uint8_t chipSelectPin;
	SPISettings COMMSettings;

};

//This struct holds the settings the driver uses to do calculations
struct SensorSettings {
public:
	// CTRL1
	uint16_t	odr;
	uint8_t		mode;
	uint8_t 	lpMode;
	
	// CTRL2
	uint8_t 	csPuDisc;
	uint8_t 	i2cDisable;
	
	// CTRL3
	uint8_t		ppOd;
	uint8_t		lir;
	uint8_t		hiActive;
	
	// CTRL6
	uint16_t 	fs;
	uint8_t		lowNoise;
	
	uint8_t 	tapTh;
	uint8_t		latency;
	uint8_t		quiet;
	uint8_t		shock;
	
	float 		accelSensitivity;
	
};


//This is the highest level class of the driver. -> TO BE MODIFIED
//
//  class LIS2DW12 inherits the core and makes use of the beginCore()
//method through it's own begin() method.  It also contains the
//settings struct to hold user settings.

class LIS2DW12 : public LIS2DW12Core
{
public:
	//IMU settings
	SensorSettings settings;
	
	//Error checking
	uint16_t allOnesCounter;
	uint16_t nonSuccessCounter;

	//Constructor generates default SensorSettings.
	//(over-ride after construction if desired)
	LIS2DW12( uint8_t busType, uint8_t inputArg, SPISettings settingArg );
	~LIS2DW12() = default;
	
	//Call to apply SensorSettings
	status_t begin(void);

	//Returns the raw bits from the sensor cast as 16-bit signed integers
	int16_t readRawAccelX( void );
	int16_t readRawAccelY( void );
	int16_t readRawAccelZ( void );

	//Returns the values as floats.  Inside, this calls readRaw___();
	float readFloatAccelX( void );
	float readFloatAccelY( void );
	float readFloatAccelZ( void );

	//Temperature related methods
	int16_t readRawTemp( void );
	int8_t readRawTempLowRes( void );
	float readTempC( void );
	int8_t readTempCLowRes( void );
	float readTempF( void );
	int8_t readTempFLowRes( void );

	//TAP detection stuff
	uint8_t initDoubleTap( uint8_t );
	uint8_t initSingleTap( uint8_t );

	float calcAccel( int16_t );
	
private:

};


/************** Device Register  *******************/
#define LIS2DW12_OUT_T_L						0x0D
#define LIS2DW12_OUT_T_H						0x0E
#define LIS2DW12_WHO_AM_I						0x0F
#define LIS2DW12_CTRL1							0x20
#define LIS2DW12_CTRL2							0x21
#define LIS2DW12_CTRL3							0x22
#define LIS2DW12_CTRL4_INT1_PAD_CTRL			0x23
#define LIS2DW12_CTRL5_INT2_PAD_CTRL			0x24
#define LIS2DW12_CTRL6							0x25
#define LIS2DW12_OUT_T							0x26
#define LIS2DW12_STATUS							0x27
#define LIS2DW12_OUT_X_L						0x28
#define LIS2DW12_OUT_X_H						0x29
#define LIS2DW12_OUT_Y_L						0x2A
#define LIS2DW12_OUT_Y_H						0x2B
#define LIS2DW12_OUT_Z_L						0x2C
#define LIS2DW12_OUT_Z_H						0x2D
#define LIS2DW12_FIFO_CTRL						0x2E
#define LIS2DW12_FIFO_SAMPLES					0x2F
#define LIS2DW12_TAP_THS_X						0x30
#define LIS2DW12_TAP_THS_Y						0x31
#define LIS2DW12_TAP_THS_Z						0x32
#define LIS2DW12_INT_DUR						0x33
#define LIS2DW12_WAKE_UP_THS					0x34
#define LIS2DW12_WAKE_UP_DUR					0x35
#define LIS2DW12_FREE_FALL						0x36
#define LIS2DW12_STATUS_DUP						0x37
#define LIS2DW12_WAKE_UP_SRC					0x38
#define LIS2DW12_TAP_SRC						0x39
#define LIS2DW12_SIXD_SRC						0x3A
#define LIS2DW12_ALL_INT_SRC					0x3B
#define LIS2DW12_X_OFS_USR						0x3C
#define LIS2DW12_Y_OFS_USR						0x3D
#define LIS2DW12_Z_OFS_USR						0x3E
#define LIS2DW12_CTRL_REG7						0x3F


/*******************************************************************************
* Register      : CTRL1
* Address       : 0x20
* Bit Group Name: LP_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_LP_MODE_1							= 0x00,
	LIS2DW12_LP_MODE_2							= 0x01,
	LIS2DW12_LP_MODE_3							= 0x02,
	LIS2DW12_LP_MODE_4							= 0x03
} LIS2DW12_LP_MODE_t;

/*******************************************************************************
* Register      : CTRL1
* Address       : 0x20
* Bit Group Name: MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_MODE_LOW_POWER						= 0x00,
	LIS2DW12_MODE_HIGH_PERF						= 0x04,
	LIS2DW12_MODE_SINGLE_CONV					= 0x08
} LIS2DW12_MODE_t;

/*******************************************************************************
* Register      : CTRL1
* Address       : 0x20
* Bit Group Name: ODR
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_ODR_POWER_DOWN						= 0x00,
	LIS2DW12_ODR_12_5_1_6HZ						= 0x10,
	LIS2DW12_ODR_12_5Hz							= 0x20,
	LIS2DW12_ODR_25Hz							= 0x30,
	LIS2DW12_ODR_50Hz							= 0x40,
	LIS2DW12_ODR_100Hz							= 0x50,
	LIS2DW12_ODR_200Hz							= 0x60,
	LIS2DW12_ODR_400_200Hz						= 0x70,
	LIS2DW12_ODR_800_200Hz						= 0x80,
	LIS2DW12_ODR_1600_200Hz						= 0x90
} LIS2DW12_ODR_t;

/*******************************************************************************
* Register      : CTRL2
* Address       : 0x21
* Bit Group Name: SIM
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_SIM_4_WIRE							= 0x00,
	LIS2DW12_SIM_3_WIRE							= 0x01
} LIS2DW12_SIM_t;

/*******************************************************************************
* Register      : CTRL2
* Address       : 0x21
* Bit Group Name: I2C_DISABLE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_I2C_ENABLE_I2C_AND_SPI 			= 0x00,
	LIS2DW12_I2C_ENABLE_SPI_ONLY 		 		= 0x02
} LIS2DW12_I2C_DISABLE_t;

/*******************************************************************************
* Register      : CTRL2
* Address       : 0x21
* Bit Group Name: IF_ADD_INC
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_IF_ADD_INC_DISABLE					= 0x00,
	LIS2DW12_IF_ADD_INC_ENABLE					= 0x04
} LIS2DW12_IF_ADD_INC_t;

/*******************************************************************************
* Register      : CTRL2
* Address       : 0x21
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_BDU_CONTINUOUS_UPDATE				= 0x00,
	LIS2DW12_BDU_NOT_UPDATE_MSB_LSB				= 0x08
} LIS2DW12_BDU_t;
	
/*******************************************************************************
* Register      : CTRL2
* Address       : 0x21
* Bit Group Name: CS_PU_DISC
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_CS_PU_DISC_CONNECT					= 0x00,
	LIS2DW12_CS_PU_DISC_DISCONNECT				= 0x10
} LIS2DW12_CS_PU_DISC_t;

/*******************************************************************************
* Register      : CTRL2
* Address       : 0x21
* Bit Group Name: SOFT_RESET
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_SOFT_RESET_DISABLE					= 0x00,
	LIS2DW12_SOFT_RESET_ENABLE					= 0x40
} LIS2DW12_SOFT_RESET_t;

/*******************************************************************************
* Register      : CTRL2
* Address       : 0x21
* Bit Group Name: BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_BOOT_DISABLE						= 0x00,
	LIS2DW12_BOOT_ENABLE						= 0x80
} LIS2DW12_BOOT_t;

/*******************************************************************************
* Register      : CTRL3
* Address       : 0x22
* Bit Group Name: SLP_MODE_1
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_SLP_MODE_1_CONV_START				= 0x01
} LIS2DW12_SLP_MODE_1_t;

/*******************************************************************************
* Register      : CTRL3
* Address       : 0x22
* Bit Group Name: SLP_MODE_SEL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_SLP_MODE_SEL_EN_INT2				= 0x00,
	LIS2DW12_SLP_MODE_SEL_EN_MODE1				= 0x02
} LIS2DW12_SLP_MODE_SEL_t;

/*******************************************************************************
* Register      : CTRL3
* Address       : 0x22
* Bit Group Name: H_LACTIVE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_H_LACTIVE_HIGH						= 0x00,
	LIS2DW12_H_LACTIVE_LOW						= 0x08
} LIS2DW12_H_LACTIVE_t;

/*******************************************************************************
* Register      : CTRL3
* Address       : 0x22
* Bit Group Name: LIR
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_LIR_NOT_LATCHED					= 0x00,
	LIS2DW12_LIR_LATCHED						= 0x10
} LIS2DW12_LIR_t;

/*******************************************************************************
* Register      : CTRL3
* Address       : 0x22
* Bit Group Name: PP_OD
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_PP_OD_PUSH_PULL					= 0x00,
	LIS2DW12_PP_OD_OPEN_DRAIN					= 0x20
} LIS2DW12_PP_OD_t;

/*******************************************************************************
* Register      : CTRL3
* Address       : 0x22
* Bit Group Name: ST
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_ST_NORMAL							= 0x00,
	LIS2DW12_ST_POSITIVE						= 0x40,
	LIS2DW12_ST_NEGATIVE						= 0x80
} LIS2DW12_ST_t;

/*******************************************************************************
* Register      : CTRL4_INT1_PAD_CTRL
* Address       : 0x23
* Bit Group Name: INT1_DRDY
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT1_DRDY_DISABLE					= 0x00,
	LIS2DW12_INT1_DRDY_ENABLE					= 0x01
} LIS2DW12_INT1_DRDY_t;

/*******************************************************************************
* Register      : CTRL4_INT1_PAD_CTRL
* Address       : 0x23
* Bit Group Name: INT1_FTH
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT1_FTH_DISABLE					= 0x00,
	LIS2DW12_INT1_FTH_ENABLE					= 0x02
} LIS2DW12_INT1_FTH_t;

/*******************************************************************************
* Register      : CTRL4_INT1_PAD_CTRL
* Address       : 0x23
* Bit Group Name: INT1_DIFF5
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT1_DIFF5_DISABLE					= 0x00,
	LIS2DW12_INT1_DIFF5_ENABLE					= 0x04
} LIS2DW12_INT1_DIFF5_t;

/*******************************************************************************
* Register      : CTRL4_INT1_PAD_CTRL
* Address       : 0x23
* Bit Group Name: INT1_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT1_TAP_DISABLE					= 0x00,
	LIS2DW12_INT1_TAP_ENABLE					= 0x08
} LIS2DW12_INT1_TAP_t;

/*******************************************************************************
* Register      : CTRL4_INT1_PAD_CTRL
* Address       : 0x23
* Bit Group Name: INT1_FF
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT1_FF_DISABLE					= 0x00,
	LIS2DW12_INT1_FF_ENABLE						= 0x10
} LIS2DW12_INT1_FF_t;

/*******************************************************************************
* Register      : CTRL4_INT1_PAD_CTRL
* Address       : 0x23
* Bit Group Name: INT1_WU
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT1_WU_DISABLE					= 0x00,
	LIS2DW12_INT1_WU_ENABLE						= 0x20
} LIS2DW12_INT1_WU_t;

/*******************************************************************************
* Register      : CTRL4_INT1_PAD_CTRL
* Address       : 0x23
* Bit Group Name: INT1_SINGLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT1_SINGLE_TAP_DISABLE			= 0x00,
	LIS2DW12_INT1_SINGLE_TAP_ENABLE				= 0x40
} LIS2DW12_INT1_SINGLE_TAP_t;

/*******************************************************************************
* Register      : CTRL4_INT1_PAD_CTRL
* Address       : 0x23
* Bit Group Name: INT1_6D
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT1_6D_DISABLE					= 0x00,
	LIS2DW12_INT1_6D_ENABLE						= 0x80
} LIS2DW12_INT1_6D_t;

/*******************************************************************************
* Register      : CTRL5_INT2_PAD_CTRL
* Address       : 0x24
* Bit Group Name: INT2_DRDY
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT2_DRDY_DISABLE					= 0x00,
	LIS2DW12_INT2_DRDY_ENABLE					= 0x01
} LIS2DW12_INT2_DRDY_t;

/*******************************************************************************
* Register      : CTRL5_INT2_PAD_CTRL
* Address       : 0x24
* Bit Group Name: INT2_FTH
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT2_FTH_DISABLE					= 0x00,
	LIS2DW12_INT2_FTH_ENABLE					= 0x02
} LIS2DW12_INT2_FTH_t;

/*******************************************************************************
* Register      : CTRL5_INT2_PAD_CTRL
* Address       : 0x24
* Bit Group Name: INT2_DIFF5
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT2_DIFF5_DISABLE					= 0x00,
	LIS2DW12_INT2_DIFF5_ENABLE					= 0x04
} LIS2DW12_INT2_DIFF5_t;

/*******************************************************************************
* Register      : CTRL5_INT2_PAD_CTRL
* Address       : 0x24
* Bit Group Name: INT2_OVR
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT2_OVR_DISABLE					= 0x00,
	LIS2DW12_INT2_OVR_ENABLE					= 0x08
} LIS2DW12_INT2_OVR_t;

/*******************************************************************************
* Register      : CTRL5_INT2_PAD_CTRL
* Address       : 0x24
* Bit Group Name: INT2_DRDY_T
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT2_DRDY_T_DISABLE				= 0x00,
	LIS2DW12_INT2_DRDY_T_ENABLE					= 0x10
} LIS2DW12_INT2_DRDY_T_t;

/*******************************************************************************
* Register      : CTRL5_INT2_PAD_CTRL
* Address       : 0x24
* Bit Group Name: INT2_BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT2_BOOT_DISABLE					= 0x00,
	LIS2DW12_INT2_BOOT_ENABLE					= 0x20
} LIS2DW12_INT2_BOOT_t;

/*******************************************************************************
* Register      : CTRL5_INT2_PAD_CTRL
* Address       : 0x24
* Bit Group Name: INT2_SLEEP_CHG
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT2_SLEEP_CHG_DISABLE				= 0x00,
	LIS2DW12_INT2_SLEEP_CHG_ENABLE				= 0x40
} LIS2DW12_INT2_SLEEP_CHG_t;

/*******************************************************************************
* Register      : CTRL5_INT2_PAD_CTRL
* Address       : 0x24
* Bit Group Name: INT2_SLEEP_STATE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT2_SLEEP_STATE_DISABLE			= 0x00,
	LIS2DW12_INT2_SLEEP_STATE_ENABLE			= 0x80
} LIS2DW12_INT2_SLEEP_STATE_t;

/*******************************************************************************
* Register      : CTRL6
* Address       : 0x25
* Bit Group Name: LOW_NOISE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_LOW_NOISE_DISABLE					= 0x00,
	LIS2DW12_LOW_NOISE_ENABLE					= 0x04
} LIS2DW12_LOW_NOISE_t;

/*******************************************************************************
* Register      : CTRL6
* Address       : 0x25
* Bit Group Name: FSD
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_FSD_LOW_PASS						= 0x00,
	LIS2DW12_FSD_HIGH_PASS						= 0x08
} LIS2DW12_FSD_t;

/*******************************************************************************
* Register      : CTRL6
* Address       : 0x25
* Bit Group Name: FS
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_FS_2G								= 0x00,
	LIS2DW12_FS_4G								= 0x10,
	LIS2DW12_FS_8G								= 0x20,
	LIS2DW12_FS_16G								= 0x30
} LIS2DW12_FS_t;

/*******************************************************************************
* Register      : CTRL6
* Address       : 0x25
* Bit Group Name: BW_FILT
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_BW_FILT_ODR2						= 0x00,
	LIS2DW12_BW_FILT_ODR4						= 0x40,
	LIS2DW12_BW_FILT_ODR10						= 0x80,
	LIS2DW12_BW_FILT_ODR20						= 0xC0
} LIS2DW12_BW_FILT_t;

/*******************************************************************************
* Register      : FIFO_CTRL
* Address       : 0x2E
* Bit Group Name: FTH
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_FTH_MASK				  	0x1F
#define  	LIS2DW12_FTH_POSITION				0

/*******************************************************************************
* Register      : FIFO_CTRL
* Address       : 0x2E
* Bit Group Name: FMODE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_FMODE_FIFO_OFF						= 0x00,
	LIS2DW12_FMODE_STOP_FULL					= 0x20,
	LIS2DW12_FMODE_CONTINUOUS_TO_FIFO			= 0x60,
	LIS2DW12_FMODE_BYPASS_TO_CONTINUOUS			= 0x80,
	LIS2DW12_FMODE_CONTINUOUS_MODE				= 0xC0
} LIS2DW12_FMODE_t;

/*******************************************************************************
* Register      : TAP_THS_X
* Address       : 0x30
* Bit Group Name: TAP_THSX
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_TAP_THSX_MASK				  	0x1F
#define  	LIS2DW12_TAP_THSX_POSITION				0

/*******************************************************************************
* Register      : TAP_THS_X
* Address       : 0x30
* Bit Group Name: 6D_THS
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_6D_THS_6							= 0x00,
	LIS2DW12_6D_THS_11							= 0x20,
	LIS2DW12_6D_THS_16							= 0x40,
	LIS2DW12_6D_THS_21							= 0x60
} LIS2DW12_6D_THS_t;

/*******************************************************************************
* Register      : TAP_THS_X
* Address       : 0x30
* Bit Group Name: 4D_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_4D_EN_DISABLE							= 0x00,
	LIS2DW12_4D_EN_PORTRAIT_LANDSCAPE				= 0x80
} LIS2DW12_4D_EN_t;

/*******************************************************************************
* Register      : TAP_THS_Y
* Address       : 0x31
* Bit Group Name: TAP_THSY
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_TAP_THSY_MASK				  	0x1F
#define  	LIS2DW12_TAP_THSY_POSITION				0

/*******************************************************************************
* Register      : TAP_THS_Y
* Address       : 0x31
* Bit Group Name: TAP_PRIOR
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_TAP_PRIOR_XYZ1						= 0x00,
	LIS2DW12_TAP_PRIOR_YXZ1						= 0x20,
	LIS2DW12_TAP_PRIOR_XZY1						= 0x40,
	LIS2DW12_TAP_PRIOR_ZYX1						= 0x60,
	LIS2DW12_TAP_PRIOR_XYZ2						= 0x80,
	LIS2DW12_TAP_PRIOR_YZX2						= 0xA0,
	LIS2DW12_TAP_PRIOR_ZXY2						= 0xC0,
	LIS2DW12_TAP_PRIOR_ZYX2						= 0xE0
} LIS2DW12_TAP_PRIOR_t;

/*******************************************************************************
* Register      : TAP_THS_Z
* Address       : 0x32
* Bit Group Name: TAP_THSZ
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_TAP_THSZ_MASK				  	0x1F
#define  	LIS2DW12_TAP_THSZ_POSITION				0

/*******************************************************************************
* Register      : TAP_THS_Z
* Address       : 0x32
* Bit Group Name: TAP_Z_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_TAP_Z_EN_DISABLE					= 0x00,
	LIS2DW12_TAP_Z_EN_ENABLE					= 0x20
} LIS2DW12_TAP_Z_EN_t;

/*******************************************************************************
* Register      : TAP_THS_Z
* Address       : 0x32
* Bit Group Name: TAP_Y_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_TAP_Y_EN_DISABLE					= 0x00,
	LIS2DW12_TAP_Y_EN_ENABLE					= 0x40
} LIS2DW12_TAP_Y_EN_t;

/*******************************************************************************
* Register      : TAP_THS_Z
* Address       : 0x32
* Bit Group Name: TAP_X_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_TAP_X_EN_DISABLE					= 0x00,
	LIS2DW12_TAP_X_EN_ENABLE					= 0x80
} LIS2DW12_TAP_X_EN_t;

/*******************************************************************************
* Register      : INT_DUR
* Address       : 0x33
* Bit Group Name: SHOCK
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_SHOCK_MASK				  	0x03
#define  	LIS2DW12_SHOCK_POSITION				0

/*******************************************************************************
* Register      : INT_DUR
* Address       : 0x33
* Bit Group Name: QUIET
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_QUIET_MASK				  	0x0C
#define  	LIS2DW12_QUIET_POSITION				2

/*******************************************************************************
* Register      : INT_DUR
* Address       : 0x33
* Bit Group Name: LATENCY
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_LATENCY_MASK				0xF0
#define  	LIS2DW12_LATENCY_POSITION			4

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0x34
* Bit Group Name: WK_THS
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_WK_THS_MASK				0x3F
#define  	LIS2DW12_WK_THS_POSITION			0

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0x34
* Bit Group Name: SLEEP_ON
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_SLEEP_ON_DISABLE					= 0x00,
	LIS2DW12_SLEEP_ON_ENABLE					= 0x40
} LIS2DW12_SLEEP_ON_t;

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0x34
* Bit Group Name: SINGLE_DOUBLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_SINGLE_DOUBLE_TAP_ONLY_SINGLE		= 0x00,
	LIS2DW12_SINGLE_DOUBLE_TAP_BOTH_EN			= 0x80
} LIS2DW12_SINGLE_DOUBLE_TAP_t;

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0x35
* Bit Group Name: SLEEP_DUR
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_SLEEP_DUR_MASK				0x0F
#define  	LIS2DW12_SLEEP_DUR_POSITION			0

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0x35
* Bit Group Name: STATIONARY
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_STATIONARY_DISABLE					= 0x00,
	LIS2DW12_STATIONARY_ENABLE					= 0x10
} LIS2DW12_STATIONARY_t;

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0x35
* Bit Group Name: WAKE_DUR
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_WAKE_DUR_MASK				0x60
#define  	LIS2DW12_WAKE_DUR_POSITION			5

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0x35
* Bit Group Name: FF_DUR5
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_FF_DUR5_LESS32						= 0x00,
	LIS2DW12_FF_DUR5_MORE32						= 0x80
} LIS2DW12_FF_DUR5_t;

/*******************************************************************************
* Register      : FREE_FALL
* Address       : 0x36
* Bit Group Name: FF_THS
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_FF_THS_MASK				0x07
#define  	LIS2DW12_FF_THS_POSITION			0

/*******************************************************************************
* Register      : FREE_FALL
* Address       : 0x36
* Bit Group Name: FF_DUR
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_FF_DUR_MASK				0xF8
#define  	LIS2DW12_FF_DUR_POSITION			3

/*******************************************************************************
* Register      : X_OFS_USR
* Address       : 0x3C
* Bit Group Name: X_OFS_USR
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_X_OFS_USR_MASK				0xFF
#define  	LIS2DW12_X_OFS_USR_POSITION			0

/*******************************************************************************
* Register      : Y_OFS_USR
* Address       : 0x3D
* Bit Group Name: Y_OFS_USR
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_Y_OFS_USR_MASK				0xFF
#define  	LIS2DW12_Y_OFS_USR_POSITION			0

/*******************************************************************************
* Register      : Z_OFS_USR
* Address       : 0x3E
* Bit Group Name: Z_OFS_USR
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_Z_OFS_USR_MASK				0xFF
#define  	LIS2DW12_Z_OFS_USR_POSITION			0

/*******************************************************************************
* Register      : CTRL_REG7
* Address       : 0x3F
* Bit Group Name: LPASS_ON6D
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_LPASS_ON6D_ODR2					= 0x00,
	LIS2DW12_LPASS_ON6D_LPF2					= 0x01
} LIS2DW12_LPASS_ON6D_t;

/*******************************************************************************
* Register      : CTRL_REG7
* Address       : 0x3F
* Bit Group Name: HP_REF_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_HP_REF_MODE_DISABLE				= 0x00,
	LIS2DW12_HP_REF_MODE_ENABLE					= 0x02
} LIS2DW12_HP_REF_MODE_t;

/*******************************************************************************
* Register      : CTRL_REG7
* Address       : 0x3F
* Bit Group Name: USR_OFF_W
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_USR_OFF_W_977U						= 0x00,
	LIS2DW12_USR_OFF_W_15M						= 0x40
} LIS2DW12_USR_OFF_W_t;

/*******************************************************************************
* Register      : CTRL_REG7
* Address       : 0x3F
* Bit Group Name: USR_OFF_ON_WU
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_USR_OFF_ON_WU_DISABLE				= 0x00,
	LIS2DW12_USR_OFF_ON_WU_ENABLE				= 0x80
} LIS2DW12_USR_OFF_ON_WU_t;

/*******************************************************************************
* Register      : CTRL_REG7
* Address       : 0x3F
* Bit Group Name: USR_OFF_ON_OUT
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_USR_OFF_ON_OUT_DISABLE				= 0x00,
	LIS2DW12_USR_OFF_ON_OUT_ENABLE				= 0x10
} LIS2DW12_USR_OFF_ON_OUT_t;

/*******************************************************************************
* Register      : CTRL_REG7
* Address       : 0x3F
* Bit Group Name: INTERRUPTS_ENABLE
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INTERRUPTS_ENABLE_DISABLE			= 0x00,
	LIS2DW12_INTERRUPTS_ENABLE_ENABLE			= 0x20
} LIS2DW12_INTERRUPTS_ENABLE_t;

/*******************************************************************************
* Register      : CTRL_REG7
* Address       : 0x3F
* Bit Group Name: INT2_ON_INT1
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_INT2_ON_INT1_DISABLE				= 0x00,
	LIS2DW12_INT2_ON_INT1_ENABLE				= 0x40
} LIS2DW12_INT2_ON_INT1_t;

/*******************************************************************************
* Register      : CTRL_REG7
* Address       : 0x3F
* Bit Group Name: DRDY_PULSED
* Permission    : RW
*******************************************************************************/
typedef enum {
	LIS2DW12_DRDY_PULSED_DISABLE				= 0x00,
	LIS2DW12_DRDY_PULSED_ENABLE					= 0x80
} LIS2DW12_DRDY_PULSED_t;

#endif  // End of __LIS2DW12_H__ definition check
