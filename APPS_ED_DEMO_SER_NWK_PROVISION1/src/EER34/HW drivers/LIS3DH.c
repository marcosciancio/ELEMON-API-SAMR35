/******************** (C) COPYRIGHT 2020 Elemon ********************************
  File Name          : LIS3DH.c
  Author             : Marcos Ciancio
  Version            : V1.2
  Date               : 2021/12
  Description        : Configuration & various feature implementation of LIS3DH
********************************************************************************
  THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
  AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
  INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
  CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
  INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.

  THIS SOURCE CODE IS PROTECTED BY A LICENSE.
  FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
  IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/

// Includes
#include "LIS3DH.h"
#include "I2C_custom.h"
#include "stdio.h"
#include "stdbool.h"

#define BIN(b7,b6,b5,b4,b3,b2,b1,b0) ((b0)+((b1)<<1)+((b2)<<2)+((b3)<<3)+((b4)<<4)+((b5)<<5)+((b6)<<6)+((b7)<<7))

// 8-bit conversion function
#define B8_(x) ((x&0x0000000FLU)?1:0) +((x&0x000000F0LU)?2:0)+((x&0x00000F00LU)?4:0)+((x&0x0000F000LU)?8:0)+((x&0x000F0000LU)?16:0)+((x&0x00F00000LU)?32:0)+((x&0x0F000000LU)?64:0)+((x&0xF0000000LU)?128:0)

//Macros
// Device Registers
#define LIS3DH_STATUS_REG_AUX         0x07
#define LIS3DH_OUT_ADC1_L             0x08
#define LIS3DH_OUT_ADC1_H             0x09
#define LIS3DH_OUT_ADC2_L             0x0A
#define LIS3DH_OUT_ADC2_H             0x0B
#define LIS3DH_OUT_ADC3_L             0x0C
#define LIS3DH_OUT_ADC3_H             0x0D
#define LIS3DH_INT_COUNTER_REG        0x0E
#define LIS3DH_WHO_AM_I               0x0F

#define LIS3DH_TEMP_CFG_REG           0x1F
#define LIS3DH_CTRL_REG1              0x20
#define LIS3DH_CTRL_REG2              0x21
#define LIS3DH_CTRL_REG3              0x22
#define LIS3DH_CTRL_REG4              0x23
#define LIS3DH_CTRL_REG5              0x24
#define LIS3DH_CTRL_REG6              0x25
#define LIS3DH_REFERENCE              0x26
#define LIS3DH_STATUS_REG2            0x27
#define LIS3DH_OUT_X_L                0x28
#define LIS3DH_OUT_X_H                0x29
#define LIS3DH_OUT_Y_L                0x2A
#define LIS3DH_OUT_Y_H                0x2B
#define LIS3DH_OUT_Z_L                0x2C
#define LIS3DH_OUT_Z_H                0x2D
#define LIS3DH_FIFO_CTRL_REG          0x2E
#define LIS3DH_FIFO_SRC_REG           0x2F
#define LIS3DH_INT1_CFG               0x30
#define LIS3DH_INT1_SRC               0x31
#define LIS3DH_INT1_THS               0x32
#define LIS3DH_INT1_DURATION          0x33

#define LIS3DH_CLICK_CFG              0x38
#define LIS3DH_CLICK_SRC              0x39
#define LIS3DH_CLICK_THS              0x3A
#define LIS3DH_TIME_LIMIT             0x3B
#define LIS3DH_TIME_LATENCY           0x3C
#define LIS3DH_TIME_WINDOW            0x3D

// Datatypes

// Variables

struct LIS3DH_settings_t LIS3DH_settings;
struct LIS3DH_t 				 LIS3DH;

// Prototypes

static LIS3DH_Status LIS3DH_WriteByte(uint8_t Register, uint8_t value)
{
  I2Ccustom_Status res;

  res = I2C_customWriteStream ( LIS3DH_ADDRESS , 1 , Register , &value);

	if (res != I2C_CUSTOM_OK) return LIS3DH_ERROR ;

	return LIS3DH_OK ;
}

static LIS3DH_Status LIS3DH_ReadByte(uint8_t Addr,  uint8_t * value)
{
  I2Ccustom_Status res;
	
	res = I2C_customRead ( LIS3DH_ADDRESS , 1 , Addr , value ); 
  if ( res != I2C_CUSTOM_OK ) return LIS3DH_ERROR ;

	return LIS3DH_OK ;
}

static double LIS3DH_calcAccel( int16_t input )
{
	double output;
	switch(LIS3DH_settings.accelRange)
	{
		case 2:
		output = (double)input / 16384;
		break;
		case 4:
		output = (double)input / 8192;
		break;
		case 8:
		output = (double)input / 4096;
		break;
		case 16:
		output = (double)input / 1280;
		break;
		default:
		output = 0;
		break;
	}
	return output;
}

static void LIS3DH_getAdcTemp ( void )
{
	uint8_t output_8;

	LIS3DH_ReadByte( LIS3DH_OUT_ADC1_H, &output_8);
	LIS3DH.adc = output_8;

	LIS3DH_ReadByte( LIS3DH_OUT_ADC1_L, &output_8);
	LIS3DH.adc = LIS3DH.adc * 256 + output_8;

	LIS3DH_ReadByte( LIS3DH_OUT_ADC1_H, &output_8);
	LIS3DH.adc = output_8;

	LIS3DH_ReadByte( LIS3DH_OUT_ADC1_L, &output_8);
	LIS3DH.adc = LIS3DH.adc * 256 + output_8;
	
	LIS3DH_ReadByte( LIS3DH_OUT_ADC1_H, &output_8);
	LIS3DH.adc = output_8;

	LIS3DH_ReadByte( LIS3DH_OUT_ADC1_L, &output_8);
	LIS3DH.adc = LIS3DH.adc * 256 + output_8;	
}

uint8_t LIS3DH_getWhoAmI()
{
	uint8_t value;
	
	LIS3DH_ReadByte( LIS3DH_WHO_AM_I, &value); 

	return value;
}

void LIS3DH_getAll ( void )
{
	uint8_t output_8;
	uint16_t output;	
	
//------------------- X 
	LIS3DH_ReadByte( LIS3DH_OUT_X_H, &output_8);
	output = output_8;

	LIS3DH_ReadByte( LIS3DH_OUT_X_L, &output_8);
	output = output * 256 + output_8;

	LIS3DH.AccelX = LIS3DH_calcAccel (output);
	
//------------------- Y
	LIS3DH_ReadByte( LIS3DH_OUT_Y_H, &output_8);
	output = output_8;

	LIS3DH_ReadByte( LIS3DH_OUT_Y_L, &output_8);
	output = output * 256 + output_8;

	LIS3DH.AccelY = LIS3DH_calcAccel (output);
	
//------------------- Z
	LIS3DH_ReadByte( LIS3DH_OUT_Z_H, &output_8);
	output = output_8;

	LIS3DH_ReadByte( LIS3DH_OUT_Z_L, &output_8);
	output = output * 256 + output_8;

	LIS3DH.AccelZ = LIS3DH_calcAccel (output);
}


void LIS3DH_init ( void )
{
	uint8_t dataToWrite = 0;  //Temporary variable

	//Build TEMP_CFG_REG
	dataToWrite = 0; //Fresh Start
	dataToWrite = ((LIS3DH_settings.tempEnabled & 0x01) << 6) | ((LIS3DH_settings.adcEnabled & 0x01) << 7);

	//Now, write the patched together data
	LIS3DH_WriteByte( LIS3DH_TEMP_CFG_REG, dataToWrite);

	//Build CTRL_REG1
	dataToWrite = 0; //Fresh Start
	
	//  Convert ODR
	switch(LIS3DH_settings.accelSampleRate)
	{
		case 1:
		dataToWrite |= (0x01 << 4);
		break;
		case 10:
		dataToWrite |= (0x02 << 4);
		break;
		case 25:
		dataToWrite |= (0x03 << 4);
		break;
		case 50:
		dataToWrite |= (0x04 << 4);
		break;
		case 100:
		dataToWrite |= (0x05 << 4);
		break;
		case 200:
		dataToWrite |= (0x06 << 4);
		break;
		default:
		case 400:
		dataToWrite |= (0x07 << 4);
		break;
		case 1600:
		dataToWrite |= (0x08 << 4);
		break;
		case 5000:
		dataToWrite |= (0x09 << 4);
		break;
	}
	dataToWrite |= (LIS3DH_settings.zAccelEnabled & 0x01) << 2;
	dataToWrite |= (LIS3DH_settings.yAccelEnabled & 0x01) << 1;
	dataToWrite |= (LIS3DH_settings.xAccelEnabled & 0x01);
	//Now, write the patched together data
	LIS3DH_WriteByte(LIS3DH_CTRL_REG1, dataToWrite);

	//Build CTRL_REG4
	dataToWrite = 0; //Start Fresh!

	//  Convert scaling
	switch(LIS3DH_settings.accelRange)
	{
		case 2:
		dataToWrite |= (0x00 << 4);
		break;
		case 4:
		dataToWrite |= (0x01 << 4);
		break;
		case 8:
		dataToWrite |= (0x02 << 4);
		break;
		default:
		case 16:
		dataToWrite |= (0x03 << 4);
		break;
	}
	dataToWrite |= 0x80; //set block update
	dataToWrite |= 0x08; //set high resolution

	//Now, write the patched together data
	LIS3DH_WriteByte(LIS3DH_CTRL_REG4, dataToWrite);
	
	if (LIS3DH_settings.fifoEnabled)
	{
		LIS3DH_ReadByte ( LIS3DH_FIFO_CTRL_REG , &dataToWrite);
		
		dataToWrite &= 0x20;//clear all but bit 5
		dataToWrite |= (LIS3DH_settings.fifoMode & 0x03) << 6; // Apply mode. Can be 0x0,0x1,0x2,0x3
		dataToWrite |= (LIS3DH_settings.fifoThreshold & 0x1F ); // Apply threshold.
		
		LIS3DH_WriteByte ( LIS3DH_FIFO_CTRL_REG , dataToWrite );
		
		//Build CTRL_REG5
		LIS3DH_ReadByte(  LIS3DH_CTRL_REG5 , &dataToWrite ); //Start with existing data
		dataToWrite &= 0xBF; //clear bit 6
		dataToWrite |= 0x01 << 6;

		//Now, write the patched together data
		LIS3DH_WriteByte( LIS3DH_CTRL_REG5, dataToWrite);
	}
}

void LIS3DH_fifoClear ( void ) 
{
	//todo
}

void LIS3DH_fifoStartRec (void)
{
	uint8_t dataToWrite = 0;  //Temporary variable
	
	//Turn off...
	LIS3DH_ReadByte( LIS3DH_FIFO_CTRL_REG , &dataToWrite ); //Start with existing data
	dataToWrite &= 0x3F;//clear mode

	LIS3DH_WriteByte(LIS3DH_FIFO_CTRL_REG, dataToWrite);	

	//  ... then back on again
	LIS3DH_ReadByte( LIS3DH_FIFO_CTRL_REG , &dataToWrite); //Start with existing data
	dataToWrite &= 0x3F;//clear mode
	dataToWrite |= (LIS3DH_settings.fifoMode & 0x03) << 6; //Apply mode

	//Now, write the patched together data
	LIS3DH_WriteByte(LIS3DH_FIFO_CTRL_REG, dataToWrite);
}

static void LIS3DH_fifoEnd (void)
{
	uint8_t dataToWrite = 0;  //Temporary variable

	//Turn off...
	LIS3DH_ReadByte( LIS3DH_FIFO_CTRL_REG, &dataToWrite ); //Start with existing data
	dataToWrite &= 0x3F; //Clear mode

	LIS3DH_WriteByte(LIS3DH_FIFO_CTRL_REG, dataToWrite);	
}

void LIS3DH_getStatus ( void ) 
{
	//Get the status register
	LIS3DH_ReadByte(LIS3DH_STATUS_REG2, &LIS3DH.status );
}

void LIS3DH_getDefaultSettings(void)
{
	
	// Acceleration range and sample rate
	
	LIS3DH_settings.accelRange = 16;
	LIS3DH_settings.accelSampleRate = 200;

	LIS3DH_settings.xAccelEnabled = true;
	LIS3DH_settings.yAccelEnabled = true;
	LIS3DH_settings.zAccelEnabled = true;
	
	// Click and free fall detection
	LIS3DH_settings.clickEnabled = false ;
	LIS3DH_settings.dobleClickenabled  = false;
	LIS3DH_settings.freeFall = false;
	
	// Fifo mode
	LIS3DH_settings.fifoEnabled =  false;
	LIS3DH_settings.fifoMode =  0;
	LIS3DH_settings.fifoThreshold =  0;
	
	// Temperature
	LIS3DH_settings.tempEnabled = true ;
}