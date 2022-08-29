/******************** (C) COPYRIGHT 2020 Elemon ********************************
  File Name          : lis3dh.h
  Author             : Marcos Ciancio
  Version            : V1.2
  Date               : 2020/07
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

#include "stdint.h"

#ifndef __LIS3DH_H
#define __LIS3DH_H

// Macros
// Device address
#define LIS3DH_ADDRESS 				 0x19

// Datatypes

typedef enum { LIS3DH_OK = 0, LIS3DH_ERROR } LIS3DH_Status;

// Global variables

extern struct LIS3DH_settings_t LIS3DH_settings;
extern struct LIS3DH_t 			LIS3DH;

//This struct holds the settings the driver uses to do calculations
struct LIS3DH_settings_t
{
	//ADC and Temperature settings
	uint8_t adcEnabled;
	uint8_t tempEnabled;

	//Accelerometer settings
	uint16_t accelSampleRate;  	//Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
	uint8_t accelRange;      	//Max G force readable.  Can be: 2, 4, 8, 16

	uint8_t xAccelEnabled;
	uint8_t yAccelEnabled;
	uint8_t zAccelEnabled;
	
	//Fifo settings
	uint8_t fifoEnabled;
	uint8_t fifoMode; 			// It can be 0x0,0x1,0x2,0x3
	uint8_t fifoThreshold;		// It can be from 0 to 31 (1F)
	
	//Click control settings
	uint8_t clickEnabled;

	//Double Click control settings
	uint8_t dobleClickenabled;
	
	//Free-Fall control settings
	uint8_t freeFall;	
};

typedef struct {
	uint8_t IA;
	uint8_t DC;
	uint8_t SC;
	uint8_t S ;
	uint8_t axisZ ;
	uint8_t axisY ;
	uint8_t axisX ;
} CLICK_t;

//This struct holds the settings the driver uses to do calculations
struct LIS3DH_t
{
	double AccelX;  // X axis
	double AccelY; 	// Y
	double AccelZ; 	// Z
	double adc; 	// adc value
	double temp;    // temperature
	uint8_t status;

	CLICK_t click;
};

// Prototypes
void     LIS3DH_init ( void ) ;
void     LIS3DH_getDefaultSettings(void);
void     LIS3DH_fifoClear ( void ) ; // not used
void     LIS3DH_getStatus ( void ) ;
void 	 LIS3DH_fifoStartRec (void); // not used
void     LIS3DH_getAll ( void );
uint8_t  LIS3DH_getWhoAmI ( void );

#endif /* __LIS3DH_H */




