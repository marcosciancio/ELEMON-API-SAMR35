/******************** (C) COPYRIGHT 2020 Elemon ********************************
  File Name          : MEM_24AA02E64.c
  Author             : Marcos Ciancio
  Version            : V1.2
  Date               : 2021/12
  Description        : Configuration & various feature implementation of MEM_24AA02E64
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
#include "EEPROM_64.h"
#include "I2C_custom.h"
#include "stdio.h"

#define BIN(b7,b6,b5,b4,b3,b2,b1,b0) ((b0)+((b1)<<1)+((b2)<<2)+((b3)<<3)+((b4)<<4)+((b5)<<5)+((b6)<<6)+((b7)<<7))

/* 8-bit conversion function */
#define B8_(x) ((x&0x0000000FLU)?1:0) +((x&0x000000F0LU)?2:0)+((x&0x00000F00LU)?4:0)+((x&0x0000F000LU)?8:0)+((x&0x000F0000LU)?16:0)+((x&0x00F00000LU)?32:0)+((x&0x0F000000LU)?64:0)+((x&0xF0000000LU)?128:0)

//Macros
// Datatypes
// Variables
// Prototypes

EEPROM_64_Status EEPROM_64_write(uint8_t addr, uint8_t value)
{
  I2Ccustom_Status res;

  res = I2C_customWriteStream(EEPROM_64_ADDRESS , 1 , addr , &value);

  if (res != I2C_CUSTOM_OK) return EEPROM_64_ERROR ;

	return EEPROM_64_OK ;
}

EEPROM_64_Status EEPROM_64_read(uint8_t addr,  uint8_t *value)
{
	I2Ccustom_Status res;
	
	res = I2C_customRead ( EEPROM_64_ADDRESS , 1 , addr , value );
	if ( res != I2C_CUSTOM_OK ) return EEPROM_64_ERROR ;

	return EEPROM_64_OK ;
}

EEPROM_64_Status EEPROM_64_read_EUI (uint8_t Addr, uint8_t *data)
{
	
	I2Ccustom_Status res;
	
	res = I2C_customRead ( EEPROM_64_ADDRESS , 8 , Addr , data );
	
	if ( res != I2C_CUSTOM_OK ) return EEPROM_64_ERROR ;
	return EEPROM_64_OK ;	
}