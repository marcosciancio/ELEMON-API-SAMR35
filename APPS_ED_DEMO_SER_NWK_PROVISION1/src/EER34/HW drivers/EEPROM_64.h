/******************** (C) COPYRIGHT 2020 Elemon ********************************
  File Name          : EEPROM_64.h
  Author             : Marcos Ciancio
  Version            : V1.2
  Date               : 2020/07
  Description        : Configuration & various feature implementation of EEPROM_64
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

#ifndef __EEPROM_64_H
#define __EEPROM_64_H

// Includes
#include "EEPROM_64.h"
#include "stdint.h"

// Macros

// Device address
#define EEPROM_64_ADDRESS 				 0x50  // 1010xxx last 3 xxx = 0 ( version without pin address)

// Device Registers
#define EEPROM_64_FIRST_ADDRESS_EUI_64   0xF8

// Datatypes
typedef enum { EEPROM_64_OK = 0, EEPROM_64_ERROR } EEPROM_64_Status;

// Global variables
// Variables
// Prototypes

EEPROM_64_Status EEPROM_64_write(uint8_t addr, uint8_t value);
EEPROM_64_Status EEPROM_64_read (uint8_t Addr, uint8_t *value);
EEPROM_64_Status EEPROM_64_read_EUI (uint8_t Addr, uint8_t *data);

#endif /* __EEPROM_64_H */




