/*****************************************************************************
    @file    I2C_customs.h
    @brief   data
******************************************************************************

  File Name          : I2C_customs.h
  Author             : Marcos Ciancio
  Version            : V1.0
  Date               : 2021/06
  Description        : Configuration & various features implementation of I2C_customs
  Developed for      : Data
********************************************************************************


*******************************************************************************/

// Define to prevent recursive inclusion

#include "stdint.h"

typedef enum { I2C_CUSTOM_OK = 0, I2C_CUSTOM_ERROR } I2Ccustom_Status;

I2Ccustom_Status I2C_custom_init       ( void );

I2Ccustom_Status I2C_customWriteStream ( uint8_t SlaveAddress , uint8_t ByteCount    , uint8_t RegisterAddr , uint8_t * TxBuffer);
I2Ccustom_Status I2C_customWriteByte   ( uint8_t SlaveAddress , uint8_t RegisterAddr , uint8_t value);

I2Ccustom_Status I2C_customWrite32Byte (uint8_t SlaveAddress  , uint8_t ByteCount    , uint8_t RegisterAddr, uint8_t *TxBuffer);

I2Ccustom_Status I2C_customRead        ( uint8_t SlaveAddress , uint8_t ByteCount    , uint8_t RegisterAddr , uint8_t * RxBuffer);
I2Ccustom_Status I2C_customReadByte    ( uint8_t SlaveAddress , uint8_t RegisterAddr , uint8_t *value) ;


