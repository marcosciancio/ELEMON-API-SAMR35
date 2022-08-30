/*****************************************************************************
    @file    I2C_customs.c
    @brief   data
******************************************************************************

  file Name          : I2C_customs.c
  Author             : Marcos Ciancio
  Version            : V1.0
  Date               : 2021/06
  Description        : Configuration & various features implementation of I2C_customs
  Developed for      : Data
********************************************************************************


*******************************************************************************/

// Includes
#include "I2C_custom.h"

#include "EER34_i2c.h"
// Macros

// Functions

I2Ccustom_Status I2C_custom_init(void) {
	EER34_I2C_begin();

	return I2C_CUSTOM_OK;
}

I2Ccustom_Status I2C_customWriteByte(uint8_t SlaveAddress, uint8_t RegisterAddr, uint8_t value) {
	
	uint8_t arrayToSend[2];

	// Prepare array to send
	arrayToSend[0] = RegisterAddr;
	arrayToSend[1] = value;	

	EER34_I2C_Write(   SlaveAddress , arrayToSend , 2 ) ;
	
	return I2C_CUSTOM_OK; // todo
}

I2Ccustom_Status I2C_customWrite32Byte(uint8_t SlaveAddress, uint8_t ByteCount, uint8_t RegisterAddr, uint8_t *TxBuffer) {

	if (ByteCount > 4)
		return I2C_CUSTOM_ERROR;

	uint8_t arrayToSend [5];

	arrayToSend[0] = RegisterAddr;
	arrayToSend[1] = TxBuffer[0];
	arrayToSend[2] = TxBuffer[1];
	arrayToSend[3] = TxBuffer[2];
	arrayToSend[4] = TxBuffer[3];

	EER34_I2C_Write(   SlaveAddress , arrayToSend , 5 ) ;  

	//  return I2C_CUSTOM_ERROR;
	return I2C_CUSTOM_OK; // todo
}

I2Ccustom_Status I2C_customWriteStream(uint8_t SlaveAddress, uint8_t ByteCount, uint8_t RegisterAddr, uint8_t *TxBuffer) {
#define MAX_STREAM_QTY 20

  if (ByteCount > MAX_STREAM_QTY)
    return I2C_CUSTOM_ERROR;

  uint8_t arrayToSend[MAX_STREAM_QTY + 1]={0};

  arrayToSend[0] = RegisterAddr;

  for (uint8_t i = 0; i < ByteCount; i++)
    arrayToSend[i + 1] = TxBuffer[i];

  EER34_I2C_Write( SlaveAddress , arrayToSend , ByteCount + 1 ) ;    

  return I2C_CUSTOM_OK;
}

I2Ccustom_Status I2C_customReadByte(uint8_t SlaveAddress, uint8_t RegisterAddr, uint8_t *value) {

	uint8_t arrayToSend[2];
	
	arrayToSend[0] = RegisterAddr;	
	arrayToSend[1] = SlaveAddress;	


	EER34_I2C_Write( SlaveAddress , arrayToSend , 1) ;    

	EER34_I2C_Read ( SlaveAddress , value , 1 ) ;    

  return I2C_CUSTOM_OK;
}

I2Ccustom_Status I2C_customRead(uint8_t SlaveAddress, uint8_t ByteCount, uint8_t RegisterAddr, uint8_t *RxBuffer) {
	uint8_t arrayToSend[MAX_STREAM_QTY]={0};
	
	if (ByteCount > MAX_STREAM_QTY)
		return I2C_CUSTOM_ERROR;

	arrayToSend[0] = RegisterAddr;
	arrayToSend[1] = SlaveAddress;


	EER34_I2C_Write( SlaveAddress , arrayToSend , 1) ; // Transmite SLAVE_ADDRESS + REGISTER_ADDRESS + SLAVE_ADDRESS

	EER34_I2C_Read ( SlaveAddress , RxBuffer , ByteCount ) ;

	return I2C_CUSTOM_OK;
}


/******************* (C) COPYRIGHT 2021 ***** END OF FILE ****/