/*****************************************************************************
    @file    appSettings.h
    @brief   data
******************************************************************************

  file Name          : appSettings.h
  Author             : Marcos Ciancio
  Version            : V1.0
  Date               : 2021/12
  Description        : Configuration & various features implementation of appSettings
  Developed for      : Data
********************************************************************************


*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __APP_SETTINGS_H__
#define __APP_SETTINGS_H__

// Macros
#define TIME_DIVIDER 60000 // 60000

#define MAX_LENGHT_GPS_STREAM      100
#define MAX_LENGHT_GPS_STREAM_2    100
#define MAX_LENGHT_LIS3DH_STREAM   100
#define MAX_LENGHT_LoRa_STREAM     51  // 51 for worst data rate. And 222 for best data rate

// FW Version
#define VERSION_LOW 0
#define VERSION_HIGH 2

// Interfases

//======================================================
// SPI and UART/UART2 shares the same pin's
//======================================================
//#define SPI_ENABLE 

/*
// SPI
PIN6	MISO
PIN7	MOSI
PIN10	CSK
PIN11	SS

// UART
PIN13	TX
PIN11	RX

// UART 2
PIN10	RX
PIN6	TX
*/


#endif /* __APP_SETTINGS_H__ */
