/*
   app.c

   Created: 22/10/2019 1:44:19 p. m.
    Author: gmont_000
*/
#include "..\eer34.h"

#include "..\drivers\EER34_gpio.h"
#include "..\drivers\EER34_adc.h"
#include "..\drivers\EER34_i2c.h"

// SPI and UART/UART2 shares the same pin's
#include "..\drivers\EER34_spi.h"
#include "..\drivers\EER34_uart.h"

// Driver
#include "..\HW drivers\LIS3DH.h"

// Eeprom memory
#include "..\HW drivers\EEPROM_64.h"

// App settings
#include "appSettings.h"

// Private variables
static int timer1;    // Para EER34_tickCallback

// Stream for GPS
static char streamGPS[MAX_LENGHT_GPS_STREAM_2] = {0};  // {"0000.0000,S,00000.0000,W"}; // There is no data from GPS yet.

static float latitudeGPS = 0;
static float longitudeGPS = 0;

// Stream for LIS3DH
static char accelData [MAX_LENGHT_LIS3DH_STREAM] = {"x:-0.00y:-0.00z:0.00"}; // There is no data from ACCELEROMETER yet.

// Stream to send to server
static char dataToServer [MAX_LENGHT_LoRa_STREAM] = {0};

// char streamGPS_dummy [MAX_LENGHT_STREAM] = {"$GNRMC,154731.000,A,3434.2719,S,05829.1594,W,0.00,326.63,240122,,,D,V*02\r\n"};


static enum {
  APP_FSM_INIT = 0,
  APP_FSM_JOINING,
  APP_FSM_JOINFAILED,
  APP_FSM_JOINED,
  APP_FSM_TXWAIT,
  APP_FSM_TXOK,
  APP_FSM_TXERROR,
  APP_FSM_IDLE,
  APP_FSM_IDLE_TX,
  APP_FSM_SLEEP,
} fsm;

/**
    @brief  Callback de status para la aplicacion
    @param  sts     status de EER34
    @param  loraSts   status del stack LoRaWan
*/
void EER34_statusCallback(EER34_status_t sts, StackRetStatus_t LoraSts)
{
  if (sts == EER34_STATUS_JOIN_SUCCSESS) {
    if (fsm == APP_FSM_JOINING)
      fsm = APP_FSM_JOINED;
  }
  else if (sts == EER34_STATUS_JOIN_ERROR) {
    if (fsm == APP_FSM_JOINING)
      fsm = APP_FSM_JOINFAILED;
  }
  else if (sts == EER34_STATUS_TX_SUCCSESS) {
    if (fsm == APP_FSM_TXWAIT)
      fsm = APP_FSM_TXOK;
  }
  else if (sts == EER34_STATUS_TX_TIMEOUT) {
    if (fsm == APP_FSM_TXWAIT)
      fsm = APP_FSM_TXERROR;
  }
}

/**
    @brief  Callback de recepcion de datos la aplicacion
    @param  port    numero de puerto
    @param  data    puntero a los datos
    @param  len     cantidad de bytes de datos
*/
void EER34_rxDataCallback(int port, uint8_t *data, int len)
{
}

/**
    @brief Callback de entrada en low power

    Sirve para que la aplicacion deinicialice y/o apague lo que haga falta
    al entrar en el modo de bajo consumo.
*/
void EES34_enterLowPower(void)
{
  // Hay que detener el tick sino no entra en bajo consumo
  EER34_tickStop();
}

/**
    Callback de salida de low power

    Sirve para que la aplicacion reponga los recursos al despertar del modo
    de bajo consumo, volviendo a configurar y/o encender los recursos
    que deinicializo y/o apago al entrar en el modo de bajo consumo.
*/
void EES34_exitLowPower(void)
{
  // Vuelve a enceder el tick que lo apago al entrar en bajo consumo
  EER34_tickStart(10);
}

/**
    Funcion de inicializacion de la aplicacion
*/

void EES34_appInit(void)
{
  printf("\r\n========================\r\nEESAMR34\r\n========================\r\n");

  //============================================================
  // Initialize I2C
  //============================================================
  EER34_I2C_begin();

  //============================================================
  // Read the EUID
  //============================================================
  // The devEuix can be set by user
  uint8_t devEuix[8] = {0xDE, 0xAF, 0xFA, 0xCE, 0xDE, 0xAF, 0x55, 0x20};

  //EUID loaded from 24AA02E64-I/OT
  //EEPROM_64_read_EUI(EEPROM_64_FIRST_ADDRESS_EUI_64, devEuix); // The devEuix is overrided by 24AA02E64-I/OT placed on board

  printf ( "EUID : " );

  for (uint8_t i = 0; i < 8; i++)
    printf ( "%02X" , devEuix[i]);

  printf ( "\r\nEUID : " );

  for (uint8_t i = 0; i < 8; i++)
    printf ( "0x%02X " , devEuix[i]);

  printf ( "\r\n" );

  //============================================================
  // LoRa Credentials
  //============================================================
  uint8_t appEuix[] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11}; // formerly JoinEUI
  uint8_t appKeyx[] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};

  //============================================================
  // Initialize LoRa
  //============================================================
  static int res;

  // Este seteo debe ir primero porque inicia el stack LoRaWan
  res = EER34_setBand(ISM_AU915, 1);

  // Estos seteos pueden ir en cualquier orden pero siempre
  // despues de setear la banda (sino dan error)
  res = EER34_setDevEui(devEuix);
  res = EER34_setAppEui(appEuix);
  res = EER34_setAppKey(appKeyx);
  res = EER34_setDeviceClass(CLASS_A);

  EER34_tickStart(10);  // arranca tick de 10ms

  printf("Initialization Done\r\n");

  //============================================================
  // Initialize pins
  //============================================================
  EER34_Gpio_pinMode ( PIN_PA14 , OUTPUT ) ;      // LED 2
  EER34_Gpio_pinMode ( PIN_PA30 , OUTPUT ) ;      // SWDCLK function
  EER34_Gpio_pinMode ( PIN_PA07 , OUTPUT ) ;      // LED 5

  EER34_Gpio_pinMode ( PIN_PA27 , INPUT  );     // SW2 with external pull-up
  EER34_Gpio_pinMode ( PIN_PA22 , INPUT_PULLUP   ); // As input with internal pull-up
  EER34_Gpio_pinMode ( PIN_PA15 , INPUT_PULLDOWN ); // As input with internal pull-down
  EER34_Gpio_pinMode ( PIN_PA08 , INPUT ) ;     // As input floating

  // Initialize adc
  EER34_Adc_startAdc();

  //======================================================
  // SPI and UART/UART2 shares the same pin's
  //======================================================
  // Initialize SPI

#ifdef SPI_ENABLE
  EER34_configureSpiMaster(100000, SPI_DATA_ORDER_MSB, SPI_TRANSFER_MODE_0 );
#else
  //============================================================
  // Initialize UART
  //============================================================
  EER34_initUart();

  //============================================================
  // Initialize UART 2
  //============================================================
  EER34_initUart_2();

  //============================================================
  // Reset for BLE module
  //============================================================
  EER34_Gpio_pinMode ( PIN_PB22 , OUTPUT ) ;

  // Ensure reset BLE
  EER34_Gpio_digitalWrite (PIN_PB22, false );
  delay_cycles_ms(100); // delay
  EER34_Gpio_digitalWrite (PIN_PB22, true );

  //============================================================
  // Enable Energy and Reset for GPS
  //============================================================
  EER34_Gpio_pinMode ( PIN_PA18 , OUTPUT ) ;  // Energy control
  EER34_Gpio_pinMode ( PIN_PA19 , OUTPUT ) ;  // Reset line

  EER34_Gpio_digitalWrite (PIN_PA18, false ); // The Mosfet is on State when PA18 low.
  EER34_Gpio_digitalWrite (PIN_PA19, false ); // Q3 acts like inverter.
#endif

  //============================================================
  // LIS3DH
  //============================================================
  LIS3DH_getDefaultSettings();

  // Initialize LIS3DH
  LIS3DH_init();

  uint8_t myName = LIS3DH_getWhoAmI();

  if (myName == 0x33)
    printf ( "LIS3DH Detected\r\n" );
  //============================================================
}

/**
    @brief  Callback del tick de la aplicacion
*/
void EER34_tickCallback(void)
{
  if (timer1) timer1--;
}

static void ToggleLed2 (void) {
  static bool led2 = 0;

  led2 = led2 ^ 1;

  if ( led2 )  EER34_Gpio_digitalWrite (PIN_PA14, true );
  else EER34_Gpio_digitalWrite (PIN_PA14, false );
}

static void ToggleLed5 (void) {
  static bool led5 = 0;

  led5 = led5 ^ 1;

  if ( led5 )  EER34_Gpio_digitalWrite (PIN_PA07, true );
  else EER34_Gpio_digitalWrite (PIN_PA07, false );
}


static void TestSPI(void)
  {
  uint8_t buffer_spi_tx [ 20 ] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13 };
  uint8_t buffer_spi_rx [ 20 ] = { 0x00 } ;

  memcpy_ram2ram ( buffer_spi_rx , buffer_spi_tx, 20 ) ;

  EER34_spiStartTransaction ( ) ;
  EER34_spiTransferBuffer   ( buffer_spi_tx , sizeof ( buffer_spi_tx) );
  EER34_spiEndTransaction   ( );

  if (memcmp_ram2ram ( buffer_spi_tx , buffer_spi_rx , 20 ) == 0  ) {
    ToggleLed5();
  }
  }

static bool trimLeft(char *stream, char i, bool inc)
{
  char *pStart;

  if (strlen(stream) == 0)
    return false;

  pStart = strchr(stream, i);

  if (pStart == NULL)
    return false;

  if (inc)
  {
    memmove(stream, pStart, (stream + strlen(stream) + 1) - pStart);
    memset(pStart + strlen(stream) + 1, 0, 1);
  }
  else
  {
    memmove(stream, pStart + 1, (stream + strlen(stream)) - pStart);
    memset(pStart + strlen(stream), 0, 1);
  }
  return true;
}

static bool trimBetween(char *stream, char i, bool incF, char j, bool incE)
{
  char *pStart;
  char *pEnd;

  if (strlen(stream) == 0)
    return false;

  pStart = strchr(stream, i);
  if (pStart == NULL)
    return false;

  pEnd = strchr(pStart + 1, j);
  if (pEnd == NULL)
    return false;

  if ((incF == true) && (incE == true))
  {
    memmove(stream, pStart, (pEnd - pStart) + 1);
    memset(stream + (pEnd - pStart) + 1, 0, 1);
  }

  if ((incF == false) && (incE == true))
  {
    memmove(stream, pStart + 1, (pEnd - pStart));
    memset(stream + (pEnd - pStart), 0, 1);
  }

  if ((incF == true) && (incE == false))
  {
    memmove(stream, pStart, (pEnd - pStart));
    memset(stream + (pEnd - pStart), 0, 1);
  }

  if ((incF == false) && (incE == false))
  {
    memmove(stream, pStart + 1, (pEnd - pStart) - 1);
    memset(stream + (pEnd - pStart) - 1, 0, 1);
  }
  return true;
}


static bool isGNRMCstream(char *caracter, uint8_t len)
{
  if ( ( caracter[1] == 'G') && ( caracter[2] == 'N') && ( caracter[3] == 'R') && ( caracter[4] == 'M') && ( caracter[5] == 'C') )
    return true;
  else
    return false;
}

static void parseGps(uint16_t caracter)
{

  static char stream [MAX_LENGHT_GPS_STREAM] = {0};
  static uint8_t index = 0;

#define START_CHAR '$'         // $
#define END_CHAR 10            // Line feed

  if (caracter > 255)
    return;

  switch (caracter)
  {
    case START_CHAR:
      index = 0;
      memset(stream, 0, MAX_LENGHT_GPS_STREAM );
      stream[index++] = (char) caracter;
      break;
    case END_CHAR:
      if ( isGNRMCstream ( stream, MAX_LENGHT_GPS_STREAM ) == true)
      {
        trimLeft( stream, ',', false);
        trimLeft( stream, ',', false);
        trimBetween(stream, 'A', false, 'W', true);
        trimLeft( stream, ',', false);

        //printf ( "STREAM GPS : %s \r\n" , stream );

        latitudeGPS = ( ( (float) ( (uint8_t) stream[0] - 48 ) ) *     10) + \
                      ( ( (float) ( (uint8_t) stream[1] - 48 ) ) *      1) + \
                      ( ( (float) ( (uint8_t) stream[2] - 48 ) ) /      6) + \
                      ( ( (float) ( (uint8_t) stream[3] - 48 ) ) /     60) + \
                      ( ( (float) ( (uint8_t) stream[5] - 48 ) ) /    600) + \
                      ( ( (float) ( (uint8_t) stream[6] - 48 ) ) /   6000) + \
                      ( ( (float) ( (uint8_t) stream[7] - 48 ) ) /  60000) + \
                      ( ( (float) ( (uint8_t) stream[8] - 48 ) ) / 600000);

        if (stream[10] == 'S' )
          latitudeGPS  = (-1) * latitudeGPS;

        longitudeGPS = ( ( (float) ( (uint8_t) stream[12] - 48 ) ) *    100) + \
                       ( ( (float) ( (uint8_t) stream[13] - 48 ) ) *     10) + \
                       ( ( (float) ( (uint8_t) stream[14] - 48 ) ) *      1) + \
                       ( ( (float) ( (uint8_t) stream[15] - 48 ) ) /      6) + \
                       ( ( (float) ( (uint8_t) stream[16] - 48 ) ) /     60) + \
                       ( ( (float) ( (uint8_t) stream[18] - 48 ) ) /    600) + \
                       ( ( (float) ( (uint8_t) stream[19] - 48 ) ) /   6000) + \
                       ( ( (float) ( (uint8_t) stream[20] - 48 ) ) /  60000) + \
                       ( ( (float) ( (uint8_t) stream[21] - 48 ) ) / 600000);

        if (stream[23] == 'W' )
          longitudeGPS  = (-1) * longitudeGPS;

        snprintf(streamGPS, sizeof(streamGPS), "%+010ld%+010ld" , (uint32_t)(latitudeGPS * 1000000) , (uint32_t)(longitudeGPS * 1000000 ) );
      }
      break;
    default:
      if (index < MAX_LENGHT_GPS_STREAM)
        stream[index++] = (char) caracter;
      break;
  }
}

//============================================================
// TEST the BLE ECHO.
// This task is always hearing the data arriving from BLE module and send this data back to the module
//============================================================
static void bleECHO(void)
{
  uint16_t character;

  if ( usart_read_wait(&usart_instance, &character) == STATUS_OK)
  {
    printf ( "Serial data from BLE :%c", character );

    // Echo
    uint8_t sendCharacter;
    sendCharacter = (uint8_t) character;
    usart_write_buffer_wait(&usart_instance, &sendCharacter, 1);

    while ( usart_read_wait(&usart_instance, &character) == STATUS_OK)
    {
      printf ( "%c", character );

      sendCharacter = (uint8_t) character;
      usart_write_buffer_wait(&usart_instance, &sendCharacter, 1);
    }
    printf ( "\r\n" );
  }
}

/**
    Callback de procesamiento de la causa de reset
*/
void EES34_appResetCallback(unsigned int rcause)
{
  printf("Last reset cause: ");
  if (rcause & (1 << 6)) {
    printf("System Reset Request\r\n");
  }
  if (rcause & (1 << 5)) {
    printf("Watchdog Reset\r\n");
  }
  if (rcause & (1 << 4)) {
    printf("External Reset\r\n");
  }
  if (rcause & (1 << 2)) {
    printf("Brown Out 33 Detector Reset\r\n");
  }
  if (rcause & (1 << 1)) {
    printf("Brown Out 12 Detector Reset\r\n");
  }
  if (rcause & (1 << 0)) {
    printf("Power-On Reset\r\n");
  }
}


static void resetBLE(void)
{
	EER34_Gpio_digitalWrite (PIN_PB22, true);
}

static void turnOnBLE(void)
{
	EER34_Gpio_digitalWrite (PIN_PB22, false);
}

/**
    Task de la aplicación
*/
void EES34_appTask(void)
{
  // Test BLE ECHO
  bleECHO();

  // TIME DIVISION
  static uint32_t divider = 0;

  divider++;
  if ( divider == TIME_DIVIDER )
  {
    divider = 0;

    // Get Accelerometer values
    LIS3DH_getAll();

    // Print
    printf ( "LIS3DH X: %2.2f Y: %2.2f Z : %2.2f\r\n", LIS3DH.AccelX, LIS3DH.AccelY, LIS3DH.AccelZ );
    snprintf( accelData, sizeof(accelData), "x:%2.2fy:%2.2fz:%2.2f", LIS3DH.AccelX, LIS3DH.AccelY, LIS3DH.AccelZ );

    // Get GPS values
    memset(dataToServer, 0, sizeof(dataToServer));

    strcat (dataToServer,  streamGPS);

    /*strcat (dataToServer,  streamGPS);
      strcat (dataToServer,  accelData);*/

    printf ( "GPS parsed : %s\r\n", dataToServer );

    // Check if SW2 is pressed ( Near to antenna ). This task is intended to test the serial input of GPS module.This test the BUTTON, ADC, and LED red ( near to USB )
    if ( EER34_Gpio_digitalRead ( PIN_PA27 ) == 0 ) {
      // Test GPS command
      printf ( "\r\nCommand PPS\r\n");
      uint8_t streamToSend [] = "$PQ1PPS,W,4,100*1D\r\n";
      printf("Stream to send : %s", streamToSend);
      usart_write_buffer_wait(&usart_instance_2, streamToSend, sizeof(streamToSend) );

      // Toggle led
      static bool toggleFlag = false ;

      toggleFlag = toggleFlag ^ 1;
      if ( toggleFlag )  EER34_Gpio_digitalWrite ( PIN_PA30 , true ) ;
      else EER34_Gpio_digitalWrite ( PIN_PA30 , false ) ;

      ToggleLed2();

#ifdef SPI_ENABLE
      TestSPI();      // SPI and UART/UART2 shares the same pin's
#endif

      uint16_t value = EER34_Adc_digitalRead ();
      printf ( "ADC: %d\r\n", value );
    }
  }
  //===================================
  // PB1 switch (pin named as PIN_PA08)
  // Button on the corner of the PCB SHIELD.
  //===================================
 // while ( EER34_Gpio_digitalRead ( PIN_PA08 ) == 0 )
//  {
    // GPS uart received
     uint16_t character;
      while ( usart_read_wait(&usart_instance_2, &character) == STATUS_OK)
       parseGps(character); 
 // }

  //===================================
  // MACHINE STATE for LoRa communication
  //===================================
  static int count;
  uint8_t lenght;

  switch (fsm) {
    case APP_FSM_JOINFAILED:
      printf("Join failed\r\n");
    case APP_FSM_INIT:
      printf("\r\nSending join request\r\n");
      EER34_joinOTAA();
      fsm = APP_FSM_JOINING;
      break;
    case APP_FSM_JOINING:
      //Nothing to do, wait.
      break;
    case APP_FSM_JOINED:
      count = 1;
      printf("Joined\r\n");

      lenght = strlen( (char *) dataToServer );

      // If joined, then the device transmit with no confirmation
      if (EER34_tx(EER34_TXMODE_UNCONF, 1, (uint8_t *)  dataToServer, lenght)) {
        printf("Transmitting ...\r\n");
        fsm = APP_FSM_TXWAIT;
      }
      else { // Error on transmission.
        printf("Transmit Error\r\n");
        timer1 = 500;
        fsm = APP_FSM_IDLE;
      }
      break;
    case APP_FSM_TXWAIT:
      break;
    case APP_FSM_TXOK:
      printf("Transmit OK\r\n");
      timer1 = 500;
      fsm = APP_FSM_IDLE;
      break;
    case APP_FSM_TXERROR:
      printf("Transmit Timeout\r\n");
      timer1 = 500;
      fsm = APP_FSM_IDLE;
      break;
    case APP_FSM_IDLE:
      if (count % 3 == 0) { // times to transmit
        printf("Going to sleep ...\r\n");
        timer1 = 0;
        fsm = APP_FSM_SLEEP;
      }
      else
        fsm = APP_FSM_IDLE_TX;
      break;
    case APP_FSM_IDLE_TX:
      if (!timer1) {

        lenght = strlen( (char *) dataToServer );

        if (EER34_tx(EER34_TXMODE_UNCONF, 1, (uint8_t *) dataToServer, lenght)) {
          count++;
          printf("Transmitting ...\r\n");
          fsm = APP_FSM_TXWAIT;
        }
        else
          printf("Transmit Error\r\n");
        timer1 = 500;
      }
      break;
    case APP_FSM_SLEEP:
      if (!timer1) {
        if (EER34_sleep(25000)) {
     		printf("Slept OK, woken-up!\r\n\r\n");
          fsm = APP_FSM_IDLE_TX;
        }
        else {
          printf("Sleep failed\r\n");
          timer1 = 50;
        }
      }
      break;
    default:
      break;
  }
}
