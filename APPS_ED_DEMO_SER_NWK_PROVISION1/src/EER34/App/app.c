/*
 * app.c
 *
 * Created: 22/10/2019 1:44:19 p. m.
 *  Author: gmont_000
 */ 
#include "..\eer34.h"

#include "..\Drivers\EER34_gpio.h"
#include "..\Drivers\EER34_adc.h"
#include "..\Drivers\EER34_spi.h"
#include "..\Drivers\EER34_i2c.h"

// Private variables

int timer1;		// Para EER34_tickCallback

static enum {
	APP_FSM_INIT = 0,
	APP_FSM_JOINING,
	APP_FSM_JOINFAILED,
	APP_FSM_JOINED,
	APP_FSM_TXWAIT,
	APP_FSM_TXOK,
	APP_FSM_TXERROR,
	APP_FSM_TXTIMEOUT,
	APP_FSM_IDLE,
	APP_FSM_IDLE_TX,
	APP_FSM_SLEEP,
} fsm;

/** 
 *	@brief	Callback de status para la aplicacion
 *	@param	sts			status de EER34
 *	@param	loraSts		status del stack LoRaWan
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
			fsm = APP_FSM_TXTIMEOUT;
	}
	else if (sts == EER34_STATUS_TX_ERROR) {
		printf("TX Error: %d\r\n", LoraSts);
		if (fsm == APP_FSM_TXWAIT)
			fsm = APP_FSM_TXERROR;
	}
}

/** 
 *	@brief	Callback de recepcion de datos la aplicacion
 *	@param	port		numero de puerto
 *	@param	data		puntero a los datos
 *	@param	len			cantidad de bytes de datos
 */
void EER34_rxDataCallback(int port, uint8_t *data, int len)
{
}

/** 
 *	@brief Callback de entrada en low power
 *
 *  Sirve para que la aplicacion deinicialice y/o apague lo que haga falta
 *  al entrar en el modo de bajo consumo.
 */
void EES34_enterLowPower(void)
{
	// Hay que detener el tick sino no entra en bajo consumo
	EER34_tickStop();
}

/** 
 *	Callback de salida de low power
 *
 *  Sirve para que la aplicacion reponga los recursos al despertar del modo 
 *  de bajo consumo, volviendo a configurar y/o encender los recursos
 *  que deinicializo y/o apago al entrar en el modo de bajo consumo.
 */
void EES34_exitLowPower(void)
{
	// Vuelve a enceder el tick que lo apago al entrar en bajo consumo
	EER34_tickStart(10);
}

/** 
 *	Funcion de inicializacion de la aplicacion
 */

void EES34_appInit(void)
{
	static volatile int res;
	
	uint8_t devEuix[] = {0xDD, 0xAD, 0xFA, 0xCE, 0xDE, 0xAF, 0x55, 0x20};
	uint8_t appEuix[] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
	uint8_t appKeyx[] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
		0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
		
	printf("\r\n\r\nEESAMR34\r\nInitializing\r\n");

	// Este seteo debe ir primero porque inicia el stack LoRaWan
	res = EER34_setBand(ISM_AU915, 1);
//	res = EER34_setBand(ISM_AU915, 0);

	// Estos seteos pueden ir en cualqueir orden pero siempre
	// despues de setear la banda (sino dan error)
	res = EER34_setDevEui(devEuix);
	res = EER34_setAppEui(appEuix);
	res = EER34_setAppKey(appKeyx);
	res = EER34_setDeviceClass(CLASS_A);
//	res = EER34_setAdr(EER34_ADR_ON);
//	res = EER34_setAdr(EER34_ADR_OFF);
	
	// Arranca tick de 10ms
	EER34_tickStart(10);	// arranca tick de 10ms

	printf("Initialization Done\r\n\r\n");
	
	//============================================================
	// Initialize pins
		
	EER34_Gpio_pinMode ( PIN_PA14 , OUTPUT ) ;			// LED 2
	EER34_Gpio_pinMode ( PIN_PA18 , OUTPUT ) ;
	EER34_Gpio_pinMode ( PIN_PA30 , OUTPUT ) ;
	EER34_Gpio_pinMode ( PIN_PA19 , OUTPUT ) ;
	EER34_Gpio_pinMode ( PIN_PA07 , OUTPUT ) ;			// LED 5	

	EER34_Gpio_pinMode ( PIN_PA27 , INPUT  );			// SW2 with external pull-up
	EER34_Gpio_pinMode ( PIN_PA22 , INPUT_PULLUP   );	// As input with internal pull-up	
	EER34_Gpio_pinMode ( PIN_PA15 , INPUT_PULLDOWN );	// As input with internal pull-down		
	EER34_Gpio_pinMode ( PIN_PA08 , INPUT ) ;			// As input floating


	// Initialize adc
	EER34_Adc_startAdc();

//	// Inilialize SPI
	EER34_configureSpiMaster(100000, SPI_DATA_ORDER_MSB, SPI_TRANSFER_MODE_0 );

	// Initialize I2C
	EER34_I2C_begin();
	
	//============================================================
}

/** 
 *	@brief	Callback del tick de la aplicacion
 */
void EER34_tickCallback(void)
{
	if (timer1)	timer1--;
}

static void ToggleLed2 (void){
	static bool led2 = 0;
	
	led2 = led2 ^ 1;
	
	if ( led2 )  EER34_Gpio_digitalWrite (PIN_PA14, true );
		else EER34_Gpio_digitalWrite (PIN_PA14, false );
}

static void ToggleLed5 (void){
	static bool led5 = 0;
	
	led5 = led5 ^ 1;
	
	if ( led5 )  EER34_Gpio_digitalWrite (PIN_PA07, true );
	else EER34_Gpio_digitalWrite (PIN_PA07, false );
}

static void Set_All_Pins (bool val)
{
	EER34_Gpio_digitalWrite ( PIN_PA18 , true ) ;
	EER34_Gpio_digitalWrite ( PIN_PA30 , true ) ;
	EER34_Gpio_digitalWrite ( PIN_PA19 , true ) ;	
}

static void TestSPI(void)
{
	uint8_t buffer_spi_tx [ 20 ] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,	0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13 };
	uint8_t buffer_spi_rx [ 20 ] = { 0x00 } ;
	
	memcpy_ram2ram ( buffer_spi_rx , buffer_spi_tx, 20 ) ;
	
	EER34_spiStartTransaction ( ) ;
	EER34_spiTransferBuffer   ( buffer_spi_tx , sizeof ( buffer_spi_tx) );
	EER34_spiEndTransaction   ( );

	if (memcmp_ram2ram ( buffer_spi_tx , buffer_spi_rx , 20 ) == 0  ){
		ToggleLed5();
	}
}

void TestI2C(void)
{
#define SLAVE_ADDRESS				0x28

	uint8_t buffer_i2c_tx [ 20 ] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,	0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13 };
	uint8_t buffer_i2c_rx [ 20 ] = { 0 };
	
	EER34_I2C_Write ( SLAVE_ADDRESS , buffer_i2c_tx , 20);

	EER34_I2C_Read  ( SLAVE_ADDRESS , buffer_i2c_rx , 20);
}


/** 
 *	Task de la aplicacion
 */
void EES34_appTask(void)
{
	unsigned char data[] = {"ABCD"};
	static int count;
	
	static bool toggleFlag = 0 ;

	static int divider = 0;

/*
	divider++;
	if ( divider == 5000 )
	{
		divider = 0;
		
		if ( EER34_Gpio_digitalRead ( PIN_PA27 ) == 0 ) {
			
			toggleFlag = toggleFlag ^ 1;
			if ( toggleFlag )  Set_All_Pins ( true );
				else Set_All_Pins ( false );

				ToggleLed2();
				TestSPI();
				TestI2C();
			}
			
			uint16_t value = EER34_Adc_digitalRead ();
			printf ( "ADC: %d\r\n", value );
	}
*/
	switch(fsm) {
	case APP_FSM_JOINFAILED:
		printf("Join failed\r\n\r\n");
	case APP_FSM_INIT:
		printf("Sending join request\r\n");
		EER34_joinOTAA();
		fsm = APP_FSM_JOINING;
		break;
	case APP_FSM_JOINING:
		break;
	case APP_FSM_JOINED:
		count = 1;
		printf("Joined\r\n\r\n");
		if (EER34_tx(EER34_TXMODE_UNCONF, 1, data, 4)) {
			printf("Transmitting ...\r\n");
			fsm = APP_FSM_TXWAIT;
		}
		else {
			printf("Transmit Error\r\n");
			timer1 = 500;
			fsm = APP_FSM_IDLE;
		}
		break;
	case APP_FSM_TXWAIT:
		break;
	case APP_FSM_TXOK:
		printf("Transmit OK\r\n");
		timer1 = 1000;
		fsm = APP_FSM_IDLE;
		break;
	case APP_FSM_TXTIMEOUT:
		printf("Transmit Timeout\r\n");
		timer1 = 500;
		fsm = APP_FSM_IDLE;
		break;
	case APP_FSM_TXERROR:
		printf("Transmit Failed\r\n");
		timer1 = 500;
		fsm = APP_FSM_IDLE;
		break;
	case APP_FSM_IDLE:
		if (count % 3 == 0) {
			printf("\r\nGoing to sleep ...\r\n");
			timer1 = 0;
			fsm = APP_FSM_SLEEP;
		}
		else
			fsm = APP_FSM_IDLE_TX;
		break;
	case APP_FSM_IDLE_TX:
		if (!timer1) {
			if (EER34_tx(EER34_TXMODE_CONF, 1, data, 4)) {
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
			if (EER34_sleep(15000)) {
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

/** 
 *	Callback de procesamiento de la causa de reset
 */
void EES34_appResetCallback(unsigned int rcause)
{
	printf("Last reset cause: ");
	if(rcause & (1 << 6)) {
		printf("System Reset Request\r\n");
	}
	if(rcause & (1 << 5)) {
		printf("Watchdog Reset\r\n");
	}
	if(rcause & (1 << 4)) {
		printf("External Reset\r\n");
	}
	if(rcause & (1 << 2)) {
		printf("Brown Out 33 Detector Reset\r\n");
	}
	if(rcause & (1 << 1)) {
		printf("Brown Out 12 Detector Reset\r\n");
	}
	if(rcause & (1 << 0)) {
		printf("Power-On Reset\r\n");
	}
}