/*
   Electronica Elemon SA

   EER34_uart.c

   Created: 15/11/2021
    Author: mciancio
*/

#include "EER34_uart.h"
#include "EER34_gpio.h"

/**
    @brief
    @param
    @param
    @return
*/
void EER34_initUart  ( void)
{
  struct usart_config config_usart;
  usart_get_config_defaults(&config_usart);

  config_usart.baudrate = 115200;

  config_usart.mux_setting = USART_RX_3_TX_0_XCK_1;

  config_usart.pinmux_pad0 = PINMUX_PB02D_SERCOM5_PAD0;
  config_usart.pinmux_pad1 = PINMUX_UNUSED;
  config_usart.pinmux_pad2 = PINMUX_UNUSED;
  config_usart.pinmux_pad3 = PINMUX_PB23D_SERCOM5_PAD3;

  while (usart_init(&usart_instance,  SERCOM5, &config_usart) != STATUS_OK) {
  }

  usart_enable(&usart_instance);
}

void EER34_initUart_2 ( void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	config_usart.baudrate = 9600;

	config_usart.mux_setting = USART_RX_1_TX_0_XCK_1;

	config_usart.pinmux_pad0 = PINMUX_PA22C_SERCOM3_PAD0; // tx
	config_usart.pinmux_pad1 = PINMUX_PA23C_SERCOM3_PAD1; // rx
	config_usart.pinmux_pad2 = PINMUX_UNUSED;
	config_usart.pinmux_pad3 = PINMUX_UNUSED;

	while (usart_init(&usart_instance_2,  SERCOM3, &config_usart) != STATUS_OK) {
	}

	usart_enable(&usart_instance_2);
}


