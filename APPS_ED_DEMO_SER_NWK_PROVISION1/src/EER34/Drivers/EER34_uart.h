/*
 * Electronica Elemon SA
 *
 * EER43_uart.h
 *
 * Created: 15/11/2021
 *  Author: mciancio
 */ 
 
#ifndef ___UART_H_
#define ___UART_H_

#include <port.h>
#include <sercom.h>
#include <usart.h>
#include "samr34_xplained_pro.h"

struct usart_module usart_instance;
struct usart_module usart_instance_2;

// Macros

// Tipos de dato

// Variables publicas

// Prototipes
void EER34_initUart (void) ;
void EER34_initUart_2 (void) ;

#endif /* ___UART_H_ */
