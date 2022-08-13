#ifndef SERIAL_H
#define	SERIAL_H
#include "stm32f10x.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "stm32f10x.h"

/* Demo application includes. */
#include "serial.h"
#include "bsp_usart.h"

#define			xComPortHandle					USART_TypeDef*
#define			vUARTInterruptHandler		USART1_IRQHandler

xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength );
signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime );
void vSerialPutString( xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength );
signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, TickType_t xBlockTime );




#endif	/*SERIAL_H*/

