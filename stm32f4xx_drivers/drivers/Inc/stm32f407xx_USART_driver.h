/*
 * stm32f407xx_USART_driver.h
 *
 *  Created on: 11-Dec-2024
 *      Author: vibhu
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_



#endif /* INC_STM32F407XX_USART_DRIVER_H_ */

#include <stm32f407xx.h>







//USART Config structure includes configurable items configured by application code

typedef struct
{
	uint8_t USART_Mode; // can be in just tx or just rx or both
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits; // can be 1  , 1.5 , 2 basically defines the bit duration for which stop bits are there
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

//Handle structure

typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
}USART_Handle_t;






