/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : usart.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/24
* Description        : This file contains the defines used for USART fuctions
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H
#define __USART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_type.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define USART_DXL			0
#define USART_ZIGBEE		1
#define USART_IR			1
#define USART_PC			2

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void TxDData(u8 PORT, u8 dat);
u8 RxDBuffer(u8 PORT);
void BufferClear(u8 PORT);
u8 IsRXD_Ready(u8 PORT);

void __ISR_USART_DXL(void);
void __ISR_USART_ZIGBEE(void);
void __ISR_USART_PC(void);

void enableDXLForwarding();

#endif /* __USART_H */

/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
