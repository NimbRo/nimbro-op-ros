/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : serial.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/26
* Description        : This file contains the serial function.
                       for the firmware.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SERIAL_H
#define __SERIAL_H

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

void TxDString(u8 PORT, u8 *bData);
void TxDHex8(u16 bSentData);
void TxDHex16(u16 wSentData);
void TxDHex32(u32 lSentData);


void TxD_Dec_U8(u8 bByte);
void TxD_Dec_U16(u16 wData);
void TxD_Dec_U32(u32 wData);
void TxD_Dec_S8(s8 wData);
void TxD_Dec_S16(s16 wData);
void TxD_Dec_S32(s32 lLong);
void TxD16DecDigit(u16 wData,u8 bDigit);



/* Exported variables ---------------------------------------------------------*/

#endif /* __SERIAL_H */

/************************ (C) COPYRIGHT 2010 ROBOTIS *********END OF FILE******/
