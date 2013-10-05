
/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : isr.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/26
* Description        : This file contains the Interrupt service routine
                       for the firmware.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ISR_H
#define _ISR_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

void ISR_USART_DXL(void);
void ISR_USART_ZIGBEE(void);
void ISR_USART_PC(void);

void ISR_LED_RGB_TIMER(void);


void ISR_ADC(void);
void ISR_BUZZER(void);
void ISR_MOTOR_CONTROL(void);


void ISR_SPI_READ(void);
void ISR_BATTERY_CHECK(void);
//void setLED_Enable(u8 Enable);

/* Exported variables ---------------------------------------------------------*/
extern vu8 low_battery_alam_enable;
#endif /* _ISR_H */

/************************ (C) COPYRIGHT 2010 ROBOTIS *********END OF FILE******/
