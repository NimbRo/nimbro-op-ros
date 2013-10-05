/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : system_init.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : This file contains the defines used for Sys init fuctions
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSTEM_INIT_H
#define __SYSTEM_INIT_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/



///////////////////PORTA//////////////////////////////////////////////////////////
#define PIN_ADC4				GPIO_Pin_0
#define PIN_ADC5				GPIO_Pin_1
#define PIN_ADC6				GPIO_Pin_2
#define PIN_ADC7				GPIO_Pin_3
#define PIN_ADC8				GPIO_Pin_4
#define PIN_ADC9				GPIO_Pin_5
#define PIN_ADC10				GPIO_Pin_6
#define PIN_ADC11				GPIO_Pin_7
#define PIN_LED6_R				GPIO_Pin_8
#define PIN_CPU_TXD				GPIO_Pin_9
#define PIN_CPU_RXD				GPIO_Pin_10
#define PIN_LED6_G				GPIO_Pin_11
#define PIN_LED6_B				GPIO_Pin_12
#define PIN_PA13				GPIO_Pin_13
#define PIN_SW_START			GPIO_Pin_14
#define PIN_SW_MODE				GPIO_Pin_15


#define PORT_ADC4				GPIOA
#define PORT_ADC5				GPIOA
#define PORT_ADC6				GPIOA
#define PORT_ADC7				GPIOA
#define PORT_ADC8				GPIOA
#define PORT_ADC9				GPIOA
#define PORT_ADC10				GPIOA
#define PORT_ADC11				GPIOA
#define PORT_LED6_R				GPIOA
#define PORT_CPU_TXD			GPIOA
#define PORT_CPU_RXD			GPIOA
#define PORT_LED6_G				GPIOA
#define PORT_LED6_B				GPIOA
#define PORT_PA13				GPIOA
#define PORT_SW_MODE			GPIOA
#define PORT_SW_START			GPIOA


///////////////////PORTB//////////////////////////////////////////////////////////
#define PIN_ADC14				GPIO_Pin_0
#define PIN_ADC15				GPIO_Pin_1
#define PIN_BOOT1				GPIO_Pin_2
#define PIN_ENABLE_ZIGBEE		GPIO_Pin_3
#define PIN_ENABLE_TXD			GPIO_Pin_4
#define PIN_ENABLE_RXD			GPIO_Pin_5
#define PIN_DXL_TXD				GPIO_Pin_6
#define PIN_DXL_RXD				GPIO_Pin_7
#define PIN_ENABLE_DXLPWR		GPIO_Pin_8
#define PIN_BUZZER				GPIO_Pin_9
#define PIN_PC_TXD				GPIO_Pin_10
#define PIN_PC_RXD				GPIO_Pin_11
#define PIN_LED3				GPIO_Pin_12
#define PIN_SIG_SCK				GPIO_Pin_13
#define PIN_SIG_MISO			GPIO_Pin_14
#define PIN_SIG_MOSI			GPIO_Pin_15

#define PORT_ADC14				GPIOB
#define PORT_ADC15				GPIOB
#define PORT_BOOT1				GPIOB
#define PORT_ENABLE_ZIGBEE		GPIOB
#define PORT_ENABLE_TXD			GPIOB
#define PORT_ENABLE_RXD			GPIOB
#define PORT_DXL_TXD			GPIOB
#define PORT_DXL_RXD			GPIOB
#define PORT_ENABLE_DXLPWR		GPIOB
#define PORT_BUZZER				GPIOB
#define PORT_PC_TXD				GPIOB
#define PORT_PC_RXD				GPIOB
#define PORT_LED3				GPIOB
#define PORT_SIG_SCK				GPIOB
#define PORT_SIG_MISO				GPIOB
#define PORT_SIG_MOSI				GPIOB





///////////////////PORTC//////////////////////////////////////////////////////////
#define PIN_ADC0				GPIO_Pin_0
#define PIN_ADC1				GPIO_Pin_1
#define PIN_ADC2				GPIO_Pin_2
#define PIN_ADC3				GPIO_Pin_3
#define PIN_ADC12				GPIO_Pin_4
#define PIN_ADC13				GPIO_Pin_5
#define PIN_LED4				GPIO_Pin_6
#define PIN_LED5_R				GPIO_Pin_7
#define PIN_LED5_G				GPIO_Pin_8
#define PIN_LED5_B				GPIO_Pin_9
#define PIN_SIG_ACC_CS			GPIO_Pin_10
#define PIN_SIG_GYRO_CS			GPIO_Pin_11
#define PIN_ZIGBEE_TXD			GPIO_Pin_12
#define PIN_LED_TX				GPIO_Pin_13
#define PIN_LED_RX				GPIO_Pin_14
#define PIN_LED2				GPIO_Pin_15


#define PORT_ADC0				GPIOC
#define PORT_ADC1				GPIOC
#define PORT_ADC2				GPIOC
#define PORT_ADC3				GPIOC
#define PORT_ADC12				GPIOC
#define PORT_ADC13				GPIOC
#define PORT_LED4				GPIOC
#define PORT_LED5_R				GPIOC
#define PORT_LED5_G				GPIOC
#define PORT_LED5_B				GPIOC
#define PORT_SIG_ACC_CS			GPIOC
#define PORT_SIG_GYRO_CS		GPIOC
#define PORT_ZIGBEE_TXD			GPIOC
#define PORT_LED_TX				GPIOC
#define PORT_LED_RX				GPIOC
#define PORT_LED2				GPIOC



///////////////////PORTD///////////////////////////////////////////////////////
#define PIN_ZIGBEE_RXD			GPIO_Pin_2
#define PORT_ZIGBEE_RXD			GPIOD


/////////////////////////// LED REDEFINE /////////////////////////////////////////
#define PIN_LED_MANAGE		PIN_LED2
#define PIN_LED_EDIT		PIN_LED3
#define PIN_LED_PLAY		PIN_LED4
#define PIN_LED_POWER		PIN_LED6_R


#define PORT_LED_MANAGE		PORT_LED2
#define PORT_LED_EDIT		PORT_LED3
#define PORT_LED_PLAY		PORT_LED4
#define PORT_LED_POWER		PORT_LED6_R


/*

///////////////////PORTA//////////////////////////////////////////////////////////
#define PIN_ADC4				GPIO_Pin_0
#define PIN_ADC5				GPIO_Pin_1
#define PIN_ADC6				GPIO_Pin_2
#define PIN_ADC7				GPIO_Pin_3
#define PIN_ADC8				GPIO_Pin_4
#define PIN_ADC9				GPIO_Pin_5
#define PIN_ADC10				GPIO_Pin_6
#define PIN_ADC11				GPIO_Pin_7
#define PIN_LED6_R				GPIO_Pin_8
#define PIN_CPU_TXD				GPIO_Pin_9
#define PIN_CPU_RXD				GPIO_Pin_10
#define PIN_LED6_G				GPIO_Pin_11
#define PIN_LED6_B				GPIO_Pin_12
#define PIN_PA13				GPIO_Pin_13
#define PIN_SW_DXLPWR			GPIO_Pin_14
#define PIN_SW_START			GPIO_Pin_15


#define PORT_ADC4				GPIOA
#define PORT_ADC5				GPIOA
#define PORT_ADC6				GPIOA
#define PORT_ADC7				GPIOA
#define PORT_ADC8				GPIOA
#define PORT_ADC9				GPIOA
#define PORT_ADC10				GPIOA
#define PORT_ADC11				GPIOA
#define PORT_LED6_R				GPIOA
#define PORT_CPU_TXD			GPIOA
#define PORT_CPU_RXD			GPIOA
#define PORT_LED6_G				GPIOA
#define PORT_LED6_B				GPIOA
#define PORT_PA13				GPIOA
#define PORT_SW_DXLPWR			GPIOA
#define PORT_SW_START			GPIOA


///////////////////PORTB//////////////////////////////////////////////////////////
#define PIN_ADC14				GPIO_Pin_0
#define PIN_ADC15				GPIO_Pin_1
#define PIN_BOOT1				GPIO_Pin_2
#define PIN_ENABLE_ZIGBEE		GPIO_Pin_3
#define PIN_ENABLE_TXD			GPIO_Pin_4
#define PIN_ENABLE_RXD			GPIO_Pin_5
#define PIN_DXL_TXD				GPIO_Pin_6
#define PIN_DXL_RXD				GPIO_Pin_7
#define PIN_ENABLE_DXLPWR		GPIO_Pin_8
#define PIN_BUZZER				GPIO_Pin_9
#define PIN_PC_TXD				GPIO_Pin_10
#define PIN_PC_RXD				GPIO_Pin_11
#define PIN_LED_TX				GPIO_Pin_12
#define PIN_LED_RX				GPIO_Pin_13
#define PIN_LED2				GPIO_Pin_14
#define PIN_LED3				GPIO_Pin_15

#define PORT_ADC14				GPIOB
#define PORT_ADC15				GPIOB
#define PORT_BOOT1				GPIOB
#define PORT_ENABLE_ZIGBEE		GPIOB
#define PORT_ENABLE_TXD			GPIOB
#define PORT_ENABLE_RXD			GPIOB
#define PORT_DXL_TXD			GPIOB
#define PORT_DXL_RXD			GPIOB
#define PORT_ENABLE_DXLPWR		GPIOB
#define PORT_BUZZER				GPIOB
#define PORT_PC_TXD				GPIOB
#define PORT_PC_RXD				GPIOB
#define PORT_LED_TX				GPIOB
#define PORT_LED_RX				GPIOB
#define PORT_LED2				GPIOB
#define PORT_LED3				GPIOB





///////////////////PORTC//////////////////////////////////////////////////////////
#define PIN_ADC0				GPIO_Pin_0
#define PIN_ADC1				GPIO_Pin_1
#define PIN_ADC2				GPIO_Pin_2
#define PIN_ADC3				GPIO_Pin_3
#define PIN_ADC12				GPIO_Pin_4
#define PIN_ADC13				GPIO_Pin_5
#define PIN_LED4				GPIO_Pin_6
#define PIN_LED5_R				GPIO_Pin_7
#define PIN_LED5_G				GPIO_Pin_8
#define PIN_LED5_B				GPIO_Pin_9
#define PIN_OLLO3				GPIO_Pin_10
#define PIN_OLLO4				GPIO_Pin_11
#define PIN_ZIGBEE_TXD			GPIO_Pin_12
#define PIN_OLLO0				GPIO_Pin_13
#define PIN_OLLO1				GPIO_Pin_14
#define PIN_OLLO2				GPIO_Pin_15


#define PORT_ADC0				GPIOC
#define PORT_ADC1				GPIOC
#define PORT_ADC2				GPIOC
#define PORT_ADC3				GPIOC
#define PORT_ADC12				GPIOC
#define PORT_ADC13				GPIOC
#define PORT_LED4				GPIOC
#define PORT_LED5_R				GPIOC
#define PORT_LED5_G				GPIOC
#define PORT_LED5_B				GPIOC
#define PORT_OLLO3				GPIOC
#define PORT_OLLO4				GPIOC
#define PORT_ZIGBEE_TXD			GPIOC
#define PORT_OLLO0				GPIOC
#define PORT_OLLO1				GPIOC
#define PORT_OLLO2				GPIOC



///////////////////PORTD///////////////////////////////////////////////////////
#define PIN_ZIGBEE_RXD			GPIO_Pin_2

#define PORT_ZIGBEE_RXD			GPIOD


/////////////////////////// LED REDEFINE /////////////////////////////////////////
#define PIN_LED_MANAGE		PIN_LED2
#define PIN_LED_EDIT		PIN_LED3
#define PIN_LED_PLAY		PIN_LED4
#define PIN_LED_POWER		PIN_LED6_R


#define PORT_LED_MANAGE		PORT_LED2
#define PORT_LED_EDIT		PORT_LED3
#define PORT_LED_PLAY		PORT_LED4
#define PORT_LED_POWER		PORT_LED6_R
*/




/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART_Configuration(u8 PORT, u32 baudrate);
u32 USART_GetBaudrate(u8 PORT);
void ADC_Configuration(void);
void Timer_Configuration(void);
void SysTick_Configuration(void);
void EXTI_Configuration(void);
void System_Configuration(void);

void SPI_Configuration(void);
void Buzzer_Configuration(void);



#endif /* __SYSTEM_INIT_H */

/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
