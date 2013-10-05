/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : adc.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/09/08
* Description        : This file contains the defines used for button fuctions
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define ADC_PORT0				0
#define ADC_PORT1				1
#define ADC_PORT2				2
#define ADC_PORT3				3
#define ADC_PORT4				4
#define ADC_PORT5				5
#define ADC_PORT6				6
#define ADC_PORT7				7
#define ADC_PORT8				8
#define ADC_PORT9				9
#define ADC_PORT10				10
#define ADC_PORT11				11
#define ADC_PORT12				12
#define ADC_PORT13				13
#define ADC_PORT14				14
#define ADC_PORT15				15

#define ADC_VOLTAGE				ADC_PORT0
#define ADC_OLLO_PORT			ADC_PORT1
#define ADC_GYRO_Z				ADC_PORT2
#define ADC_GYRO_Y				ADC_PORT3
#define ADC_GYRO_X				ADC_PORT4
#define ADC_ACC_X				ADC_PORT5
#define ADC_ACC_Y				ADC_PORT6
#define ADC_ACC_Z				ADC_PORT7


#define ADC_NUMBER					16







/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

u16 getADC(u8 ADC_PORT);
void __ISR_ADC(void);


#endif /* __ADC_H */

/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
