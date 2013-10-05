/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : adc.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/09/08
* Description        : functions about button control
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_type.h"
#include "system_init.h"
#include "common_type.h"
#include "adc.h"
#include "CM_DXL_COM.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

vu8 ADC_Channel_Index=0;
vu16 ADC_Value[ADC_NUMBER];
vu32 ADC_Channel[ADC_NUMBER]=
{
	ADC_Channel_10, //ADC1
	ADC_Channel_11,
	ADC_Channel_12,
	ADC_Channel_13,
	ADC_Channel_0,
	ADC_Channel_1,
	ADC_Channel_2,
	ADC_Channel_3,

	ADC_Channel_4, //ADC2
	ADC_Channel_5,
	ADC_Channel_6,
	ADC_Channel_7,
	ADC_Channel_14,
	ADC_Channel_15,
	ADC_Channel_8,
	ADC_Channel_9
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



u16 getADC(u8 ADC_PORT)
{
	return ADC_Value[ADC_PORT];
}



void __ISR_ADC(void)
{

	ADC_Value[ADC_Channel_Index] 	= ADC_GetConversionValue(ADC1);
	ADC_Value[ADC_Channel_Index+8] 	= ADC_GetConversionValue(ADC2);

	ADC_Channel_Index++;

	if(ADC_Channel_Index==8)
	{
		u8 bCount=0;
		ADC_Channel_Index = 0;
		for (bCount = 0; bCount < 15; bCount++)
		{
			WORD_CAST(gbpControlTable[P_ADC1_MIC1+(bCount<<1)]) = getADC(bCount+1)>>2;
		}
	}


	ADC_RegularChannelConfig(ADC1, ADC_Channel[ADC_Channel_Index], 1 , ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel[ADC_Channel_Index+8], 1 , ADC_SampleTime_239Cycles5);
}



/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
