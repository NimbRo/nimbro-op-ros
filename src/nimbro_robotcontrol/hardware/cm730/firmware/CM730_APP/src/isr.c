/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : isr.c
* Author             : danceww
* Version            : V0.1
* Date               : 08/23/2010
* Description        : functions about Interrupt service routine
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_type.h"
#include "common_type.h"
#include "usart.h"
#include "isr.h"
#include "led.h"
#include "adc.h"
#include "system_func.h"
#include "CM_DXL_COM.h"
#include "system_init.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLINK_COUNT		10 //(0.5s * 10 = 5s)
#define BLINK_TIME		1042 //(480us * 1042 = 0.5s)

#define VOLTAGE_LEVEL_5     161     //12.05V
#define VOLTAGE_LEVEL_4     155     //11.62V
#define VOLTAGE_LEVEL_3     152     //11.40V
#define VOLTAGE_LEVEL_2     149     //11.25V
#define VOLTAGE_LEVEL_1     147     //11.05V
#define VOLTAGE_LEVEL_0     141     //10.64V
#define LOW_BATTERY_COUNT       20  //

#define	SOUND_LOW_VOLTAGE_LENGTH    255
#define	SOUND_LOW_VOLTAGE_DATA      22


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern int giBusUsing;          // from dynamixel.c
extern vu8 gbUSB_Enable;
vu8 bLED_Counter=0;
vu16 wLED_Timer=0;
vu8 bLedBlinkFlag=0;




/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#define LOW_BATTERY_LIMIT
void ISR_BATTERY_CHECK(void)
{

	static byte bLowBatteryCount;
	GB_ADC0_VOLTAGE = getVoltage();

	if (  GB_ADC0_VOLTAGE <= VOLTAGE_LEVEL_1) {
		if (bLowBatteryCount < LOW_BATTERY_COUNT) bLowBatteryCount++;
	}
	else {
		bLowBatteryCount = 0;
	}

	if ( bLowBatteryCount == LOW_BATTERY_COUNT  )
	{
		setBuzzerPlayLength(SOUND_LOW_VOLTAGE_LENGTH);
		setBuzzerData(SOUND_LOW_VOLTAGE_DATA);


		if(!getBuzzerState())
		{

			PlayBuzzer();
		}
	}

}

void ISR_1ms_TIMER(void)
{
	gbMiliSec++;
}

void ISR_SPI_READ(void)
{
	__GYRO_ACC_READ_ISR();
}

void ISR_USART_DXL(void)
{
	__ISR_USART_DXL();

}
void ISR_USART_ZIGBEE(void)
{
	__ISR_USART_ZIGBEE();
}

void ISR_USART_PC(void)
{
	__ISR_USART_PC();
}


void ISR_LED_RGB_TIMER(void)
{
	vu8 gbUSB_Enable = 0;

	gbLEDPwm = (++gbLEDPwm)&0x1f;

	if( GPIO_ReadInputDataBit(PORT_PA13, 	PIN_PA13) != SET ) 	gbUSB_Enable = 0;
	else														gbUSB_Enable = 1;


	if( gbUSB_Enable == 0 )
	{
		gbLEDHeadR = 0;
		gbLEDHeadG = 0x1F;
		gbLEDHeadB = 0;

		if( bLED_Counter )
		{
			wLED_Timer--;
			if( wLED_Timer == 0 )
			{
				bLED_Counter--;
				wLED_Timer = BLINK_TIME;
				if( bLedBlinkFlag )	bLedBlinkFlag = 0;
				else				bLedBlinkFlag = 1;

				if( bLED_Counter == 0 ) bLedBlinkFlag = 0;
			}
		}

		if( !bLedBlinkFlag )
		{
			GPIO_SetBits(PORT_LED5_R, PIN_LED5_R);
			GPIO_SetBits(PORT_LED5_G, PIN_LED5_G);
			GPIO_SetBits(PORT_LED5_B, PIN_LED5_B);
		}
		else
		{
			GPIO_SetBits(PORT_LED5_R, PIN_LED5_R);
			GPIO_ResetBits(PORT_LED5_G, PIN_LED5_G);
			GPIO_SetBits(PORT_LED5_B, PIN_LED5_B);
		}

	}
	else
	{
		bLED_Counter=BLINK_COUNT;
		wLED_Timer=BLINK_TIME;

		if (gbLEDPwm >= gbLEDHeadR)
		{
			GPIO_SetBits(PORT_LED5_R, PIN_LED5_R);
		}
		else
		{
			GPIO_ResetBits(PORT_LED5_R, PIN_LED5_R);
		}
		if (gbLEDPwm >= gbLEDHeadG)
		{
			GPIO_SetBits(PORT_LED5_G, PIN_LED5_G);
		}
		else
		{
			GPIO_ResetBits(PORT_LED5_G, PIN_LED5_G);
		}
		if (gbLEDPwm >= gbLEDHeadB)
		{
			GPIO_SetBits(PORT_LED5_B, PIN_LED5_B);
		}
		else
		{
			GPIO_ResetBits(PORT_LED5_B, PIN_LED5_B);
		}
	}

	if (gbLEDPwm >= gbLEDEyeR)
	{
		GPIO_SetBits(PORT_LED6_R, PIN_LED6_R);
	}
	else
	{
		GPIO_ResetBits(PORT_LED6_R, PIN_LED6_R);
	}
	if (gbLEDPwm >= gbLEDEyeG)
	{
		GPIO_SetBits(PORT_LED6_G, PIN_LED6_G);
	}
	else
	{
		GPIO_ResetBits(PORT_LED6_G, PIN_LED6_G);
	}
	if (gbLEDPwm >= gbLEDEyeB)
	{
		GPIO_SetBits(PORT_LED6_B, PIN_LED6_B);
	}
	else
	{
		GPIO_ResetBits(PORT_LED6_B, PIN_LED6_B);
	}
}


void ISR_DELAY(void)
{
	__ISR_DELAY();
}

void ISR_ADC(void)
{
	__ISR_ADC();
}

void ISR_BUZZER(void)
{

}

void ISR_MOTOR_CONTROL(void)
{

}

//gyro read.
//while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
//GPIO_ResetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);

/*
for(i=0;i<3;i++)
{
	address = (0x28 + (i*2));
	address |= 0xC0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);



	SPI_I2S_SendData(SPI2,address);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_I2S_ReceiveData(SPI2);



	SPI_I2S_SendData(SPI2,0xFF);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI2) );


	SPI_I2S_SendData(SPI2,0xFF);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI2) );

	//while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_SetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);
}
*/



/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
