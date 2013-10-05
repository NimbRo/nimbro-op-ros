/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : main.c
* Author             : zerom
* Version            : V0.0.1
* Date               : 08/23/2010
* Description        : Main program body
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "common_type.h"
#include "led.h"
//#include "eeprom.h"
#include "button.h"
#include "usart.h"
#include "system_init.h"
#include "system_func.h"
#include "dynamixel.h"
#include "serial.h"
#include "adc.h"
#include "zigbee.h"
#include "CM_DXL_COM.h"
#include "sound.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


extern u8 gbDxlPwr;
/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


int main(void)
{

	byte bCount;
	long lTemp;

	System_Configuration();

	dxl_set_power(ON);
	Zigbee_SetState(ON);
	//gbDxlPwr = ON;

	//LED_SetState(LED_POWER, ON);
	//	LED_RGB_SetState(LED_R|LED_G|LED_B);
	//LED_RGB_SetState(OFF);

	BufferClear(USART_DXL);
	BufferClear(USART_PC);
	BufferClear(USART_ZIGBEE);

	setBuzzerOff();

/*
	for(bCount =0; bCount < 50; bCount++ )
	{
		setBuzzerData(bCount);
		setBuzzerPlayLength(10);
		PlayBuzzer();
		//mDelay(3000);
		while(getBuzzerState());
		mDelay(500);
	}
*/


/*
	//BKP_WriteBackupRegister((P_OPERATING_MODE+1)<<2, 0xffff);

	  if(BKP_ReadBackupRegister((P_OPERATING_MODE+1)<<2) == 0xffff) //Initialize to Factory Default or reset mode restart
	  {
	    for(bCount=0; bCount < ROM_CONTROL_TABLE_LEN; bCount++)
	    {
	        //ROM_CAST(bCount) = gbpControlTable[bCount] = ROM_INITIAL_DATA[bCount];
	    	gbpControlTable[bCount] = ROM_INITIAL_DATA[bCount];
	        BKP_WriteBackupRegister((bCount+1)<<2, WORD_CAST(gbpControlTable[bCount]));
	    }
	    gbLEDBlinkCounter = 32;
	  }
	  else
	  {
	    for(bCount=0; bCount < ROM_CONTROL_TABLE_LEN; bCount++)
	    {
	      //gbpControlTable[bCount] = (byte)(BKP_ReadBackupRegister((bCount+1)<<2)); !!!!!
	      gbpControlTable[bCount] = ROM_INITIAL_DATA[bCount];
	    }
	    gbLEDBlinkCounter = 8;
	  }

	for (bCount = 0; bCount < 3; bCount++) {
		LED_SetState(LED_MANAGE|LED_EDIT|LED_PLAY, ON);
		mDelay(50);
		LED_SetState(LED_MANAGE|LED_EDIT|LED_PLAY, OFF);
		mDelay(50);
	}
*/
	/*
	if( (EEPROM_Read(P_OPERATING_MODE) == 0xffff) || (EEPROM_Read(P_OPERATING_MODE) == 0xff) ) //Initialize to Factory Default or reset mode restart
	{
		EEPROM_Write( P_BAUD_RATE, ROM_INITIAL_DATA[P_BAUD_RATE] );
		EEPROM_Write( P_OPERATING_MODE, 0);
	}
	else if(EEPROM_Read(P_OPERATING_MODE) == 0x11) //Digital Reset
	{
		EEPROM_Write( P_BAUD_RATE, ROM_INITIAL_DATA[P_BAUD_RATE] );
		EEPROM_Write( P_OPERATING_MODE, 0);
	}
	*/


	for(bCount=0; bCount < ROM_CONTROL_TABLE_LEN; bCount++)
	{
		  gbpControlTable[bCount] = ROM_INITIAL_DATA[bCount];
	}
	//GB_BAUD_RATE = EEPROM_Read(P_BAUD_RATE);
/*
	lTemp = 2000000;
	lTemp /= (GB_BAUD_RATE+1);
	USART_Configuration(USART_DXL,lTemp);
	USART_Configuration(USART_PC,lTemp);
*/


    gbLEDBlinkCounter = 8;


	for (bCount = 0; bCount < 3; bCount++) {
		LED_SetState(LED_MANAGE|LED_EDIT|LED_PLAY, ON);
		mDelay(50);
		LED_SetState(LED_MANAGE|LED_EDIT|LED_PLAY, OFF);
		mDelay(50);
	}

	TorqueOff();

	for (bCount = 0; bCount < 3; bCount++) {
		LED_SetState(LED_MANAGE|LED_EDIT|LED_PLAY, ON);
		mDelay(50);
		LED_SetState(LED_MANAGE|LED_EDIT|LED_PLAY, OFF);
		mDelay(50);
	}

	dxl_set_power(OFF);
/*
	dxl_set_power(1);
	gbDxlPwr = 1;
	mDelay(100);

	GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Disable
	GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Enable
	  while(1)
	  {
		GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Disable
		GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Enable

		  USART_SendData(USART1, 'a');
			while( USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET );

			GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
			GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
			mDelay(100);

	  }

*/

	Process();

	while(1);

}

