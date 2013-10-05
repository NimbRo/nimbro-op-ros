/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : led.c
* Author             : zerom
* Version            : V0.0.1
* Date               : 08/23/2010
* Description        : functions about led control
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_type.h"
#include "common_type.h"
#include "led.h"
#include "system_init.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void LED_SetState(u8 LED_PORT, PowerState NewState)
{
	if( NewState == ON )
	{ 
		if( LED_PORT & LED_MANAGE )	GPIO_ResetBits(PORT_LED_MANAGE, PIN_LED_MANAGE);
		if( LED_PORT & LED_EDIT ) 	GPIO_ResetBits(PORT_LED_EDIT, PIN_LED_EDIT);
		if( LED_PORT & LED_PLAY ) 	GPIO_ResetBits(PORT_LED_PLAY, PIN_LED_PLAY);
		if( LED_PORT & LED_TX ) 	GPIO_ResetBits(PORT_LED_TX, PIN_LED_TX);
		if( LED_PORT & LED_RX ) 	GPIO_ResetBits(PORT_LED_RX, PIN_LED_RX);
		//if( LED_PORT & LED_AUX ) 	GPIO_ResetBits(PORT_LED_AUX, PIN_LED_AUX);
		if( LED_PORT & LED_POWER ) 	GPIO_ResetBits(PORT_LED_POWER, PIN_LED_POWER);
	}
	else
	{
		if( LED_PORT & LED_MANAGE )	GPIO_SetBits(PORT_LED_MANAGE, PIN_LED_MANAGE);
		if( LED_PORT & LED_EDIT ) 	GPIO_SetBits(PORT_LED_EDIT, PIN_LED_EDIT);
		if( LED_PORT & LED_PLAY ) 	GPIO_SetBits(PORT_LED_PLAY, PIN_LED_PLAY);
		if( LED_PORT & LED_TX ) 	GPIO_SetBits(PORT_LED_TX, PIN_LED_TX);
		if( LED_PORT & LED_RX ) 	GPIO_SetBits(PORT_LED_RX, PIN_LED_RX);
		//if( LED_PORT & LED_AUX ) 	GPIO_SetBits(PORT_LED_AUX, PIN_LED_AUX);
		if( LED_PORT & LED_POWER ) 	GPIO_SetBits(PORT_LED_POWER, PIN_LED_POWER);
	}
}

PowerState LED_GetState(u8 LED_PORT)
{
    if( LED_PORT == LED_MANAGE )
    {
    	if( GPIO_ReadOutputDataBit(PORT_LED_MANAGE , 	PIN_LED_MANAGE) != SET ) 	return ON;
    	else																		return OFF;
    }
    else if( LED_PORT == LED_EDIT )
    {
		if( GPIO_ReadOutputDataBit(PORT_LED_EDIT , 	PIN_LED_EDIT) != SET ) 			return ON;
		else																		return OFF;
    }
    else if( LED_PORT == LED_PLAY )
    {
		if( GPIO_ReadOutputDataBit(PORT_LED_PLAY , 	PIN_LED_PLAY) != SET ) 			return ON;
		else																		return OFF;
    }
    else if( LED_PORT == LED_TX )
    {
		if( GPIO_ReadOutputDataBit(PORT_LED_TX , 	PIN_LED_TX) != SET ) 			return ON;
		else																		return OFF;
    }
    else if( LED_PORT == LED_RX )
    {
		if( GPIO_ReadOutputDataBit(PORT_LED_RX , 	PIN_LED_RX) != SET ) 			return ON;
		else																		return OFF;
    }
    else if( LED_PORT == LED_POWER )
    {
		if( GPIO_ReadOutputDataBit(PORT_LED_POWER , 	PIN_LED_POWER) != SET ) 		return ON;
		else																		return OFF;
    }
    else
    	return OFF;

    /*
    else if( LED_PORT == LED_AUX )
    {
		if( GPIO_ReadOutputDataBit(PORT_LED_AUX , 	PIN_LED_AUX) != SET ) 	return ON;
		else																		return OFF;
    }
*/
	return OFF;
}

void LED_RGB_SetState(u8 RGB)
{
	if( RGB & LED_R ) 	GPIO_ResetBits(PORT_LED5_R, PIN_LED5_R);
	else				GPIO_SetBits(PORT_LED5_R, PIN_LED5_R);

	if( RGB & LED_G ) 	GPIO_ResetBits(PORT_LED5_G, PIN_LED5_G);
	else				GPIO_SetBits(PORT_LED5_G, PIN_LED5_G);

	if( RGB & LED_B ) 	GPIO_ResetBits(PORT_LED5_B, PIN_LED5_B);
	else				GPIO_SetBits(PORT_LED5_B, PIN_LED5_B);

}


u8 LED_RGB_GetState()
{
	u8 rgb=0;

	rgb |= GPIO_ReadOutputDataBit(PORT_LED5_R , PIN_LED5_R);
	rgb |= GPIO_ReadOutputDataBit(PORT_LED5_G , PIN_LED5_G)<<1;
	rgb |= GPIO_ReadOutputDataBit(PORT_LED5_B , PIN_LED5_B)<<2;

	return rgb;

}


/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
