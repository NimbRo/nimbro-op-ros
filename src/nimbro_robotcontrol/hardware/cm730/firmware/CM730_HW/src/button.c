/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : button.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : functions about button control
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_type.h"
#include "common_type.h"
#include "button.h"
#include "system_init.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/*******************************************************************************
* Function Name  : ReadKey
* Description    : Reads key from demoboard.
* Input          : None
* Output         : None
* Return         : Return RIGHT, LEFT, SEL, UP, DOWN or NOKEY
*******************************************************************************/

u8 ReadButton(void)
{
	u8 retval=0;

	if( GPIO_ReadInputDataBit(PORT_SW_START, 	PIN_SW_START) != SET ) 	retval |= BUTTON_START;
	if( GPIO_ReadInputDataBit(PORT_SW_MODE, 	PIN_SW_MODE)  != SET ) 	retval |= BUTTON_MODE;
	
	return retval;
}




/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
