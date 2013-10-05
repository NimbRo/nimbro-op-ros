/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : common_type.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/24
* Description        : This file contains the common data types and defines used
                       for the firmware.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMON_TYPE_H
#define __COMMON_TYPE_H

/* */
#define byte u8
#define word u16


/* Includes ------------------------------------------------------------------*/
#include "cortexm3_macro.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {OFF = 0, ON = !OFF} PowerState;

/* Exported constants --------------------------------------------------------*/
#define COMMAND_BUFFER_SIZE (60+70)
#define COMMAND_LENGTH  (16)
#define PARA_NUM        (30+2)

#define ESC_KEY     	0x1b
#define TAB_KEY     	0x09
#define ENTER_KEY   	0x0D

/*ASCII CODE NAME*/
#define CTRL_A          1
#define CLEAR_SCREEN    12
#define LF_ASCII        10
#define CR_ASCII        13
#define BS_ASCII        8
#define BEEP            7


/* Exported macro ------------------------------------------------------------*/
#define __disable_interrupt() __SETPRIMASK()
#define __enable_interrupt()  __RESETPRIMASK()

#define WORD_CAST(AA)		(*(u16 *)(&(AA)))

/* Exported variables ---------------------------------------------------------*/

#endif /* __COMMON_TYPE_H */

/************************ (C) COPYRIGHT 2010 ROBOTIS *********END OF FILE******/
