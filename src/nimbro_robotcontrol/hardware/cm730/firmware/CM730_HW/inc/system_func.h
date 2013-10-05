/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : system_func.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : This file contains the defines used for System fucntion
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSTEM_FUNC_H
#define __SYSTEM_FUNC_H

/* Includes ------------------------------------------------------------------*/
#include "common_type.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define NO_RESET			0
#define PIN_RESET			1
#define POWER_RESET			2
#define SOFT_RESET			3
#define IWDG_RESET			4 // independent watchdog reset
#define WWDG_RESET			5 // window watchdog reset
#define LOW_POWER_RESET 	6	


#define EEPROM_START_ADDRESS    ((u32)0x0807F000) /* EEPROM emulation start address:
                                                  after 64KByte of used Flash memory */

/* Exported macro ------------------------------------------------------------*/

#define getTimer() ((SysTick->VAL)/9)

/* Exported functions ------------------------------------------------------- */

void mDelay(u32 nTime);
void uDelay(u32 uTime);
u8 getResetSource(void);
void __ISR_DELAY(void);
void Zigbee_SetState(PowerState state);
void dxl_set_power(PowerState state);

u8 getVoltage(void);


void EEPROM_Write( u32 Offset, u16 Data );
u16 EEPROM_Read( u32 Offset );
void EEPROM_Clear( void );

#endif /* __SYSTEM_FUNC_H */

/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
