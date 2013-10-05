/*
 * sound.h
 *
 *  Created on: 2011. 2. 17.
 *      Author: Administrator
 */

#ifndef SOUND_H_
#define SOUND_H_

#include "stm32f10x_type.h"

#define SIZE_DOREMITABLE		52
#define SIZE_SOUNDTABLE			29

//void Buzzer_Configuration(void);
void SetBuzzer(u16 periodUs);


void __ISR_Buzzer_Manage(void);


void PlayDoremi(u16 index, u16 playTime);
void PlayMusic(u8 musicIndex);


void setBuzzerPlayLength(u8 length);
u8 getBuzzerPlayLength(void);
void setBuzzerData(u8 data);
u8 getBuzzerData(void);
u8 getBuzzerState(void);
void PlayBuzzer(void);
void setBuzzerOff(void);


#endif /* SOUND_H_ */
