/*
 * sound.c
 *
 *  Created on: 2011. 2. 17.
 *      Author: Administrator
 */

#include "sound.h"

#include "stm32f10x_lib.h"
#include "stm32f10x_type.h"
#include "common_type.h"
#include "system_init.h"


const u16 DoremiTable[SIZE_DOREMITABLE] =
	{9091, 8584, 8099, 7645, 7220, 6811, 6428, 6067, 5727, 5406,
	5102, 4816, 4545, 4292, 4049, 3822, 3610, 3405, 3214, 3033,
	2863, 2703, 2551, 2408, 2272, 2145, 2025, 1911, 1804, 1702,
	1607, 1517, 1431, 1351, 1275, 1204, 1136, 1072, 1012, 955,
	902, 850, 803, 758, 716, 675, 638, 602, 568, 536, 506, 478};

/*	{18182, 17168, 16198, 15290, 14440, 13622, 12856, 12134, 11454,
	10812, 10204, 9632, 9091, 8584, 8099, 7645, 7220, 6811, 6428,
	6067, 5727, 5406, 5102, 4816, 4545, 4290, 4050, 3822, 3608,
	3405, 3214, 3034, 2863, 2703, 2551, 2408, 2273, 2145, 2025,
	1911, 1804, 1701, 1607, 1517, 1432, 1351, 1276, 1204, 1136,
	1073, 1012, 956};
*/


const u16 SoundTable[SIZE_SOUNDTABLE] =
	{0, 750, 4292, 4049, 3822, 3610, 3405, 3214, 3033, 2863, 2703,
	2551, 2408, 2272, 2145, 2025, 1911, 1804, 1702, 1607, 1517,
	1431, 1351, 1275, 1204, 1136, 900, 12500, 6250};

/*	{0, 1500, 8584, 8099, 7645, 7220, 6811, 6428, 6067, 5727, 5406,
	5102, 4816, 4545, 4290, 4050, 3822, 3608, 3405, 3214, 3034, 2863,
	2703, 2551, 2408, 2273, 1800, 25000, 12500};
*/

// 0x0000 16bit.	0x0200 : OnOff / 0x0780 : LEN / 0x0040 : S_WAVE sign / 0x0020 : S_WAVE flag /  0x001F : SOUND

#define DIGIT_SOUND_LEN		0x0080
#define LEN_16				1 * DIGIT_SOUND_LEN
#define LEN_8				2 * DIGIT_SOUND_LEN
#define LEN_4				4 * DIGIT_SOUND_LEN
#define LEN_2				8 * DIGIT_SOUND_LEN
#define LEN_8H				3 * DIGIT_SOUND_LEN
#define LEN_4H				6 * DIGIT_SOUND_LEN

enum  {high1=1,la0_,si0,do1,do1_,le1,le1_,mi1,fa1,fa1_,sol1,sol1_,la1,la1_,si1,
	   do2,do2_,le2,le2_,mi2,fa2,fa2_,sol2,sol2_,la2,high2,low0,low1};

#define S_OFF			0
#define S_CONTINUE		30
#define S_WAVE			32
#define S_WAVE_SIGN		64
#define S_WAVE_P		S_WAVE
#define S_WAVE_M		S_WAVE + S_WAVE_SIGN
#define S_NULL			0



#define DELTA_M0	4
const u16 Music0[] = {
		8,
		DELTA_M0 + do1	+ LEN_8,
		DELTA_M0 + le1	+ LEN_8,
		DELTA_M0 + mi1	+ LEN_8,
		DELTA_M0 + fa1	+ LEN_8,
		DELTA_M0 + sol1	+ LEN_8,
		DELTA_M0 + la1	+ LEN_8,
		DELTA_M0 + si1	+ LEN_8,
		DELTA_M0 + do2	+ LEN_8,
		S_NULL
};

#define DELTA_M1	0
const u16 Music1[] = {
		8,
		DELTA_M1 + do2	+ LEN_8,
		DELTA_M1 + si1	+ LEN_8,
		DELTA_M1 + la1	+ LEN_8,
		DELTA_M1 + sol1	+ LEN_8,
		DELTA_M1 + fa1	+ LEN_8,
		DELTA_M1 + mi1	+ LEN_8,
		DELTA_M1 + le1	+ LEN_8,
		DELTA_M1 + do1	+ LEN_2,
		S_NULL
};

#define DELTA_M6	4
const u16 Music6[] = {
		15,
		DELTA_M6 + do1 	+ LEN_8,
		DELTA_M6 + fa1 	+ LEN_8,
		DELTA_M6 + la1 	+ LEN_8,
		DELTA_M6 + do2 	+ LEN_8H,

		S_OFF 			+ LEN_16,
		DELTA_M6 + la1 	+ LEN_8,
		DELTA_M6 + do2 	+ LEN_2,
		S_CONTINUE 		+ LEN_8,
		S_OFF 			+ LEN_8,

		DELTA_M6 + do2 	+ LEN_8,
		S_OFF 			+ LEN_8,
		DELTA_M6 + do2 	+ LEN_16,
		DELTA_M6 + do2 	+ LEN_16,
		DELTA_M6 + do2 	+ LEN_8,
		S_OFF 			+ LEN_8,
		DELTA_M6 + do2 	+ LEN_8,

		DELTA_M6 + fa2 	+ LEN_2,

		S_OFF 			+ LEN_4,
		S_NULL
};

#define DELTA_M7	12
const u16 Music7[] = {
		23,
		DELTA_M7 + do1	+ LEN_8,
		S_OFF 			+ LEN_16,
		DELTA_M7 + do1	+ LEN_16,
		DELTA_M7 + do1	+ LEN_16,
		DELTA_M7 + do1	+ LEN_8,
		DELTA_M7 + do1	+ LEN_8,
		DELTA_M7 + sol1	+ LEN_16,
		DELTA_M7 + mi1	+ LEN_16,
		DELTA_M7 + sol1	+ LEN_16,
		DELTA_M7 + mi1	+ LEN_16,
		DELTA_M7 + do1	+ LEN_2,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound0[] = {
		20,
		do2				+ LEN_16,
		S_WAVE_P + 8	+ LEN_8H,
		S_CONTINUE		+ LEN_8,
		S_WAVE_M + 8	+ LEN_8H,
		S_WAVE_P + 12	+ LEN_16,
		S_WAVE_M + 12	+ LEN_16,

		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound1[] = {
		20,
		mi1				+ LEN_8,
		S_WAVE_P + 4	+ LEN_8,
		S_WAVE_M + 2	+ LEN_2,
		S_CONTINUE		+ LEN_2,
		S_CONTINUE		+ LEN_4,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound2[] = {
		16,
		sol2			+ LEN_16,
		S_WAVE_P + 18	+ LEN_16,
		S_WAVE_M + 18	+ LEN_16,
		S_OFF			+ LEN_16,
		S_WAVE_P + 18	+ LEN_16,
		S_WAVE_M + 18	+ LEN_16,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound3[] = {
		32,
		fa1				+ LEN_8,
		S_WAVE_P + 1	+ LEN_2,
		S_CONTINUE		+ LEN_4,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound4[] = {
		20,
		sol2			+ LEN_8,
		S_WAVE_P + 16	+ LEN_16,
		S_WAVE_M + 1	+ LEN_4,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound5[] = {
		16,
		sol2			+ LEN_8,
		S_WAVE_P + 3	+ LEN_4,
		S_WAVE_M + 3	+ LEN_4,
		S_WAVE_P + 3	+ LEN_4,
		S_WAVE_M + 3	+ LEN_4,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound6[] = {
		20,
		do2				+ LEN_16,
		S_WAVE_M + 12	+ LEN_2,
		S_CONTINUE		+ LEN_2,
		S_NULL
};

const u16 Sound7[] = {
		20,
		high2			+ LEN_16,
		S_WAVE_M + 1	+ LEN_2,
		S_WAVE_P + 1	+ LEN_2,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound8[] = {
		16,
		high1			+ LEN_16,
		S_WAVE_M + 1	+ LEN_16,
		high2			+ LEN_16,
		S_WAVE_M + 1	+ LEN_16,
		S_WAVE_P + 1	+ LEN_16,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound10[] = {
		16,
		high2			+ LEN_8,
		S_WAVE_M + 10	+ LEN_8,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound11[] = {
		16,
		high2			+ LEN_16,
		S_WAVE_P + 10	+ LEN_16,
		S_WAVE_M + 10	+ LEN_16,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound12[] = {
		16,
		high2			+ LEN_16,
		S_WAVE_P + 10	+ LEN_16,
		S_WAVE_M + 10	+ LEN_16,
		S_OFF			+ LEN_16,
		S_WAVE_P + 10	+ LEN_16,
		S_WAVE_M + 10	+ LEN_16,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound13[] = {
		8,
		mi2				+ LEN_16,
		S_WAVE_P + 5	+ LEN_16,
		sol2			+ LEN_4,
		mi2				+ LEN_4,
		S_WAVE_P + 5	+ LEN_16,
		sol2			+ LEN_4,
		mi2				+ LEN_4,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound15[] = {
		9,
		S_OFF			+ LEN_2,
		la1_			+ LEN_4,
		S_OFF			+ LEN_16,
		la0_			+ LEN_8H,
		la0_			+ LEN_8H,
		la0_			+ LEN_8H,
		S_OFF			+ LEN_2,
		S_NULL

};

const u16 Sound20[] = {
		16,
		low1			+ LEN_16,
		S_WAVE_P + 50	+ LEN_2,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound21[] = {
		4,
		high1			+ LEN_16,
		S_WAVE_M + 20	+ LEN_2,
		S_WAVE_P + 20	+ LEN_2,
		S_WAVE_M + 40	+ LEN_2,
		S_WAVE_P + 40	+ LEN_2,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound22[] = {
		1,
		high2			+ LEN_16,
		S_WAVE_M + 10	+ LEN_2,
		S_WAVE_P + 10	+ LEN_2,
		S_WAVE_M + 20	+ LEN_2,
		S_WAVE_P + 20	+ LEN_2,
		S_WAVE_M + 30	+ LEN_2,
		S_WAVE_P + 30	+ LEN_2,
		S_WAVE_M + 40	+ LEN_2,
		S_WAVE_P + 40	+ LEN_2,
		S_WAVE_M + 50	+ LEN_2,
		S_WAVE_P + 50	+ LEN_2,
		S_WAVE_M + 60	+ LEN_2,
		S_WAVE_P + 60	+ LEN_2,
		S_WAVE_M + 70	+ LEN_2,
		S_WAVE_P + 70	+ LEN_2,
		S_WAVE_M + 80	+ LEN_2,
		S_WAVE_P + 80	+ LEN_2,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound23[] = {
		8,
		high1			+ LEN_16,
		S_WAVE_M + 8	+ LEN_2,
		S_WAVE_P + 1	+ LEN_2,
		S_OFF 			+ LEN_8,
		S_NULL
};

const u16 Sound24[] = {
		4,
		la2				+ LEN_8H,
		S_OFF			+ LEN_16,
		la2				+ LEN_8H,
		S_OFF			+ LEN_16,
		la2				+ LEN_8H,
		S_OFF			+ LEN_16,
		la2				+ LEN_8H,
		S_OFF			+ LEN_16,
		S_NULL
};

const u16 Sound25[] = {
		16,
		fa2				+ LEN_8,
		S_WAVE_M + 18	+ LEN_4,
		S_WAVE_M + 18	+ LEN_4,
		S_WAVE_M + 18	+ LEN_4,
		S_OFF			+ LEN_8,
		S_NULL
};

#define DELTA_S30	1
const u16 Sound30[] = {
		8,
		high2			+ LEN_16,
		high1			+ LEN_16,
		S_WAVE_M + 8	+ LEN_16,
		S_WAVE_P + 2	+ LEN_4H,
		S_WAVE_P + 1	+ LEN_4,
		S_OFF			+ LEN_8,
		S_NULL
};

const u16 Sound31[] = {
		7,
		high1			+ LEN_16,
		fa2 - 1			+ LEN_8,
		S_OFF			+ LEN_16,
		high1			+ LEN_16,
		fa2 - 2			+ LEN_8,
		S_OFF			+ LEN_16,
		high1			+ LEN_16,
		fa2 - 3			+ LEN_8,
		S_OFF			+ LEN_16,
		high1			+ LEN_16,
		fa2 - 4			+ LEN_8,
		S_OFF			+ LEN_16,
		high1			+ LEN_16,
		fa2 - 5			+ LEN_8,
		S_OFF			+ LEN_16,
		S_NULL
};


const u16* MusicTable[] = {&Music0, &Music1, &Music6, &Music7, &Sound0, &Sound1, &Sound2, &Sound3,
							&Sound4, &Sound5, &Sound6, &Sound7, &Sound8, &Sound10, &Sound11, &Sound12,
							&Sound13, &Sound15, &Sound20, &Sound21, &Sound22, &Sound23, &Sound24, &Sound25,
							&Sound30, &Sound31};

#define MUSIC_N		26



u16 Music_MusicIndex = 0;

u16 Music_TempoContainer = 1;
u16 Music_TempoCnt = 0;
u16 Music_ReadIndex = 0;
u16 Music_Play = 0;					// 0 : 정지 / 1 : 재생
u16 Music_CurrentPacket = 0;
u16 Music_CurrentSound = 0;
u16 Music_CurrentLen = 0;

u16 Music_WaveFlag = 0;
u16 Music_WaveBuffer = 0;
s16 Music_WaveStep = 0;

u16 Music_StartDelayFlag = 0;

u16 Doremi_Play = 0;
u16 Doremi_Index = 0;
u16 Doremi_TimCount = 0;

u8 gbBuzzerPlayLength = 0;
u8 gbBuzzerData = 0;



void setBuzzerPlayLength(u8 length)
{
	gbBuzzerPlayLength = length;
}

u8 getBuzzerPlayLength(void)
{
	return gbBuzzerPlayLength;
}

void setBuzzerData(u8 data)
{
	gbBuzzerData = data;
}

u8 getBuzzerData(void)
{
	return gbBuzzerData;
}

u8 getBuzzerState(void)
{
	return (Music_Play || Doremi_Play);
}

void PlayBuzzer(void)
{
	if( gbBuzzerPlayLength == 0xFF )
	{
		PlayMusic(gbBuzzerData);
	}
	else
	{
		PlayDoremi(gbBuzzerData,gbBuzzerPlayLength);
	}
}

void setBuzzerOff(void)
{
	Music_Play = Doremi_Play = 0;
	SetBuzzer(0);
}

void __ISR_Buzzer_Manage(void)
{
	if (Music_Play || Doremi_Play)
	{
		// 마이크 인터럽트 disable.
	}
	else
	{
		// 마이크 인터럽트 enable.
	}

	if (Doremi_Play)
	{
		if (!Doremi_TimCount)
		{
			Doremi_Play = 0;
			SetBuzzer(0);
			return;
		}

		Doremi_TimCount--;

		SetBuzzer(DoremiTable[Doremi_Index]);

		return;
	}

	// 정지 상태라면 종료.
	if (Music_Play == 0)
		return;

	// 템포 카운트가 0이 되면,
	if (!Music_TempoCnt)
	{
		// 인덱스가 0이면 템포를 저장하고 인덱스를 +1.
		if (Music_ReadIndex == 0)
			Music_TempoContainer = (*(MusicTable + Music_MusicIndex))[Music_ReadIndex++];

		Music_CurrentPacket = (*(MusicTable + Music_MusicIndex))[Music_ReadIndex];
		Music_CurrentLen = (Music_CurrentPacket >> 7) & 0x000F;
		Music_CurrentSound = Music_CurrentPacket & 0x001F;

		// 악보의 'Null'이 발견되면 종료.
		if (Music_CurrentPacket == S_NULL)
		{
			Music_ReadIndex = 0;
			Music_Play = 0;
			SetBuzzer(0);
			return;
		}

		// 현재 음계가 'WAVE'로 되어 있으면 WaveFlag를 Set, Step을 저장, 템포 Set.
		if (Music_CurrentPacket & S_WAVE)
		{
			Music_WaveFlag = 1;
			if (Music_CurrentPacket & S_WAVE_SIGN)
				Music_WaveStep = 0 - (s16)Music_CurrentSound;
			else
				Music_WaveStep = (s16)Music_CurrentSound;

			Music_TempoCnt = Music_CurrentLen * Music_TempoContainer;
			Music_ReadIndex++;
			return;
		}
		else
			Music_WaveFlag = 0;

		// 현재 음계가 'CONTINUE'로 되어 있으면, 음계를 유지하고 템포만 Set.
		if (Music_CurrentSound == S_CONTINUE)
		{
			Music_TempoCnt = Music_CurrentLen * Music_TempoContainer;
			Music_ReadIndex++;
			return;
		}

		if (Music_StartDelayFlag == 1)
		{
			Music_StartDelayFlag = 0;
			// 현재 음 길이에 템포를 곱하여 세팅.
			Music_TempoCnt = Music_CurrentLen * Music_TempoContainer;
			// 현재 음계를 넣는다.
			SetBuzzer(SoundTable[Music_CurrentSound]);
			Music_WaveBuffer = SoundTable[Music_CurrentSound];
			Music_ReadIndex++;
		}
		else
			Music_StartDelayFlag = 1;

	}
	else
	{
		Music_TempoCnt--;

		if (Music_WaveFlag == 1)
		{
			Music_WaveBuffer = (u16)((s16)Music_WaveBuffer + Music_WaveStep);
			SetBuzzer(Music_WaveBuffer);
		}
	}

}

void PlayDoremi(u16 index, u16 playTime)	// playTime unit : 0.1s
{
	if (index > 51)
		index = 51;

	if (playTime > 50)
		playTime = 50;
	else if (playTime == 0)
		playTime = 3;

	Doremi_Index = index;
	Doremi_TimCount = playTime * 25;
	Doremi_Play = 1;

}

void PlayMusic(u8 musicIndex)
{
	if (Music_Play)
		return;

	if (musicIndex < MUSIC_N)
	{
		Music_MusicIndex = musicIndex;
		Music_ReadIndex = 0;
		Music_Play = 1;
	}
}

void SetBuzzer(u16 periodUs)
{
	//periodUs /= 2;
	TIM_SetAutoreload(TIM4, periodUs);
	TIM_SetCompare4(TIM4, periodUs / 2);

}



