/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
 * File Name          : system_func.c
 * Author             : zerom
 * Version            : V0.0.1
 * Date               : 08/25/2010
 * Description        : functions about System function.
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_type.h"
#include "common_type.h"
#include "system_init.h"
#include "system_func.h"
#include "adc.h"
#include "serial.h"
#include "usart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define getTimer() 		{(SysTick->VAL)/9;}
/* Private variables ---------------------------------------------------------*/
u32 tmpdly;
vu32 gwTimingDelay;



u8 gbVoltageTable[256]=
{
0 	,
1 	,
2 	,
3 	,
4 	,
5 	,
6 	,
7 	,
8 	,
9 	,
10 	,
11 	,
12 	,
13 	,
14 	,
15 	,
16 	,
17 	,
18 	,
19 	,
20 	,
21 	,
22 	,
23 	,
24 	,
25 	,
26 	,
27 	,
28 	,
29 	,
30 	,
31 	,
32 	,
33 	,
34 	,
35 	,
36 	,
37 	,
38 	,
39 	,
40 	,
41 	,
42 	,
42 	,
43 	,
44 	,
45 	,
46 	,
47 	,
48 	,
49 	,
50 	,
51 	,
52 	,
53 	,
54 	,
55 	,
56 	,
57 	,
58 	,
59 	,
60 	,
61 	,
62 	,
63 	,
64 	,
65 	,
66 	,
67 	,
68 	,
69 	,
70 	,
71 	,
72 	,
73 	,
74 	,
75 	,
76 	,
77 	,
78 	,
79 	,
80 	,
81 	,
82 	,
83 	,
84 	,
85 	,
86 	,
87 	,
88 	,
89 	,
90 	,
91 	,
92 	,
93 	,
94 	,
95 	,
96 	,
97 	,
98 	,
99 	,
100 	,
101 	,
102 	,
103 	,
104 	,
105 	,
106 	,
107 	,
108 	,
109 	,
110 	,
111 	,
112 	,
113 	,
114 	,
115 	,
116 	,
117 	,
118 	,
119 	,
120 	,
121 	,
122 	,
123 	,
124 	,
125 	,
126 	,
127 	,
127 	,
128 	,
129 	,
130 	,
131 	,
132 	,
133 	,
134 	,
135 	,
136 	,
137 	,
138 	,
139 	,
140 	,
141 	,
142 	,
143 	,
144 	,
145 	,
146 	,
147 	,
148 	,
149 	,
150 	,
151 	,
152 	,
153 	,
154 	,
155 	,
156 	,
157 	,
158 	,
159 	,
160 	,
161 	,
162 	,
163 	,
164 	,
165 	,
166 	,
167 	,
168 	,
169 	,
170 	,
171 	,
172 	,
173 	,
174 	,
175 	,
176 	,
177 	,
178 	,
179 	,
180 	,
181 	,
182 	,
183 	,
184 	,
185 	,
186 	,
187 	,
188 	,
189 	,
190 	,
191 	,
192 	,
193 	,
194 	,
195 	,
196 	,
197 	,
198 	,
199 	,
200 	,
201 	,
202 	,
203 	,
204 	,
205 	,
206 	,
207 	,
208 	,
209 	,
210 	,
211 	,
211 	,
212 	,
213 	,
214 	,
215 	,
216 	,
217 	,
218 	,
219 	,
220 	,
221 	,
222 	,
223 	,
224 	,
225 	,
226 	,
227 	,
228 	,
229 	,
230 	,
231 	,
232 	,
233 	,
234 	,
235 	,
236 	,
237 	,
238 	,
239 	,
240 	,
241 	,
242 	,
243 	,
244 	,
245 	,
246 	,
247 	,
248 	,
249 	,
250 	,
251 	,
252
};




/* Private function prototypes -----------------------------------------------*/
u32 Dummy(u32 tmp);


/* Private functions ---------------------------------------------------------*/

u8 getResetSource(void) {
	u8 retval = 0;

	if (RCC_GetFlagStatus(RCC_FLAG_PORRST) == SET)
		retval = POWER_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) == SET)
		retval = PIN_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_SFTRST) == SET)
		retval = SOFT_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET)
		retval = IWDG_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) == SET)
		retval = WWDG_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_LPWRRST) == SET)
		retval = LOW_POWER_RESET;

	RCC_ClearFlag();

	return retval;
}

/*******************************************************************************
 * Function Name  : mDelay
 * Description    : Inserts a delay time.
 * Input          : nTime: specifies the delay time length, in milliseconds.
 * Output         : None
 * Return         : None
 *******************************************************************************/


void mDelay(u32 nTime)
{

	  /* Enable the SysTick Counter */
	  SysTick_CounterCmd(SysTick_Counter_Enable);

	  gwTimingDelay = nTime;

	  while(gwTimingDelay != 0);

	  /* Disable SysTick Counter */
	  SysTick_CounterCmd(SysTick_Counter_Disable);
	  /* Clear SysTick Counter */
	  SysTick_CounterCmd(SysTick_Counter_Clear);

}

u32 Dummy(u32 tmp)
{
	return tmp;
}

void uDelay(u32 uTime) {
	u32 cnt, max;
	static u32 tmp = 0;

	for( max=0; max < uTime; max++)
	{
		for( cnt=0; cnt < 10 ; cnt++ )
		{
			tmp +=Dummy(cnt);
		}
	}
	tmpdly = tmp;
}

void __ISR_DELAY(void)
{
	if (gwTimingDelay != 0x00)
	{
		gwTimingDelay--;
	}

}

void dxl_set_power(PowerState state)
{
	if(state == ON) GPIO_SetBits(PORT_ENABLE_DXLPWR, PIN_ENABLE_DXLPWR);
	else			GPIO_ResetBits(PORT_ENABLE_DXLPWR, PIN_ENABLE_DXLPWR);
	//gbDxlPwr = state;
}

u8 getVoltage(void)
{
    return ( gbVoltageTable[ (getADC(0)) >>4 ] );
}


u16 EEPROM_Read( u32 Offset )
{
	u16* Adr;
	Adr = (u16*)(EEPROM_START_ADDRESS + (Offset<<1));
	return *Adr;
}

void EEPROM_Write( u32 Offset, u16 Data )
{
	volatile FLASH_Status FLASHStatus;
	u32 Adr;
	u16 cnt;
	u16 Buffer[512];

	Adr = EEPROM_START_ADDRESS + (Offset<<1);

	if( (Data != EEPROM_Read(Offset)) && (Offset<512) )
	{
		for( cnt=0; cnt<512; cnt++ )
		{
			Buffer[cnt] = EEPROM_Read(cnt);
		}
		Buffer[Offset] = Data;

		FLASH_Unlock();
		/* Clear All pending flags */
		FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

		if( (Data==0) || (EEPROM_Read(Offset)==0xFFFF) )
		{
			FLASHStatus = FLASH_ProgramHalfWord( Adr, Data );
		}
		else
		{		// Erase
			/* Erase the FLASH pages */
			FLASHStatus = FLASH_ErasePage( EEPROM_START_ADDRESS);

			Adr = EEPROM_START_ADDRESS;

			for( cnt=0; cnt<512; cnt++ )
			{
				if( Buffer[cnt] != 0xFFFF ) FLASHStatus = FLASH_ProgramHalfWord( Adr, Buffer[cnt] );
				Adr += 2;
			}
		}

		FLASH_Lock();
	}
}

void EEPROM_Clear( void )
{
	volatile FLASH_Status FLASHStatus;

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	FLASHStatus = FLASH_ErasePage( EEPROM_START_ADDRESS);
	FLASH_Lock();
}

/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
