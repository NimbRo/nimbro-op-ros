/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : system_init.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : functions about system init
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "common_type.h"
#include "system_init.h"
#include "system_func.h"
#include "usart.h"
#include "dynamixel.h"
#include "zigbee.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/




// CCR unit= 10uS
vu16 CCR1_Val = 100; 		// 1ms
vu16 CCR2_Val = 778; 		// 7.81ms
vu16 CCR3_Val = 12400;    	// 125ms
vu16 CCR4_Val = 12;    		// 12us

u32 Baudrate_DXL = 	1000000;
u32 Baudrate_ZIGBEE = 57600;
//u32 Baudrate_PC = 57600;
u32 Baudrate_PC = 1000000;

//u8 SPI_Data_Transmit_Complete=FALSE;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void System_Configuration(void)
{

	__disable_interrupt();
	/* System Clocks Configuration */
	RCC_Configuration();
	   
	/* NVIC configuration */
	NVIC_Configuration();


	/* Configure the GPIO ports */
	GPIO_Configuration();



	/* Unlock the Flash Program Erase controller */
	FLASH_Unlock();

	/* USART Configuration */
	USART_Configuration(USART_DXL,Baudrate_DXL);
	//dxl_initialize(USART_DXL,Baudrate_DXL);
	zgb_initialize(0);
	//USART_Configuration(USART_ZIGBEE,Baudrate_ZIGBEE);

	//USART_Configuration(USART_PC,1000000);
	//USART_Configuration(USART_PC,3000000);
	USART_Configuration(USART_PC,Baudrate_PC);


	/* ADC Configuration */
	ADC_Configuration();
	
	

	SysTick_Configuration();
	
	Timer_Configuration();


	SPI_Configuration();

	Buzzer_Configuration();


	GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
	GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
	GPIO_SetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);
	GPIO_SetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);

	__enable_interrupt();



	Gyro_Configuration();
	ACC_Configuration();






}



void Buzzer_Configuration(void)
{

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);


	// Timer Base Init	- Buzzer
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 2000;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	// PWM Init			- Buzzer
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure. TIM_Period / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable);
	TIM_Cmd(TIM4, ENABLE);
	TIM_CtrlPWMOutputs(TIM4, ENABLE);


}



void Timer_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_DeInit(TIM2);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM2, 722, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

/*
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val ;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

*/
	/*
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val ;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);



	TIM_OCInitStructure.TIM_Pulse = CCR3_Val ;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);
*/

	TIM_OCInitStructure.TIM_Pulse = CCR4_Val ;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);


	/* TIM IT enable */
	TIM_ITConfig(TIM2, /*TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 |*/ TIM_IT_CC4 , ENABLE);

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

//#define NO_RESET			0
//#define PIN_RESET			1
//#define POWER_RESET		2
//#define SOFT_RESET		3
//#define IWDG_RESET		4 // independent watchdog reset
//#define WWDG_RESET		5 // window watchdog reset
//#define LOW_POWER_RESET 	6	


void SysTick_Configuration(void)
{
	  /* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
	  SysTick_SetReload(9000);

	  /* Enable SysTick interrupt */
	  SysTick_ITConfig(ENABLE);
}




/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1); 

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1); 

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		/* Enable PLL */ 
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while(RCC_GetSYSCLKSource() != 0x08)
		{
		}
	} 
 
	/* Enable peripheral clocks --------------------------------------------------*/

	/* Enable USART5, GPIOA,and AFIO clocks */
	/* Enable USART5, GPIOA, GPIOB, and AFIO clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 |
							RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
							RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_AFIO, ENABLE);

	RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 |
							 RCC_APB1Periph_USART3 |  RCC_APB1Periph_UART5 | RCC_APB1Periph_SPI2|
							 RCC_APB1Periph_BKP | RCC_APB1Periph_PWR, ENABLE);

	PWR_BackupAccessCmd(ENABLE);
}



void USART_Configuration(u8 PORT, u32 baudrate)
{
	USART_InitTypeDef USART_InitStructure;
	
	USART_StructInit(&USART_InitStructure);
	
	
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


	if( PORT == USART_DXL )
	{
		Baudrate_DXL = baudrate;

		USART_DeInit(USART1);
		/* Configure the USART1 */
		USART_Init(USART1, &USART_InitStructure);
		
		/* Enable USART1 Receive and Transmit interrupts */
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		//USART_ITConfig(USART1, USART_IT_TC, ENABLE);
		
		/* Enable the USART1 */
		USART_Cmd(USART1, ENABLE);
	}
	else if( PORT == USART_ZIGBEE )
	{
		Baudrate_ZIGBEE = baudrate;

		USART_DeInit(UART5);
		/* Configure the UART5 */
		USART_Init(UART5, &USART_InitStructure);
		
		
		/* Enable UART5 Receive and Transmit interrupts */
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
		
		/* Enable the UART5 */
		USART_Cmd(UART5, ENABLE);
	}
	else if( PORT == USART_PC )
	{
		Baudrate_PC = baudrate;
		
		USART_DeInit(USART3);
		
		/* Configure the USART3 */
		USART_Init(USART3, &USART_InitStructure);

		/* Enable USART3 Receive and Transmit interrupts */
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		//USART_ITConfig(USART3, USART_IT_TC, ENABLE);
		
		/* Enable the USART3 */
		USART_Cmd(USART3, ENABLE);
	}
	
}
u32 USART_GetBaudrate(u8 PORT)
{

	if( PORT == USART_DXL )
	{
		return Baudrate_DXL;
	}
	else if( PORT == USART_ZIGBEE )
	{
		return Baudrate_ZIGBEE;
	}
	else if( PORT == USART_PC )
	{
		return Baudrate_PC;
	}
	
	return 0;
}

void ADC_Configuration(void)
{
	
	ADC_InitTypeDef ADC_InitStructure;
	
	ADC_StructInit(&ADC_InitStructure);
	
	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;

	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_Init(ADC2, &ADC_InitStructure);

	/* ADC1 regular channels configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1 , ADC_SampleTime_239Cycles5);
	ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE);

	/* ADC2 regular channels configuration */
	ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5);
	ADC_ITConfig(ADC2, ADC_IT_EOC, DISABLE);

	/* Enable ADC1 DMA */
	//ADC_DMACmd(ADC1, ENABLE);
  
	/* Enable ADC1,2 */
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);


	/* Enable ADC1,2 reset calibaration register */
	/* Check the end of ADC1,2 reset calibration register */
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));


	ADC_ResetCalibration(ADC2);
	while(ADC_GetResetCalibrationStatus(ADC2));



	/* Start ADC1,2 calibaration */
	/* Check the end of ADC1,2 calibration */
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC2);
	while(ADC_GetCalibrationStatus(ADC2));


	/* Start ADC2 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);
}


void SPI_Configuration(void)
{

	//SPI_StructInit(&SPI_InitStructure);

	SPI_InitTypeDef   SPI_InitStructure;

	GPIO_SetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);
	GPIO_SetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	//SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Init(SPI2, &SPI_InitStructure);


	/* Enable SPI2 RXNE interrupt */
	//SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, DISABLE);
	//SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);

	/* Enable SPI1 */
	//SPI_Cmd(SPI1, ENABLE);

	/* Enable SPI2 */
    SPI_Cmd(SPI2, ENABLE);


}



/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	
	// PORTA CONFIG
	GPIO_InitStructure.GPIO_Pin = 	PIN_LED6_R | PIN_LED6_G | PIN_LED6_B ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	


	GPIO_InitStructure.GPIO_Pin = PIN_PA13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = PIN_CPU_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PIN_ADC4 | PIN_ADC5 | PIN_ADC6 | PIN_ADC7 | PIN_ADC8 | PIN_ADC9 | PIN_ADC10 | PIN_ADC11 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PIN_CPU_TXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = PIN_SW_MODE | PIN_SW_START ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA , &GPIO_InitStructure);


	// PORTB CONFIG
	GPIO_InitStructure.GPIO_Pin = 	PIN_ENABLE_ZIGBEE | PIN_ENABLE_TXD | PIN_ENABLE_RXD | PIN_ENABLE_DXLPWR |
									PIN_BOOT1  | PIN_LED3 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

	GPIO_InitStructure.GPIO_Pin = PIN_DXL_RXD | PIN_PC_RXD ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
/*
	 GPIO_Mode_AIN = 0x0,
	  GPIO_Mode_IN_FLOATING = 0x04,
	 GPIO_Mode_IPD = 0x28,
	  GPIO_Mode_IPU = 0x48,
	  GPIO_Mode_Out_OD = 0x14,
	  GPIO_Mode_Out_PP = 0x10,
	  GPIO_Mode_AF_OD = 0x1C,
	  GPIO_Mode_AF_PP = 0x18

	*/


	GPIO_InitStructure.GPIO_Pin = PIN_DXL_TXD | PIN_PC_TXD | PIN_SIG_SCK  | PIN_SIG_MOSI | PIN_SIG_MISO | PIN_BUZZER  ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

	GPIO_InitStructure.GPIO_Pin = PIN_ADC14 | PIN_ADC15 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	
	// PORTC CONFIG
	GPIO_InitStructure.GPIO_Pin = PIN_LED4 | PIN_LED5_R | PIN_LED5_G | PIN_LED5_B | PIN_SIG_ACC_CS | PIN_SIG_GYRO_CS | PIN_LED_TX | PIN_LED_RX | PIN_LED2  ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	

	GPIO_InitStructure.GPIO_Pin =  PIN_ZIGBEE_TXD ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = PIN_ADC0 | PIN_ADC1 | PIN_ADC2 | PIN_ADC3 | PIN_ADC12 | PIN_ADC13 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	

	// PORTD CONFIG
	GPIO_InitStructure.GPIO_Pin = PIN_ZIGBEE_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Configure USART1 Remap enable */
	GPIO_PinRemapConfig( GPIO_Remap_USART1, ENABLE);
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_Disable, ENABLE);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  
//	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);   
//	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);     


#ifdef  VECT_TAB_RAM  
	// Set the Vector Table base location at 0x20000000  
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  // VECT_TAB_FLASH  
	// Set the Vector Table base location at 0x08003000
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);   
//	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);     
#endif


	// Configure the NVIC Preemption Priority Bits   
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
	// Enable the USART1 Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  


	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);



	/* Configure and enable SPI2 interrupt -------------------------------------*/
/*
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);
*/

	/* Configure and enable ADC interrupt */

	//NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQChannel;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_Init(&NVIC_InitStructure);
	
}




/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
