/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : gyro_acc.c
* Author             : danceww
* Version            : V0.0.1
* Date               : 2011/01/15
* Description        : functions about sensor control
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "gyro_acc.h"
#include "stm32f10x_type.h"
#include "stm32f10x_lib.h"
#include "system_init.h"
#include "serial.h"
#include "CM_DXL_COM.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// raw data -32000~32000
s16 Gyro_X_raw;
s16 Gyro_Y_raw;
s16 Gyro_Z_raw;
s16 ACC_X_raw;
s16 ACC_Y_raw;
s16 ACC_Z_raw;


// convert data 0 ~ 1023
u16 Gyro_X;
u16 Gyro_Y;
u16 Gyro_Z;
u16 ACC_X;
u16 ACC_Y;
u16 ACC_Z;


vu8 SPI_TxBuffer[9]={	0xE8,0xFF,0xFF,
						0xEA,0xFF,0xFF,
						0xEC,0xFF,0xFF };

vu8 SPI_RxBuffer[18];

vu8 SPI_RxBufferPointer=0;
vu8 SPI_TxBufferPointer=0;

vu8 GYRO_ACC_ENABLE = FALSE ;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


void Push_SPI_Data(u16 dat)
{
	SPI_RxBuffer[SPI_RxBufferPointer++] = (u8)( dat & 0x00FF );
}

void Clear_SPI_Data(void)
{
	SPI_RxBufferPointer = 0;
}

void CovertData(void)
{
	s32 temp;

	Gyro_X_raw = (s16)((SPI_RxBuffer[2] << 8) + SPI_RxBuffer[1]);
	Gyro_Y_raw = (s16)((SPI_RxBuffer[5] << 8) + SPI_RxBuffer[4]);
	Gyro_Z_raw = (s16)((SPI_RxBuffer[8] << 8) + SPI_RxBuffer[7]);

	ACC_X_raw = (s16)((SPI_RxBuffer[11] << 8) + SPI_RxBuffer[10]);
	ACC_Y_raw = (s16)((SPI_RxBuffer[14] << 8) + SPI_RxBuffer[13]);
	ACC_Z_raw = (s16)((SPI_RxBuffer[17] << 8) + SPI_RxBuffer[16]);

// 400dps /1023 0.39
	// 500dps /65535 0.007


	// convert data s16 -> u10
	temp = (Gyro_X_raw /64);
	temp = temp * 5 / 4;
	temp = 512 + temp;
	if( temp > 1023 ) temp = 1023;
	if(temp  < 0 ) temp = 0;
	GW_GYRO_X = Gyro_X = temp;

	// convert data s16 -> u10
	temp = Gyro_Y_raw /64;
	temp = temp * 5 / 4;
	temp = 512 + temp;
	if( temp > 1023 ) temp = 1023;
	if(temp  < 0 ) temp = 0;
	GW_GYRO_Y = Gyro_Y = temp;

	// convert data s16 -> u10
	temp = Gyro_Z_raw /64;
	temp = temp * 5 / 4;
	temp = 512 + temp;
	if( temp > 1023 ) temp = 1023;
	if(temp  < 0 ) temp = 0;
	GW_GYRO_Z = Gyro_Z = temp;


	// convert data s16 -> u10
	temp = (-1)*(ACC_X_raw /64);
	//temp = temp * 4 / 3;
	temp = 512 + temp;
	if( temp > 1023 ) temp = 1023;
	if(temp  < 0 ) temp = 0;
	GW_ACC_X = ACC_X = temp;


	// convert data s16 -> u10
	temp = (-1)*(ACC_Y_raw /64);
	//temp = temp * 4 / 3;
	temp = 512 + temp;
	if( temp > 1023 ) temp = 1023;
	if(temp  < 0 ) temp = 0;
	GW_ACC_Y = ACC_Y = temp;


	// convert data s16 -> u10
	temp = ACC_Z_raw /64;
	//temp = temp * 4 / 3;
	temp = 512 + temp;
	if( temp > 1023 ) temp = 1023;
	if(temp  < 0 ) temp = 0;
	GW_ACC_Z  = ACC_Z = temp;

/*
	TxD_Dec_S16(Gyro_X_raw);
	TxDString(USART_PC," ");

	TxD_Dec_S16(Gyro_Y_raw);
	TxDString(USART_PC," ");

	TxD_Dec_S16(Gyro_Z_raw);
	TxDString(USART_PC,"            ");

	TxD_Dec_S16(ACC_X_raw);
	TxDString(USART_PC," ");

	TxD_Dec_S16(ACC_Y_raw);
	TxDString(USART_PC," ");

	TxD_Dec_S16(ACC_Z_raw);
	TxDString(USART_PC,"\r\n");


	TxD_Dec_U16(Gyro_X);
	TxDString(USART_PC," ");

	TxD_Dec_U16(Gyro_Y);
	TxDString(USART_PC," ");

	TxD_Dec_U16(Gyro_Z);
	TxDString(USART_PC,"            ");

	TxD_Dec_U16(ACC_X);
	TxDString(USART_PC," ");

	TxD_Dec_U16(ACC_Y);
	TxDString(USART_PC," ");

	TxD_Dec_U16(ACC_Z);
	TxDString(USART_PC,"\r\n");

*/




}

u16 getGyroX(void)
{
	return Gyro_X;
}
u16 getGyroY(void)
{
	return Gyro_Y;
}
u16 getGyroZ(void)
{
	return Gyro_Z;
}

u16 getACC_X(void)
{
	return ACC_X;
}

u16 getACC_Y(void)
{
	return ACC_Y;
}
u16 getACC_Z(void)
{
	return ACC_Z;
}




s16 getGyroX_raw(void)
{
	return Gyro_X_raw;
}
s16 getGyroY_raw(void)
{
	return Gyro_Y_raw;
}
s16 getGyroZ_raw(void)
{
	return Gyro_Z_raw;
}

s16 getACC_X_raw(void)
{
	return ACC_X_raw;
}

s16 getACC_Y_raw(void)
{
	return ACC_Y_raw;
}
s16 getACC_Z_raw(void)
{
	return ACC_Z_raw;
}


void Gyro_Configuration(void)
{


	// write 0x20FF
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);

	SPI_I2S_SendData(SPI2,0x20);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI2) );

	SPI_I2S_SendData(SPI2,0xFF);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI2) );

	GPIO_SetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);


	//write 0x2310
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);

	SPI_I2S_SendData(SPI2,0x23);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI2) );

	SPI_I2S_SendData(SPI2,0x00);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI2) );

	GPIO_SetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);

	Clear_SPI_Data();

}


void ACC_Configuration(void)
{

	// write 0x202F
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);

	SPI_I2S_SendData(SPI2,0x20);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI2) );

	SPI_I2S_SendData(SPI2,0x3F);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI2) );

	GPIO_SetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);




	//write 0x2310
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);

	SPI_I2S_SendData(SPI2,0x23);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI2) );

	SPI_I2S_SendData(SPI2,0x10);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	Push_SPI_Data( SPI_I2S_ReceiveData(SPI2) );

	GPIO_SetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);

	Clear_SPI_Data();


}


void __GYRO_ACC_READ_ISR(void)
{

	int i;

	//gyro read
	for(i=0;i<9;i++)
	{
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		GPIO_ResetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);

		SPI_I2S_SendData(SPI2,SPI_TxBuffer[i]);
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
		Push_SPI_Data( SPI_I2S_ReceiveData(SPI2) );

		if( (i+1)%3 == 0 ) GPIO_SetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);
	}
	//GPIO_SetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);


	//acc read
	for(i=0;i<9;i++)
	{
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		GPIO_ResetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);

		SPI_I2S_SendData(SPI2,SPI_TxBuffer[i]);
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
		Push_SPI_Data( SPI_I2S_ReceiveData(SPI2) );

		if( (i+1)%3 == 0 ) GPIO_SetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);
	}
	//GPIO_SetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);

	CovertData();
	Clear_SPI_Data();


}


/*


void Gyro_Configuration(void)
{

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

	//mDelay(1);

	GPIO_ResetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);


	//SPI_I2S_SendData(SPI2,0x200F);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	//SPI_I2S_SendData(SPI2,0x20FF);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	SPI_I2S_SendData(SPI2,0x20);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2,0xFF);	// WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 800HZ, POWER : NORMAL, XYZ ENABLE.


	//mDelay(1);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

	SPI_I2S_ReceiveData(SPI2); // for flush FIFO
	GPIO_SetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);



	/////////////////////////////////////////////////////////////////////////////



	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

	GPIO_ResetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);


	//SPI_I2S_SendData(SPI2,0x2310);	// WRITE 0XFF TO CTRL_REG3(0X23). FULL SCALE 500DPS
	SPI_I2S_SendData(SPI2,0x23);	// WRITE 0XFF TO CTRL_REG3(0X23). FULL SCALE 500DPS
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2,0x10);	// WRITE 0XFF TO CTRL_REG3(0X23). FULL SCALE 500DPS


	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

	SPI_I2S_ReceiveData(SPI2); // for flush FIFO

	GPIO_SetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);


}


void ACC_Configuration(void)
{

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);

	//SPI_I2S_SendData(SPI2,0x203F); // WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 1000HZ, POWER : NORMAL, XYZ ENABLE.
	SPI_I2S_SendData(SPI2,0x20); // WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 1000HZ, POWER : NORMAL, XYZ ENABLE.
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2,0x3F); // WRITE 0XFF TO CTRL_REG1(0X20). OUTPUT DATA RATE 1000HZ, POWER : NORMAL, XYZ ENABLE.

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

	SPI_I2S_ReceiveData(SPI2);

	GPIO_SetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);




	/////////////////////////////////////////////////////////////////////////////

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_ResetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);

	//SPI_I2S_SendData(SPI2,0x2310); // WRITE 0XFF TO CTRL_REG4(0X20). FULL SCALE 4G.
	SPI_I2S_SendData(SPI2,0x23); // WRITE 0XFF TO CTRL_REG4(0X20). FULL SCALE 4G.
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2,0x10); // WRITE 0XFF TO CTRL_REG4(0X20). FULL SCALE 4G.

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

	SPI_I2S_ReceiveData(SPI2);

	GPIO_SetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);

}
*/
/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
