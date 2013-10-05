// Zigbee SDK platform dependent source
#include "stm32f10x_lib.h"
#include "system_init.h"
#include "usart.h"
#include "zgb_hal.h"
//#include <math.h>

int zgb_hal_open( int devIndex, int baudrate )
{


	//unsigned long int baud = 0;//(unsigned long int)baudrate;
	//int baud = (int)baudrate;
	//TxDHex32(baudrate);
	USART_Configuration(USART_ZIGBEE,57600);

	return 1;

	// Opening device
	// devIndex: Device index
	// baudrate: Real baudrate (ex> 115200, 57600, 38400...)
	// Return: 0(Failed), 1(Succeed)

}

void zgb_hal_close()
{
	// Closing device

}

int zgb_hal_tx( unsigned char *pPacket, int numPacket )
{
	int i;

	for( i=0 ; i < numPacket ; i++ )
		TxDData(USART_ZIGBEE,pPacket[i]);

	return 0;

	// Transmiting date
	// *pPacket: data array pointer
	// numPacket: number of data array
	// Return: number of data transmitted. -1 is error.

}

int zgb_hal_rx( unsigned char *pPacket, int numPacket )
{

	int cnt=0;

	//TxDData(USART_PC,'a');
	if( IsRXD_Ready(USART_ZIGBEE) == 0 ) return -1;


	while(1)
	{

		if( IsRXD_Ready(USART_ZIGBEE) )
		{
			pPacket[cnt++] = RxDBuffer(USART_ZIGBEE);
			if(cnt >= numPacket) break;
		}

	}


	return cnt;

	// Recieving date
	// *pPacket: data array pointer
	// numPacket: number of data array
	// Return: number of data recieved. -1 is error.

}
