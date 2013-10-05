
#include "common_type.h"
#include "dynamixel.h"
#include "stm32f10x_lib.h"
#include "system_func.h"
#include "adc.h"
#include "led.h"
#include "system_init.h"
#include "CM_DXL_COM.h"
#include "gyro_acc.h"
#include "button.h"
#include "sound.h"
#include "zigbee.h"
#include "usart.h"

/////////////////////////////////////////////////////////////////////////////
//	<Compile Option>
/////////////////////////////////////////////////////////////////////////////

#define TIMEOUT_CHECK
#define	BROADCAST_PING_DISABLE

/////////////////////////////////////////////////////////////////////////////
//	</Compile Option>
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
//	<Constant Definition>
/////////////////////////////////////////////////////////////////////////////


#define VOLTAGE_ERROR_BIT     0x01
#define ANGLE_LIMIT_ERROR_BIT 0x02
#define OVERHEATING_ERROR_BIT 0x04
#define RANGE_ERROR_BIT       0x08
#define CHECKSUM_ERROR_BIT    0x10
#define OVERLOAD_ERROR_BIT    0x20
#define INSTRUCTION_ERROR_BIT 0x40

#define RETURN_NO_PACKET 0
#define RETURN_READ_PACKET 1
#define RETURN_ALL_PACKET 2


/*__flash*/ byte gbpParameterRange[][2] =
{
  {1,0},//MODEL_NUMBER_L      0
  {1,0},//MODEL_NUMBER_H      1
  {1,0},//VERSION             2
  {0,253},//ID                3
  {1,254},//BAUD_RATE         4
  {0,254},  //Return Delay time 5
  {0,255},//CW_ANGLE_LIMIT_L   6
  {0,3},  //CW_ANGLE_LIMIT_H   7
  {0,255},//CCW_ANGLE_LIMIT_L   8
  {0,3},  //CCW_ANGLE_LIMIT_H   9
//  {0,255},//RESERVED SYSTEM_DATA2        10
  {1,0},//RESERVED SYSTEM_DATA2        10 for center calibration offset
  {0,150},//LIMIT_TEMPERATURE   11
  {50,250},//DOWN_LIMIT_VOLTAGE   12
  {50,250},//UP_LIMIT_VOLTAGE 13
  {0,255},//MAX_TORQUE_L        14
  {0,3},//MAX_TORQUE_H          15
  {0,2},//RETURN_LEVEL          16
  {0,0x7f},//ALARM_LED          17
  {0,0x7f},//ALARM_SHUTDOWN     18
  {0,255},//RESERVED OPERATING_MODE      19
  {1,0},//DOWN_CALIBRATION_L    20
  {1,0},//DOWN_CALIBRATION_H    21
  {1,0},//UP_CALIBRATION_L      22
  {1,0},//UP_CALIBRATOIN_H      23

    //RAM area
  {0, 1}, //	P_DYNAMIXEL_POWER	24
  {0, 7}, //	P_LED_PANNEL	25
  {0, 255}, //	P_LED_HEAD	26
  {0, 127}, //	-	27
  {0, 255}, //	P_LED_EYE	28
  {0, 127}, //	-	29
  {1, 0}, //	P_BUTTON	30
  {1, 0}, //	-	31
  {1, 0}, //	-	32
  {1, 0}, //	-	33
  {1, 0}, //	-	34
  {1, 0}, //	-	35
  {1, 0}, //	-	36
  {1, 0}, //	-	37
  {1, 0}, //	P_GYRO_Z	38
  {1, 0}, //	-	39
  {1, 0}, //	P_GYRO_Y	40
  {1, 0}, //	-	41
  {1, 0}, //	P_GYRO_X	42
  {1, 0}, //	-	43
  {1, 0}, //	P_ACC_X	44
  {1, 0}, //	-	45
  {1, 0}, //	P_ACC_Y	46
  {1, 0}, //	-	47
  {1, 0}, //	P_ACC_Z	48
  {1, 0}, //	-	49
  {1, 0}, //	P_ADC0_VOLTAGE	50
  {1, 0}, //	P_ADC1_MIC1		51
  {1, 0}, //	-				52
  {1, 0}, //	P_ADC9_MIC2		53
  {1, 0}, //	-				54
  {1, 0}, //	P_ADC2			55
  {1, 0}, //	-				56
  {1, 0}, //	P_ADC3			57
  {1, 0}, //	-				58
  {1, 0}, //	P_ADC4			59
  {1, 0}, //	-				60
  {1, 0}, //	P_ADC5			61
  {1, 0}, //	-				62
  {1, 0}, //	P_ADC6			63
  {1, 0}, //	-				64
  {1, 0}, //	P_ADC7			65
  {1, 0}, //	-				66
  {1, 0}, //	P_ADC8			67
  {1, 0}, //	-				68
  {1, 0}, //	P_ADC10			69
  {1, 0}, //	-				70
  {1, 0}, //	P_ADC11			71
  {1, 0}, //	-				72
  {1, 0}, //	P_ADC12			73
  {1, 0}, //	-				74
  {1, 0}, //	P_ADC13			75
  {1, 0}, //	-				76
  {1, 0}, //	P_ADC14			77
  {1, 0}, //	-				78
  {1, 0}, //	P_ADC15			79
  {1, 0}, //	-				80
  {0,0xFF},//	P_BUZZER_DATA0	81
  {0,0xFF},//	P_BUZZER_DATA1	82
  {0,0xFF},//	P_TX_REMOCON_DATA_L			83
  {0,0xFF},//	P_TX_REMOCON_DATA_H			84
  {1,0},//		P_RX_REMOCON_DATA_L			85
  {1,0},//		P_RX_REMOCON_DATA_H			86
  {1,0},//		P_RX_REMOCON_DATA_ARRIVED	87
  {1,0},//		P_ZIGBEE_ID_L				88
  {1,0},//		P_ZIGBEE_ID_H				89
};

/*__flash*/ byte gbpDataSize[] =
{
  2,//MODEL_NUMBER_L      0
  0,//MODEL_NUMBER_H      1
  1,//VERSION             2
  1,//ID                  3
  1,//BAUD_RATE           4
  1,//SYSTEM_DATA         5
  2,//CW_ANGLE_LIMIT_L    6
  0,//CW_ANGLE_LIMIT_H    7
  2,//CCW_ANGLE_LIMIT_L   8
  0,//CCW_ANGLE_LIMIT_H   9
  1,//ACCEL               10
  1,//LIMIT_TEMPERATURE   11
  1,//UP_LIMIT_VOLTAGE    12
  1,//DOWN_LIMIT_VOLTAGE  13
  2,//MAX_TORQUE_L        14
  0,//MAX_TORQUE_H        15
  1,//RETURN_LEVEL        16
  1,//ALARM_LED           17
  1,//ALARM_SHUTDOWN      18
  1,//OPERATING_MODE      19
  2,//DOWN_CALIBRATION_L  20
  0,//DOWN_CALIBRATION_H  21
  2,//UP_CALIBRATION_L    22
  0,//UP_CALIBRATOIN_H    23

      //RAM area
  1, //	P_DYNAMIXEL_POWER	24
  1, //	P_LED_PANNEL	25
  2, //	P_LED_HEAD	26
  0, //	-	27
  2, //	P_LED_EYE	28
  0, //	-	29
  1, //	P_BUTTON	30
  1, //	-	31
  1, //	-	32
  1, //	-	33
  1, //	-	34
  1, //	-	35
  1, //	-	36
  1, //	-	37
  2, //	P_GYRO_Z	38
  0, //	-	39
  2, //	P_GYRO_Y	40
  0, //	-	41
  2, //	P_GYRO_X	42
  0, //	-	43
  2, //	P_ACC_X	44
  0, //	-	45
  2, //	P_ACC_Y	46
  0, //	-	47
  2, //	P_ACC_Z	48
  0, //	-	49
  1, //	P_ADC0_VOLTAGE	50
  2, //	P_ADC1_MIC1		51
  0, //	-				52
  2, //	P_ADC9_MIC1		53
  0, //	-				54
  2, //	P_ADC2			55
  0, //	-				56
  2, //	P_ADC3			57
  0, //	-				58
  2, //	P_ADC4			59
  0, //	-				60
  2, //	P_ADC5			61
  0, //	-				62
  2, //	P_ADC6			63
  0, //	-				64
  2, //	P_ADC7			65
  0, //	-				66
  2, // P_ADC8			67
  0, //	-				68
  2, //	P_ADC10			69
  0, //	-				70
  2, //	P_ADC11			71
  0, //	-				72
  2, //	P_ADC12			73
  0, //	-				74
  2, //	P_ADC13			75
  0, //	-				76
  2, //	P_ADC14			77
  0, //	-				78
  2, //	P_ADC15			79
  0, //	-				80
  1,//	P_BUZZER_DATA0	81
  1,//	P_BUZZER_DATA1	82
  2,//	P_TX_REMOCON_DATA_L			83
  0,//	P_TX_REMOCON_DATA_H			84
  2,//	P_RX_REMOCON_DATA_L			85
  0,//	P_RX_REMOCON_DATA_H			86
  1,//	P_RX_REMOCON_DATA_ARRIVED	87
  2,//	P_ZIGBEE_ID_L				88
  0//	P_ZIGBEE_ID_H				89
};


#define DEFAULT_BAUD_RATE 1    //1Mbps at 16MHz
#define PROGRAM_VERSION 0x13     // for ax12, freq. selectable
#define CW_ANGLE_FIXED_LIMIT 0 // 0+30 dudung031002
#define CCW_ANGLE_FIXED_LIMIT (1023) // 300-30 dudung031002


//byte ROM_INITIAL_DATA[]={ 28, 0 ,PROGRAM_VERSION, DEFAULT_ID, DEFAULT_BAUD_RATE, 0, CW_ANGLE_FIXED_LIMIT&0xff, CW_ANGLE_FIXED_LIMIT>>8, CCW_ANGLE_FIXED_LIMIT&0xff, CCW_ANGLE_FIXED_LIMIT>>8,  0,  85-5, 60,190,255,  3,  2/*0Ver8*/, 0x24,  0x24,  0,  0&0xff,0>>8,0&0xff,0>>8};
//MODEL NUMBER 0X7300
byte ROM_INITIAL_DATA[]={ 0, 0x73 ,PROGRAM_VERSION, DEFAULT_ID, DEFAULT_BAUD_RATE, 0, CW_ANGLE_FIXED_LIMIT&0xff, CW_ANGLE_FIXED_LIMIT>>8, CCW_ANGLE_FIXED_LIMIT&0xff, CCW_ANGLE_FIXED_LIMIT>>8,  0,  85-5, 60,190,255,  3,  2/*0Ver8*/, 0x24,  0x24,  0,  0&0xff,0>>8,0&0xff,0>>8};

/////////////////////////////////////////////////////////////////////////////
//	</Constant Definition>
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
//	<Macro Definition>
/////////////////////////////////////////////////////////////////////////////

#define SYSTEM_RESET NVIC_GenerateSystemReset()

#define TxD8 TxDByte
#define RxD8 RxDByte
#define TxDByte TxD1Byte
#define RxDByte RxD1Byte
#define TxD8Hex TxDByte16
#define TxD8Dec TxDByte10




/////////////////////////////////////////////////////////////////////////////
//	</Macro Definition>
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////
//	<Include Header Files>
/////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////
//	</Include Header Files>
/////////////////////////////////////////////////////////////////////////////



volatile byte gbpRxInterruptBuffer[256];
volatile byte gbpTxInterruptBuffer[256];
volatile byte gbRxBufferWritePointer;
volatile byte gbRxBufferReadPointer;
volatile byte gbTxBufferWritePointer;
volatile byte gbTxBufferReadPointer;

volatile byte gbpRxD0Buffer[256];
volatile byte gbpTxD0Buffer[256];
volatile byte gbRxD0BufferWritePointer;
volatile byte gbRxD0BufferReadPointer;
volatile byte gbTxD0BufferWritePointer;
volatile byte gbTxD0BufferReadPointer;

volatile byte gbpRxD1Buffer[256];
volatile byte gbpTxD1Buffer[256];
volatile byte gbRxD1BufferWritePointer;
volatile byte gbRxD1BufferReadPointer;
volatile byte gbTxD1BufferWritePointer;
volatile byte gbTxD1BufferReadPointer;

volatile byte gbTxD0Transmitting;
volatile byte gbTxD1Transmitting;


volatile byte gbMiliSec;

byte gbRxID;

byte gbParameterLength;

byte gbInstruction;

volatile byte gbpParameter[256];

byte gbStartAddress;

byte gbInterruptCheckError;
byte gbRegAddress, gbRegParameterLength;
byte gbpRegParameter[MAX_PACKET_LENGTH];

volatile byte gbpControlTable[CONTROL_TABLE_LEN+1];

byte gbLEDBlinkCounter;
volatile byte gbLEDHeadR;
volatile byte gbLEDHeadG;
volatile byte gbLEDHeadB;
volatile byte gbLEDEyeR;
volatile byte gbLEDEyeG;
volatile byte gbLEDEyeB;
volatile byte gbLEDPwm;

void WriteControlTable(void);
byte WriteControlTableRangeCheck(void);
void ReturnPacket(byte bError);
void ProcessAfterWriting(void);

u8 gbDxlPwr;

void TorqueOff(void)
{
	gbTxD0BufferReadPointer = gbTxD0BufferWritePointer = 0;
	gbpTxD0Buffer[gbTxD0BufferWritePointer++] = 0xff;
	gbpTxD0Buffer[gbTxD0BufferWritePointer++] = 0xff;
	gbpTxD0Buffer[gbTxD0BufferWritePointer++] = 0xFE; // Broadcast
	gbpTxD0Buffer[gbTxD0BufferWritePointer++] = 4;
	gbpTxD0Buffer[gbTxD0BufferWritePointer++] = INST_WRITE;
	gbpTxD0Buffer[gbTxD0BufferWritePointer++] = 0x18; // Torque enable
	gbpTxD0Buffer[gbTxD0BufferWritePointer++] = 0x00; // off
	gbpTxD0Buffer[gbTxD0BufferWritePointer++] = ~((u8)(0xFE + 4 + INST_WRITE + 0x18 + 0x00));

	if ( GPIO_ReadOutputDataBit(PORT_ENABLE_TXD, PIN_ENABLE_TXD) == Bit_RESET) {
		//TxDString(USART_ZIGBEE,"\r\n TEST0");
		//if (TXD0_FINISH) {
		GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Disable
		GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Enable

		USART_SendData(USART1, gbpTxD0Buffer[gbTxD0BufferReadPointer++]);
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	}
}

void Process(void)
{
  byte bCount, bLength, bEndAddress, bCount0xff, bCheckSum, bReturn,bPrevID;;
  word wTemp;
  byte bTemp;


  GB_ID = DEFAULT_ID;


  GW_LED_HEAD = ((0>>3)<<10)|((255>>3)<<5)|(0>>3);
  GW_LED_EYE =  ((255>>3)<<10)|((0>>3)<<5)|(0>>3);


  /*
	GW_LED_HEAD = ((0>>3)<<10)|((128>>3)<<5)|(255>>3);
	GW_LED_EYE =  ((255>>3)<<10)|((0>>3)<<5)|(0>>3);
*/
	gbLEDHeadR = GW_LED_HEAD&0x1f;
	gbLEDHeadG = (GW_LED_HEAD>>5)&0x1f;
	gbLEDHeadB = (GW_LED_HEAD>>10)&0x1f;

	gbLEDEyeR = GW_LED_EYE&0x1f;
	gbLEDEyeG = (GW_LED_EYE>>5)&0x1f;
	gbLEDEyeB = (GW_LED_EYE>>10)&0x1f;

  GW_ZIGBEE_ID = ScanZigbee();







  #define TIMEOUT_MILISEC 100
  while(1)
  {
    RX_PACKET_START:
    RX_PACKET_TIMEOUT:

    bCount0xff = 0;
    while(1)
    {
      #ifdef TIMEOUT_CHECK
      gbMiliSec = 0;
      #endif
      while(gbRxBufferReadPointer == gbRxBufferWritePointer)
      {
        #ifdef TIMEOUT_CHECK
        if(gbMiliSec > TIMEOUT_MILISEC) goto RX_PACKET_TIMEOUT;
        #endif
      }


      if((gbRxID = gbpRxInterruptBuffer[gbRxBufferReadPointer++]) == 0xff) bCount0xff++;
      else
      {
        if(bCount0xff >= 2) break;
        bCount0xff = 0;
      }
    }

    if(gbRxID == GB_ID || gbRxID == BROADCASTING_ID)
    {
      #ifdef TIMEOUT_CHECK
      gbMiliSec = 0;
      #endif
      while(gbRxBufferReadPointer == gbRxBufferWritePointer)
      {
        #ifdef TIMEOUT_CHECK
        if(gbMiliSec > TIMEOUT_MILISEC) goto RX_PACKET_TIMEOUT;
        #endif
      }
      bLength = gbpRxInterruptBuffer[gbRxBufferReadPointer++];

      gbParameterLength = bLength-2;
      if(gbParameterLength > MAX_PACKET_LENGTH) goto RX_PACKET_START; //Ver8

      //from this state, status packet will be returned

      #ifdef TIMEOUT_CHECK
      gbMiliSec = 0;
      #endif
      while(gbRxBufferReadPointer == gbRxBufferWritePointer)
      {
        #ifdef TIMEOUT_CHECK
        if(gbMiliSec > TIMEOUT_MILISEC) goto RX_PACKET_TIMEOUT;
        #endif
      }
      gbInstruction = gbpRxInterruptBuffer[gbRxBufferReadPointer++];
      bCheckSum = gbRxID+bLength+gbInstruction;

      for(bCount = 0; bCount < gbParameterLength+1; bCount++)
      {
        #ifdef TIMEOUT_CHECK
        gbMiliSec = 0;
        #endif
        while(gbRxBufferReadPointer == gbRxBufferWritePointer)
        {
          #ifdef TIMEOUT_CHECK
          if(gbMiliSec > TIMEOUT_MILISEC) goto RX_PACKET_TIMEOUT;
          #endif
        }
        bCheckSum += (gbpParameter[bCount] = gbpRxInterruptBuffer[gbRxBufferReadPointer++]);
      }
      //Packet Receiving End
      gbStartAddress = gbpParameter[0];

      for(bCount = 0; bCount < GB_RETURN_DELAY_TIME; bCount++)
      {
        // __delay_cycles(32);//2usec
    	  uDelay(2);
      }
      if(bCheckSum != 0xff)
      {
        //buffer initialize
        gbRxBufferWritePointer = gbRxBufferReadPointer;
        if(gbInstruction == INST_PING) {
        }
        else {
            ReturnPacket(CHECKSUM_ERROR_BIT);
        }
      }
      else
      {
    	  	if(gbInstruction == INST_BULK_READ ) //INST_SYNC_WR only 2009.12.11.buche
			{
    	  	  gbRxD0BufferWritePointer = gbRxD0BufferReadPointer = 0;


			  bPrevID = 0xFF;
			  for(bCount = 2; bCount < bLength-3; bCount += (3) )
			  {
				if(gbpParameter[bCount] == GB_ID)
				{
				gbRxID = GB_ID;
				gbInstruction = INST_READ;
				  bLength = 4;
				  gbStartAddress = gbpParameter[bCount+1];
				  gbpParameter[1] = gbpParameter[bCount-1];

			// waiting
				  if (bPrevID == 0xFF) break;
				  else {

					  while(1)
					  {
					//    RX_PACKET_START:
					//    RX_PACKET_TIMEOUT:

						u8 bWaitRxID, bWaitLength, bWaitParameterLength, bWaitInstruction, bWaitCheckSum;

						bCount0xff = 0;


						while(1)
						{
						  gbMiliSec = 0;

						  while( gbRxD0BufferWritePointer == gbRxD0BufferReadPointer )
						  {
							if(gbMiliSec > TIMEOUT_MILISEC) goto RX_PACKET_TIMEOUT;


						  }

						  if((bWaitRxID = gbpRxD0Buffer[gbRxD0BufferReadPointer++] ) == 0xff) bCount0xff++;
						  else
						  {

							if(bCount0xff >= 2)	break;
							bCount0xff = 0;
						  }
						}


						if(bWaitRxID == bPrevID)
						{
						  gbMiliSec = 0;
						  while( gbRxD0BufferWritePointer == gbRxD0BufferReadPointer )
						  {
							if(gbMiliSec > TIMEOUT_MILISEC) goto RX_PACKET_TIMEOUT;
						  }

						  bWaitLength = gbpRxD0Buffer[gbRxD0BufferReadPointer++];

						  bWaitParameterLength = bWaitLength-2;
						  if(bWaitParameterLength > MAX_PACKET_LENGTH) goto RX_PACKET_START; //Ver8

						  //from this state, status packet will be returned

						  gbMiliSec = 0;
						  while( gbRxD0BufferWritePointer == gbRxD0BufferReadPointer )
						  {
							if(gbMiliSec > TIMEOUT_MILISEC) goto RX_PACKET_TIMEOUT;
						  }

						  bWaitInstruction = gbpRxD0Buffer[gbRxD0BufferReadPointer++];
						  bWaitCheckSum = bWaitRxID+bWaitLength+bWaitInstruction;

						  for(bCount = 0; bCount < bWaitParameterLength+1; bCount++)
						  {
							gbMiliSec = 0;
							while( gbRxD0BufferWritePointer == gbRxD0BufferReadPointer )
							{
							  if(gbMiliSec > TIMEOUT_MILISEC) goto RX_PACKET_TIMEOUT;
							}
							bWaitCheckSum += (gbpRxD0Buffer[gbRxD0BufferReadPointer++]);
						  }


						  for(bCount = 0; bCount < GB_RETURN_DELAY_TIME; bCount++)
						  {
							  uDelay(2);

							//__delay_cycles(32);//2usec
					#ifdef	PLL_CLOCK_48MHZ
							DelayCycle( 16 );
					#endif
					#ifdef	PLL_CLOCK_32MHZ
							DelayCycle( 10 );
					#endif
						  }

						  break;
						}
					  }

				  }
				}
				bPrevID = gbpParameter[bCount];
			  }
			  //bPrevID = gbpParameter[bCount];
			}
        if(gbInstruction == INST_SYNC_WRITE || gbInstruction == INST_SYNC_REG_WRITE) //INST_SYNC_WR or INST_SYNC_REG_WR
        {
          byte bTmpLength, bCount0;
          bTmpLength = gbpParameter[1];
          //Blink(bTmpLength);
//          for(bCount = 2; bCount < bLength; bCount += (bTmpLength+1) ) ;;;;;;;
          for(bCount = 2; bCount < bLength-3; bCount += (bTmpLength+1) )
          {
            if(gbpParameter[bCount] == GB_ID)
            {
              bCount++; //point to Data_N
              for(bCount0 = 1; bCount0 <= bTmpLength; bCount0++)
              {
                gbpParameter[bCount0] = gbpParameter[bCount++];
              }
              gbInstruction &= 0x7f;//change to INST_WRITE or INST_REG_WRITE
              bLength = bTmpLength + 3;
              gbParameterLength = bLength-2;
              break;
            }
          }
        }
        //else
        if(gbInstruction == INST_WRITE)
        {
          bReturn = WriteControlTableRangeCheck();
          ReturnPacket(bReturn);
          if(bReturn != RANGE_ERROR_BIT)
          {
            WriteControlTable();
            ProcessAfterWriting();
          }
        }
        else if(gbInstruction == INST_READ)
        {

          if(gbRxID != BROADCASTING_ID && GB_RETURN_LEVEL >= RETURN_READ_PACKET)
          {
        	  	  	//byte readData;
					//Call routine processing item : InstructionError,ChecksumError,TimeoutError
					bEndAddress = gbStartAddress+gbpParameter[1]-1;
					bLength = gbpParameter[1]+2; //Errorstatus,Checksum
					bCheckSum = GB_ID + bLength + gbInterruptCheckError;


					gbpTxD1Buffer[gbTxD1BufferWritePointer++] = 0xff;
					gbpTxD1Buffer[gbTxD1BufferWritePointer++] = 0xff;
					gbpTxD1Buffer[gbTxD1BufferWritePointer++] = GB_ID;
					gbpTxD1Buffer[gbTxD1BufferWritePointer++] = bLength;
					gbpTxD1Buffer[gbTxD1BufferWritePointer++] = gbInterruptCheckError;


					gbpTxD0Buffer[gbTxD0BufferWritePointer++]= 0xff;
					gbpTxD0Buffer[gbTxD0BufferWritePointer++]= 0xff;
					gbpTxD0Buffer[gbTxD0BufferWritePointer++]= GB_ID;
					gbpTxD0Buffer[gbTxD0BufferWritePointer++]= bLength;
					gbpTxD0Buffer[gbTxD0BufferWritePointer++]= gbInterruptCheckError;


					for(bCount=gbStartAddress; bCount <= bEndAddress; bCount++)   // ;;;;;;;;;;;DEBUG after 1.15!!!
					{
					  byte bFixedData;
					  word wFixedData;

					  if( bCount == P_RX_REMOCON_DATA_ARRIVED)
					  {
						  GB_RX_REMOCON_DATA_ARRIVED = zgb_rx_check();
					  }
					  else if( bCount == P_RX_REMOCON_DATA_L )
					  {
						  GW_RX_REMOCON_DATA = zgb_rx_data();
					  }

					  if(gbpDataSize[bCount] == 2 && bCount < bEndAddress)
					  {
						wFixedData = WORD_CAST(gbpControlTable[bCount]);
						bFixedData = (byte)(wFixedData&0xff);
						gbpTxD1Buffer[gbTxD1BufferWritePointer++] = bFixedData;
						gbpTxD0Buffer[gbTxD0BufferWritePointer++]= bFixedData;
						bCheckSum += bFixedData;
						bFixedData = (byte)((wFixedData>>8)&0xff);
						gbpTxD1Buffer[gbTxD1BufferWritePointer++] = bFixedData;
						gbpTxD0Buffer[gbTxD0BufferWritePointer++]= bFixedData;
						bCheckSum += bFixedData;
						bCount++;
					  }
					  else //length == 1 or 0
					  {
						bFixedData = gbpControlTable[bCount];
						gbpTxD1Buffer[gbTxD1BufferWritePointer++] = bFixedData;
						gbpTxD0Buffer[gbTxD0BufferWritePointer++]= bFixedData;
						bCheckSum += bFixedData;
					  }
					}
					bCheckSum ^= 0xff;

				   gbpTxD1Buffer[gbTxD1BufferWritePointer++] = bCheckSum;
				   gbpTxD0Buffer[gbTxD0BufferWritePointer++]= bCheckSum;



					if (gbTxD1Transmitting==0) {
					  gbTxD1Transmitting = 1;
					//  if (TXD1_FINISH) {
						USART_SendData(USART3, gbpTxD1Buffer[gbTxD1BufferReadPointer++]);
						USART_ITConfig(USART3, USART_IT_TC, ENABLE);
						//TXD1_DATA = gbpTxD1Buffer[gbTxD1BufferReadPointer++];
					}

					if ( GPIO_ReadOutputDataBit(PORT_ENABLE_TXD, PIN_ENABLE_TXD) == Bit_RESET) {
						//TxDString(USART_ZIGBEE,"\r\n TEST0");
						//if (TXD0_FINISH) {
						GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Disable
						GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Enable

						USART_SendData(USART1, gbpTxD0Buffer[gbTxD0BufferReadPointer++]);
						USART_ITConfig(USART1, USART_IT_TC, ENABLE);

					}


					while(gbTxD1Transmitting);



					/*

					LED_SetState(LED_PLAY,OFF);
					LED_SetState(LED_EDIT,OFF);



					if(gbTxD0BufferWritePointer != gbTxD0BufferReadPointer ) 	LED_SetState(LED_PLAY,ON);
					else														LED_SetState(LED_EDIT,ON);

					*/


/*
					for(bTemp = gbTxD0BufferReadPointer; bTemp < gbTxD0BufferWritePointer; bTemp++ )
					{
						TxDString(USART_ZIGBEE,"\r\n");
						TxDHex8(gbpTxD0Buffer[bTemp]);

					}

*/


					//USART_ITConfig(USART1, USART_IT_TC, ENABLE);

/*
					gbTxD0Transmitting = 1;
				//  if (TXD1_FINISH) {
					USART_SendData(USART1, gbpTxD0Buffer[gbTxD0BufferReadPointer++]);
					USART_ITConfig(USART1, USART_IT_TC, ENABLE);

					while(gbTxD0Transmitting);

*/







										//TXD1_DATA = gbpTxD1Buffer[gbTxD1BufferReadPointer++];


/*
					while(gbTxD0BufferReadPointer != gbTxD0BufferWritePointer);


					gbpTxD0Buffer[gbTxD0BufferWritePointer++]= 0xff;
					gbpTxD0Buffer[gbTxD0BufferWritePointer++]= 0xff;
					gbpTxD0Buffer[gbTxD0BufferWritePointer++]= GB_ID;
					gbpTxD0Buffer[gbTxD0BufferWritePointer++]= bLength;
					gbpTxD0Buffer[gbTxD0BufferWritePointer++]= gbInterruptCheckError;
					gbpTxD0Buffer[gbTxD0BufferWritePointer++]= bCheckSum;


					while(GPIO_ReadOutputDataBit(PORT_ENABLE_TXD, PIN_ENABLE_TXD) == Bit_SET);

					GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Disable
					GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Enable

					USART_SendData(USART1, gbpTxD0Buffer[gbTxD0BufferReadPointer++]);
					USART_ITConfig(USART1, USART_IT_TC, ENABLE);
*/




			  }
        }
        else if(gbInstruction == INST_SYSTEM_WRITE)
        { //[Addr] [Data] [0xf0] [0x55] [0x0f] [0xaa]
          if( (gbpParameter[2] == 0xf0) && (gbpParameter[3] == 0x55) &&
              (gbpParameter[4] == 0x0f) && (gbpParameter[5] == 0xaa) &&
              (gbParameterLength == 6) )
          {
            if (gbStartAddress < CONTROL_TABLE_LEN) {
            	BKP_WriteBackupRegister((gbStartAddress+1)<<2, gbpParameter[1]);
            	//ROM_CAST(gbStartAddress) = gbpControlTable[gbStartAddress] = gbpParameter[1];
            }
            else {
            	//BKP_WriteBackupRegister(gbStartAddress<<2, gbpParameter[1]);
            	//ROM_CAST(gbStartAddress) = gbpParameter[1];
            }
          }
        }
        else if(gbInstruction == INST_PING)
        {
          if(gbRxID == BROADCASTING_ID) //for avoiding data crush
          {
//            MiliSec((word)(GB_ID<<1));
        	  mDelay((word)(GB_ID<<0)); //Ver0x14
          }
          ReturnPacket(0);
        }
        else if(gbInstruction == INST_RESET)
        {
          ReturnPacket(0);

          //EEPROM_Write( P_OPERATING_MODE, 0xFF );
          //EEP_GB_OPERATING_MODE = 0xff;
          SYSTEM_RESET;
        }
        else if(gbInstruction == INST_DIGITAL_RESET)
        {
          ReturnPacket(0);
          //EEPROM_Write( P_OPERATING_MODE, 0x11 );
          //EEP_GB_OPERATING_MODE = 0x11;
          SYSTEM_RESET;
          //MiliSec(20);
        }
        else
        {
          ReturnPacket(INSTRUCTION_ERROR_BIT);
        }
      }
    }
  }
}


void WriteControlTable(void)
{
  byte bCount, bPointer;
  for(bCount = 1; bCount < gbParameterLength; bCount++) //Writing
  //bCount = 0 : address value
  {
    bPointer = gbStartAddress+bCount-1;
    if(gbpDataSize[bPointer] == 2) //&& bCount < gbParameterLength-2) //length was already checked.
    {

      WORD_CAST(gbpControlTable[bPointer]) = WORD_CAST(gbpParameter[bCount]);
      if(bPointer < ROM_CONTROL_TABLE_LEN) BKP_WriteBackupRegister((bPointer+1)<<2, WORD_CAST(gbpParameter[bCount]));
      bCount++;

/*
      if(bPointer < ROM_CONTROL_TABLE_LEN) { WORD_ROM_CAST((bPointer)) = WORD_CAST(gbpParameter[bCount]);}
      if(bPointer < ROM_CONTROL_TABLE_LEN) { WORD_CAST(gbpControlTable[bPointer]) = WORD_ROM_CAST((bPointer))+1;}
      else {
        WORD_CAST(gbpControlTable[bPointer]) = WORD_CAST(gbpParameter[bCount]);
      }
      bCount++;
*/
    }
    else //if(gbpDataSize[bPointer] == 1)//length was already checked.
    {
      gbpControlTable[bPointer] = gbpParameter[bCount];
      if(bPointer < ROM_CONTROL_TABLE_LEN) BKP_WriteBackupRegister((bPointer+1)<<2, WORD_CAST(gbpParameter[bCount]));
    }
  }
}

byte WriteControlTableRangeCheck(void)
{
  byte bCount, bPointer;

  if(gbpDataSize[gbStartAddress] == 0 || gbpDataSize[gbStartAddress+gbParameterLength-2] == 2) return RANGE_ERROR_BIT;

  for(bCount = 1; bCount < gbParameterLength; bCount++) //Range Check
  {
    bPointer = gbStartAddress+bCount-1;

    if(bPointer > CONTROL_TABLE_LEN ||
       gbpParameterRange[bPointer][LOW_LIMIT] > gbpParameter[bCount] ||
       gbpParameter[bCount] > gbpParameterRange[bPointer][HIGH_LIMIT])
    {
      return RANGE_ERROR_BIT;
    }
    //else if(bPointer == P_GOAL_POSITION_L)
  }
  return 0;
}


void ReturnPacket(byte bError)
{
  byte bCheckSum;
  //if(GB_OPERATING_MODE == ANALOG_MODE) return;
  if(gbInstruction == INST_PING || (gbRxID != BROADCASTING_ID && GB_RETURN_LEVEL >= RETURN_ALL_PACKET))
  {
    bError |= gbInterruptCheckError;
    //Call routine processing item : InstructionError,ChecksumError,TimeoutError
    bCheckSum = ~(GB_ID+ 2 + bError);
//    RS485_TXD;

    gbpTxD1Buffer[gbTxD1BufferWritePointer++] = 0xff;
    gbpTxD1Buffer[gbTxD1BufferWritePointer++] = 0xff;
    gbpTxD1Buffer[gbTxD1BufferWritePointer++] = GB_ID;
    gbpTxD1Buffer[gbTxD1BufferWritePointer++] = 2;
    gbpTxD1Buffer[gbTxD1BufferWritePointer++] = bError;
    gbpTxD1Buffer[gbTxD1BufferWritePointer++] = bCheckSum;

    if (gbTxD1Transmitting==0) {
      gbTxD1Transmitting = 1;
    //  if (TXD1_FINISH) {
		USART_SendData(USART3, gbpTxD1Buffer[gbTxD1BufferReadPointer++]);
		USART_ITConfig(USART3, USART_IT_TC, ENABLE);
      //TXD1_DATA = gbpTxD1Buffer[gbTxD1BufferReadPointer++];
    }

    while(gbTxD1Transmitting);

  }
}


void ProcessAfterWriting(void)
{
  byte bCount, bComplianceFlag;
  word wTemp;
  u32 lTemp;

  bComplianceFlag = 0;
  for(bCount = 0; bCount < gbParameterLength-1; bCount++) //Range Check
  {
    switch(gbStartAddress+bCount)
    {

    	case P_BAUD_RATE:
			//EEPROM_Write(P_BAUD_RATE,GB_BAUD_RATE);
    		lTemp = 2000000;
			lTemp /= (GB_BAUD_RATE+1);
			USART_Configuration(USART_DXL,lTemp);
			USART_Configuration(USART_PC,lTemp);

		break;

		case	P_DYNAMIXEL_POWER:
				dxl_set_power(GB_DYNAMIXEL_POWER);
				if(GB_DYNAMIXEL_POWER == ON)
				{
					enableDXLForwarding();
				}
				gbDxlPwr = GB_DYNAMIXEL_POWER;
		break;
		case	P_LED_PANNEL:
				LED_SetState(GB_LED_MODE, ON);
				LED_SetState(~GB_LED_MODE, OFF);
		break;
		case	P_LED_HEAD:
		    gbLEDHeadR = GW_LED_HEAD&0x1f;
		    gbLEDHeadG = (GW_LED_HEAD>>5)&0x1f;
		    gbLEDHeadB = (GW_LED_HEAD>>10)&0x1f;
		break;

		case	P_LED_EYE:
		    gbLEDEyeR = GW_LED_EYE&0x1f;
		    gbLEDEyeG = (GW_LED_EYE>>5)&0x1f;
		    gbLEDEyeB = (GW_LED_EYE>>10)&0x1f;
		break;

		case	P_BUZZER_DATA0:

			setBuzzerPlayLength(GB_BUZZER_DATA1);

			if( getBuzzerState() == 0 )
			{
				setBuzzerData(GB_BUZZER_DATA0);
				PlayBuzzer();
			}
			else
			if( GB_BUZZER_DATA1 == 0xFE )
			{
				setBuzzerData(GB_BUZZER_DATA0);
				PlayBuzzer();
			}
		break;

		case	P_BUZZER_DATA1:
			if( GB_BUZZER_DATA1 == 0x00 )
			{
				setBuzzerOff();
			}
		break;

		case	P_TX_REMOCON_DATA_L:
				zgb_tx_data(GW_TX_REMOCON_DATA);
				//TxDData(USART_ZIGBEE,'a');

				break;

		/*
		case	P_GPIO_MODE:
		{
			wTemp = (word)(GB_GPIO_MODE);
            wTemp = ((wTemp&(0x0010|0x0008))<<7)|((wTemp&(0x0004|0x0002|0x0001))<<13);

			GPIO_InitTypeDef GPIO_InitStructure;
			GPIO_StructInit(&GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = wTemp;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
          	GPIO_Init(PORT_OLLO0 , &GPIO_InitStructure);

          	wTemp = ~wTemp;
          	wTemp = (wTemp&(0x0800|0x0400))|(wTemp&(0x8000|0x4000|0x2000));
			GPIO_InitStructure.GPIO_Pin = wTemp;
          	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
			GPIO_Init(PORT_OLLO0 , &GPIO_InitStructure);
		}

		break;
		case	P_GPIO_OUT:
		{
			//GPIO_InitTypeDef GPIO_InitStructure;
          	//GPIO_StructInit(&GPIO_InitStructure);
            //GPIO_InitStructure.GPIO_Pin = PIN_OLLO3 | PIN_OLLO4 | PIN_OLLO0 | PIN_OLLO1 | PIN_OLLO2 ;
        	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
          	//GPIO_Init(GPIOC , &GPIO_InitStructure);
            wTemp = (word)(GB_GPIO_OUT);
            wTemp = ((wTemp&(0x0010|0x0008))<<7)|((wTemp&(0x0004|0x0002|0x0001))<<13);
            GPIO_Write(PORT_OLLO0, wTemp);

            if (GB_GPIO_OUT&0x10) 	GPIO_SetBits(PORT_OLLO4, PIN_OLLO4);
			else					GPIO_ResetBits(PORT_OLLO4, PIN_OLLO4);
			if (GB_GPIO_OUT&0x08) 	GPIO_SetBits(PORT_OLLO3, PIN_OLLO3);
			else					GPIO_ResetBits(PORT_OLLO3, PIN_OLLO3);
			if (GB_GPIO_OUT&0x04) 	GPIO_SetBits(PORT_OLLO2, PIN_OLLO2);
			else					GPIO_ResetBits(PORT_OLLO2, PIN_OLLO2);
			if (GB_GPIO_OUT&0x02) 	GPIO_SetBits(PORT_OLLO1, PIN_OLLO1);
			else					GPIO_ResetBits(PORT_OLLO1, PIN_OLLO1);
			if (GB_GPIO_OUT&0x01) 	GPIO_SetBits(PORT_OLLO0, PIN_OLLO0);
			else					GPIO_ResetBits(PORT_OLLO0, PIN_OLLO0);

		}
		break;
*/
      default:
        break;
    }
  }

}

