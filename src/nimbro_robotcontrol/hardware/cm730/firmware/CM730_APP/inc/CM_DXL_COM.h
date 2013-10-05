
/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : CM_DXL_COM.h
* Author             : dudung
* Version            : V0.1
* Date               : 2010/11/03
* Description        : This file contains the inteligent toss mode between
                       host and dynamixel.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DYNAMIXEL_CM_DXL_COM_HEADER
#define _DYNAMIXEL_CM_DXL_COM_HEADER

/* Includes ------------------------------------------------------------------*/
#include "common_type.h"
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
//EEPROM AREA
#define P_MODEL_NUMBER_L      0
#define P_MODOEL_NUMBER_H     1
#define P_VERSION             2
#define P_ID                  3
#define P_BAUD_RATE           4
#define P_RETURN_DELAY_TIME   5
#define P_CW_ANGLE_LIMIT_L    6
#define P_CW_ANGLE_LIMIT_H    7
#define P_CCW_ANGLE_LIMIT_L   8
#define P_CCW_ANGLE_LIMIT_H   9
#define P_SYSTEM_DATA2        10
#define P_LIMIT_TEMPERATURE   11
#define P_DOWN_LIMIT_VOLTAGE  12
#define P_UP_LIMIT_VOLTAGE    13
#define P_MAX_TORQUE_L        14
#define P_MAX_TORQUE_H        15
#define P_RETURN_LEVEL        16
#define P_ALARM_LED           17
#define P_ALARM_SHUTDOWN      18
#define P_OPERATING_MODE      19
#define P_DOWN_CALIBRATION_L  20
#define P_DOWN_CALIBRATION_H  21
#define P_UP_CALIBRATION_L    22
#define P_UP_CALIBRATION_H    23
/*
#define P_TORQUE_ENABLE         (24)
#define P_LED                   (25)
#define P_CW_COMPLIANCE_MARGIN  (26)
#define P_CCW_COMPLIANCE_MARGIN (27)
#define P_CW_COMPLIANCE_SLOPE   (28)
#define P_CCW_COMPLIANCE_SLOPE  (29)
#define P_GOAL_POSITION_L       (30)
#define P_GOAL_POSITION_H       (31)
#define P_GOAL_SPEED_L          (32)
#define P_GOAL_SPEED_H          (33)
#define P_TORQUE_LIMIT_L        (34)
#define P_TORQUE_LIMIT_H        (35)
#define P_PRESENT_POSITION_L    (36)
#define P_PRESENT_POSITION_H    (37)
#define P_PRESENT_SPEED_L       (38)
#define P_PRESENT_SPEED_H       (39)
#define P_PRESENT_LOAD_L        (40)
#define P_PRESENT_LOAD_H        (41)
#define P_PRESENT_VOLTAGE       (42)
#define P_PRESENT_TEMPERATURE   (43)
#define P_REGISTERED_INSTRUCTION (44)
#define P_PAUSE_TIME            (45)
#define P_MOVING (46)
#define P_LOCK                  (47)
#define P_PUNCH_L               (48)
#define P_PUNCH_H               (49)
#define P_RESERVED4             (50)
#define P_RESERVED5             (51)
#define P_POT_L                 (52)
#define P_POT_H                 (53)
#define P_PWM_OUT_L             (54)
#define P_PWM_OUT_H             (55)
*/

#define	P_DYNAMIXEL_POWER	24
#define	P_LED_PANNEL	25
#define	P_LED_HEAD	26
//#define	-	27
#define	P_LED_EYE	28
//#define	-	29
#define	P_BUTTON	30
//#define     -		31
//#define     -		32
//#define     -		33
//#define     -		34
//#define     -		35
//#define     -		36
//#define     -		37

//#define	P_GPIO_MODE	31
//#define	P_GPIO_OUT	32
//#define	P_GPIO_IN	33
//#define	P_ADC0_VOLTAGE	34
//#define	-	35
//#define	P_ADC1_RESERVED	36
//#define	-	37
#define	P_GYRO_Z	38
//#define	-	39
#define	P_GYRO_Y	40
//#define	-	41
#define	P_GYRO_X	42
//#define	-	43
#define	P_ACC_X	44
//#define	-	45
#define	P_ACC_Y	46
//#define	-	47
#define	P_ACC_Z	48
//#define	-	49
#define	P_ADC0_VOLTAGE	50
#define	P_ADC1_MIC1		51
//#define	-			52
#define	P_ADC9_MIC2		53
//#define	-			54
#define	P_ADC2			55
//#define	-			56
#define	P_ADC3			57
//#define	-			58
#define	P_ADC4			59
//#define	-			60
#define	P_ADC5			61
//#define	-			62
#define	P_ADC6			63
//#define	-			64
#define	P_ADC7			65
//#define	-			66
#define	P_ADC8			67
//#define	-			68
#define	P_ADC10			69
//#define	-			70
#define	P_ADC11			71
//#define	-			72
#define	P_ADC12			73
//#define	-			74
#define	P_ADC13			75
//#define	-			76
#define	P_ADC14			77
//#define	-			78
#define	P_ADC15			79
//#define	-			80
#define P_BUZZER_DATA0	81
#define P_BUZZER_DATA1	82
#define P_TX_REMOCON_DATA_L   		83
//#define P_TX_REMOCON_DATA_H   		84
#define P_RX_REMOCON_DATA_L   		85
//#define P_RX_REMOCON_DATA_H   		86
#define P_RX_REMOCON_DATA_ARRIVED 	87
#define P_ZIGBEE_ID_L         		88
#define P_ZIGBEE_ID_H         		89





#define EEP_GW_MODEL_NUMBER           WORD_ROM_CAST((P_MODEL_NUMBER_L))
#define EEP_GB_GB_VERSION             ROM_CAST(P_VERSION)
#define EEP_GB_ID                     ROM_CAST(P_ID)
#define EEP_GB_BAUD_RATE              ROM_CAST(P_BAUD_RATE)
#define EEP_GB_RETURN_DELAY_TIME      ROM_CAST(P_RETURN_DELAY_TIME)
#define EEP_GW_CW_ANGLE_LIMIT         WORD_ROM_CAST((P_CW_ANGLE_LIMIT_L))
#define EEP_GW_CCW_ANGLE_LIMIT        WORD_ROM_CAST((P_CCW_ANGLE_LIMIT_L))
#define EEP_SYSTEM_DATA2              ROM_CAST(P_SYSTEM_DATA2)
#define EEP_GB_LIMIT_TEMPERATURE      ROM_CAST(P_LIMIT_TEMPERATURE)
#define EEP_GB_UP_LIMIT_VOLTAGE       ROM_CAST(P_UP_LIMIT_VOLTAGE)
#define EEP_GB_DOWN_LIMIT_VOLTAGE     ROM_CAST(P_DOWN_LIMIT_VOLTAGE)
#define EEP_GB_MAX_TORQUE             WORD_ROM_CAST((P_MAX_TORQUE_L))
#define EEP_GB_RETURN_LEVEL           ROM_CAST(P_RETURN_LEVEL)
#define EEP_GB_ALARM_LED              ROM_CAST(P_ALARM_LED)
#define EEP_GB_ALARM_SHUTDOWN         ROM_CAST(P_ALARM_SHUTDOWN)
#define EEP_GB_OPERATING_MODE         ROM_CAST(P_OPERATING_MODE)
#define EEP_GW_DOWN_CALIBRATION       WORD_ROM_CAST((P_DOWN_CALIBRATION_L))
#define EEP_GW_UP_CALIBRATION         WORD_ROM_CAST((P_UP_CALIBRATION_L))

#define EEP_GB_BOOTLOADER_VERSION     ROM_CAST(P_BOOTLOADER_VERSION)

////////////////////////////////////////////////////////////////////////
#define GW_MODEL_NUMBER           WORD_CAST(gbpControlTable[P_MODEL_NUMBER_L])
#define GB_GB_VERSION             gbpControlTable[P_VERSION]
#define GB_ID                     gbpControlTable[P_ID]
#define GB_BAUD_RATE              gbpControlTable[P_BAUD_RATE]
#define GB_RETURN_DELAY_TIME      gbpControlTable[P_RETURN_DELAY_TIME]
#define GW_CW_ANGLE_LIMIT         WORD_CAST(gbpControlTable[P_CW_ANGLE_LIMIT_L])
#define GW_CCW_ANGLE_LIMIT        WORD_CAST(gbpControlTable[P_CCW_ANGLE_LIMIT_L])
#define GB_EEP_SYSTEM_DATA2       gbpControlTable[P_SYSTEM_DATA2]
#define GB_LIMIT_TEMPERATURE      gbpControlTable[P_LIMIT_TEMPERATURE]
#define GB_UP_LIMIT_VOLTAGE       gbpControlTable[P_UP_LIMIT_VOLTAGE]
#define GB_DOWN_LIMIT_VOLTAGE     gbpControlTable[P_DOWN_LIMIT_VOLTAGE]
#define GW_MAX_TORQUE             WORD_CAST(gbpControlTable[P_MAX_TORQUE_L])
#define GB_RETURN_LEVEL           gbpControlTable[P_RETURN_LEVEL]
#define GB_ALARM_LED              gbpControlTable[P_ALARM_LED]
#define GB_ALARM_SHUTDOWN         gbpControlTable[P_ALARM_SHUTDOWN]
#define GB_OPERATING_MODE         gbpControlTable[P_OPERATING_MODE]
#define GW_DOWN_CALIBRATION       WORD_CAST(gbpControlTable[P_DOWN_CALIBRATION_L])
#define GW_UP_CALIBRATION         WORD_CAST(gbpControlTable[P_UP_CALIBRATION_L])
////////////////////////////////////////////////////////////////////////
#define	GB_DYNAMIXEL_POWER	gbpControlTable[P_DYNAMIXEL_POWER]
#define	GB_LED_MODE	gbpControlTable[P_LED_PANNEL]
#define	GW_LED_HEAD	WORD_CAST(gbpControlTable[P_LED_HEAD])
#define	GW_LED_EYE	WORD_CAST(gbpControlTable[P_LED_EYE])
#define	GB_BUTTON	gbpControlTable[P_BUTTON]


#define	GW_GYRO_Z	WORD_CAST(gbpControlTable[P_GYRO_Z])
#define	GW_GYRO_Y	WORD_CAST(gbpControlTable[P_GYRO_Y])
#define	GW_GYRO_X	WORD_CAST(gbpControlTable[P_GYRO_X])

#define	GW_ACC_X	WORD_CAST(gbpControlTable[P_ACC_X])
#define	GW_ACC_Y	WORD_CAST(gbpControlTable[P_ACC_Y])
#define	GW_ACC_Z	WORD_CAST(gbpControlTable[P_ACC_Z])

#define	GB_ADC0_VOLTAGE	gbpControlTable[P_ADC0_VOLTAGE]
#define	GW_ADC1_MIC1	WORD_CAST(gbpControlTable[P_ADC1_MIC1])
#define	GW_ADC2			WORD_CAST(gbpControlTable[P_ADC2])
#define	GW_ADC3			WORD_CAST(gbpControlTable[P_ADC3])
#define	GW_ADC4			WORD_CAST(gbpControlTable[P_ADC4])
#define	GW_ADC5			WORD_CAST(gbpControlTable[P_ADC5])
#define	GW_ADC6			WORD_CAST(gbpControlTable[P_ADC6])
#define	GW_ADC7			WORD_CAST(gbpControlTable[P_ADC7])
#define	GW_ADC8			WORD_CAST(gbpControlTable[P_ADC8])
#define	GW_ADC9_MIC2	WORD_CAST(gbpControlTable[P_ADC9_MIC2])
#define	GW_ADC10		WORD_CAST(gbpControlTable[P_ADC10])
#define	GW_ADC11		WORD_CAST(gbpControlTable[P_ADC11])
#define	GW_ADC12		WORD_CAST(gbpControlTable[P_ADC12])
#define	GW_ADC13		WORD_CAST(gbpControlTable[P_ADC13])
#define	GW_ADC14		WORD_CAST(gbpControlTable[P_ADC14])
#define	GW_ADC15		WORD_CAST(gbpControlTable[P_ADC15])

#define	GB_BUZZER_DATA0					gbpControlTable[P_BUZZER_DATA0]
#define	GB_BUZZER_DATA1					gbpControlTable[P_BUZZER_DATA1]
#define	GW_TX_REMOCON_DATA				WORD_CAST(gbpControlTable[P_TX_REMOCON_DATA_L])
#define	GW_RX_REMOCON_DATA				WORD_CAST(gbpControlTable[P_RX_REMOCON_DATA_L])
#define	GB_RX_REMOCON_DATA_ARRIVED		gbpControlTable[P_RX_REMOCON_DATA_ARRIVED]
#define	GW_ZIGBEE_ID					WORD_CAST(gbpControlTable[P_ZIGBEE_ID_L])


#define HIGH_LIMIT 1
#define LOW_LIMIT 0


#define MAX_PACKET_LENGTH (256)

#define ROM_CONTROL_TABLE_LEN 24
#define RAM_CONTROL_TABLE_LEN 66
#define CONTROL_TABLE_LEN (ROM_CONTROL_TABLE_LEN+RAM_CONTROL_TABLE_LEN)
#define BROADCASTING_ID 0xfe
#define DEFAULT_ID	200

extern volatile byte gbpRxInterruptBuffer[];
extern volatile byte gbpTxInterruptBuffer[];
extern volatile byte gbRxBufferWritePointer;
extern volatile byte gbRxBufferReadPointer;
extern volatile byte gbTxBufferWritePointer;
extern volatile byte gbTxBufferReadPointer;
extern volatile byte gbpRxD0Buffer[];
extern volatile byte gbpTxD0Buffer[];
extern volatile byte gbRxD0BufferWritePointer;
extern volatile byte gbRxD0BufferReadPointer;
extern volatile byte gbTxD0BufferWritePointer;
extern volatile byte gbTxD0BufferReadPointer;
extern volatile byte gbpRxD1Buffer[];
extern volatile byte gbpTxD1Buffer[];
extern volatile byte gbRxD1BufferWritePointer;
extern volatile byte gbRxD1BufferReadPointer;
extern volatile byte gbTxD1BufferWritePointer;
extern volatile byte gbTxD1BufferReadPointer;
extern volatile byte gbTxD0Transmitting;
extern volatile byte gbTxD1Transmitting;
extern volatile byte gbMiliSec;
extern byte ROM_INITIAL_DATA[];
extern volatile byte gbpControlTable[];
extern byte gbLEDBlinkCounter;
extern volatile byte gbLEDHeadR;
extern volatile byte gbLEDHeadG;
extern volatile byte gbLEDHeadB;
extern volatile byte gbLEDEyeR;
extern volatile byte gbLEDEyeG;
extern volatile byte gbLEDEyeB;
extern volatile byte gbLEDPwm;

void Process(void);

void TorqueOff();


#endif /* _DYNAMIXEL_CM_DXL_COM_HEADER */

/************************ (C) COPYRIGHT 2010 ROBOTIS *********END OF FILE******/

