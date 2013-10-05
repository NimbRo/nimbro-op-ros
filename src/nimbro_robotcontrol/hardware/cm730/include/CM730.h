//cm730_comm.h
//author: Sebastian Schüller

#ifndef CM730_COMM_H
#define CM730_COMM_H

#include <stdint.h>
#include <ros/time.h>
#include "read_thread.h"

struct BRData
{
	int id;
	int length;
	int startAddress;

	int position;
};

struct BRBoard
{
	int id;
	int length;
	int startAddress;

	unsigned char power;

	int compassX;
	int compassY;
	int compassZ;

	int gyroX;
	int gyroY;
	int gyroZ;

	int accelX;
	int accelY;
	int accelZ;

	unsigned char voltage;
	unsigned char ledPannel;
	int ledHead;
	int ledEye;

	unsigned char button;
};


class CM730
{
public:

	CM730();
	~CM730();

	int syncWrite(int startAddress, int paramLength, int paramNumber, const uint8_t* param);
	int bulkRead(std::vector<BRData>* data, BRBoard* cmdata);

	//use SyncWrite if possible
	int writeByte(int id, int address, int value);
	int writeWord(int id, int address, int value);

	int readByte(int id, int address, int* value);
	int readWord(int id, int address, int *value);
	int readData(int id, int address, void* data, size_t size);

	int ping(int id);

	int connect();

	void setQuerySet(const std::vector<int>& servos);

	inline int lastFailedID() const
	{ return m_lastFailedID; }
private:

	static const char* PATH;
	static const int BAUDRATE = 1000000; // 1 Mbps
	static const int MAX_TXPAR = 256;
	static const int MAX_RXPAR = 1024;

	int m_fd;
	unsigned char m_TxBulkRead[MAX_TXPAR + 10];

	int m_lastFailedID;

	io::ReadThread m_readThread;
	pthread_t m_readThread_thread;

	int txPacket(unsigned char* txpacket);
	int rxPacket(unsigned char* rxpacket, int size, struct timespec* abstime = 0);

	unsigned char checksum(unsigned char* txpacket);
	void flushPort();

	void parseBRPacket(unsigned char* pos, int maxLength, std::vector<BRData>* data, BRBoard* cmdata);

	void syncRxPackets(unsigned char* rxpacket, int* readSize);


public:
	enum
	{
		ID		= 2,
		LENGTH		= 3,
		INSTRUCTION	= 4,
		ERRBIT		= 4,
		PARAMETER	= 5,
	};

	enum
	{
		INST_PING	= 1,
		INST_READ,
		INST_WRITE,
		INST_REG_WRITE,
		INST_ACTION,
		INST_RESET,
		INST_SYNC_WRITE	= 131,
		INST_BULK_READ	= 146
	};

	enum
	{
		SUCCESS,
		TX_CORRUPT,
		TX_FAIL,
		RX_FAIL,
		RX_TIMEOUT,
		RX_CORRUPT
	};

// 	enum
// 	{
// 		INPUT_VOLTAGE   = 1,
// 		ANGLE_LIMIT     = 2,
// 		OVERHEATING     = 4,
// 		RANGE           = 8,
// 		CHECKSUM        = 16,
// 		OVERLOAD        = 32,
// 		INSTRUCTION     = 64
// 	};


	enum
	{
		P_MODEL_NUMBER_L	= 0,
		P_MODEL_NUMBER_H	= 1,
		P_VERSION		= 2,
		P_ID			= 3,
		P_BAUD_RATE		= 4,
		P_RETURN_DELAY_TIME	= 5,
		P_RETURN_LEVEL		= 16,
		P_DXL_POWER		= 24,
		P_LED_PANNEL		= 25,
		P_LED_HEAD_L		= 26,
		P_LED_HEAD_H		= 27,
		P_LED_EYE_L		= 28,
		P_LED_EYE_H		= 29,
		P_BUTTON		= 30,
		P_CMP_X_L       = 32,
		P_CMP_Y_L       = 34,
		P_CMP_Z_L       = 36,
		P_GYRO_Z_L		= 38,
		P_GYRO_Z_H		= 39,
		P_GYRO_Y_L		= 40,
		P_GYRO_Y_H		= 41,
		P_GYRO_X_L		= 42,
		P_GYRO_X_H		= 43,
		P_ACCEL_X_L		= 44,
		P_ACCEL_X_H		= 45,
		P_ACCEL_Y_L		= 46,
		P_ACCEL_Y_H		= 47,
		P_ACCEL_Z_L		= 48,
		P_ACCEL_Z_H		= 49,
		P_VOLTAGE		= 50,
		P_LEFT_MIC_L		= 51,
		P_LEFT_MIC_H		= 52,
		P_ADC2_L		= 53,
		P_ADC2_H		= 54,
		P_ADC3_L		= 55,
		P_ADC3_H		= 56,
		P_ADC4_L		= 57,
		P_ADC4_H		= 58,
		P_ADC5_L		= 59,
		P_ADC5_H		= 60,
		P_ADC6_L		= 61,
		P_ADC6_H		= 62,
		P_ADC7_L		= 63,
		P_ADC7_H		= 64,
		P_ADC8_L		= 65,
		P_ADC8_H		= 66,
		P_RIGHT_MIC_L		= 67,
		P_RIGHT_MIC_H		= 68,
		P_ADC10_L		= 69,
		P_ADC10_H		= 70,
		P_ADC11_L		= 71,
		P_ADC11_H		= 72,
		P_ADC12_L		= 73,
		P_ADC12_H		= 74,
		P_ADC13_L		= 75,
		P_ADC13_H		= 76,
		P_ADC14_L		= 77,
		P_ADC14_H		= 78,
		P_ADC15_L		= 79,
		P_ADC15_H		= 80,
		MAXNUM_ADDRESS
	};

	enum
	{
		ID_CM		= 200,
		ID_BROADCAST	= 254
	};

};

#endif