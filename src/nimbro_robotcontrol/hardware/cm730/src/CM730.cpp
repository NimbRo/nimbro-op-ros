//cm730 Communication
//Author: Sebastian Schüller

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <stdio.h>

#include <ros/ros.h>

#include "CM730.h"
#include "dynamixel.h"

#define DEBUG 0
#define BULKREAD_DETAILED_TIMESTAMPS 0

const char* CM730::PATH = "/dev/cm730";

inline void getabs_timeout(struct timespec* out)
{
	clock_gettime(CLOCK_MONOTONIC, out);
	out->tv_sec += (out->tv_nsec + 7000000LL) / 1000000000LL;
	out->tv_nsec = (out->tv_nsec + 7000000LL) % 1000000000LL;
}

inline int makeBRWord(unsigned char* pos)
{
	return ((pos[1] << 8) | (pos[0]));
}

void dump(const char* prefix, const uint8_t* data, uint8_t len)
{
	printf("%s:", prefix);

	for(int i = 0; i < len; ++i)
	{
		printf(" %02X", data[i]);
	}
	printf("\n");
}


CM730::CM730()
 : m_fd(-1)
 , m_lastFailedID(0)
{
	memset(&m_TxBulkRead, 0, sizeof(m_TxBulkRead));
	setQuerySet(std::vector<int>());
}

CM730::~CM730()
{

}

int CM730::connect()
{
	struct termios config;
	struct serial_struct serinfo;

	if ((m_fd = open(PATH, O_RDWR | O_NOCTTY)) < 0)
	{
		perror("[CM730] Could not open serial connection");
		return -1;
	}

	if(lockf(m_fd, F_TLOCK, 0) != 0)
	{
		perror("[CM730] Could not acquire serial lock (is another robotcontrol running?)");
		return -1;
	}

	memset(&config, 0, sizeof(config));

	config.c_cflag      = B38400|CS8|CLOCAL|CREAD;
	config.c_iflag      = IGNPAR;
	config.c_oflag      = 0;
	config.c_lflag      = 0;
	config.c_cc[VTIME]  = 0;
	config.c_cc[VMIN]   = 1;

	if((tcsetattr(m_fd, TCSANOW, &config)) < 0)
	{
		perror("[CM730] Could not set terminal attributes");

		close(m_fd);
		m_fd = -1;
		return -2;
	}

	if(ioctl(m_fd, TIOCGSERIAL, &serinfo) < 0)
	{
		perror("[CM730] Could not get serial attributes");

		close(m_fd);
		m_fd = -1;
		return -2;
	}

	serinfo.flags |= ASYNC_LOW_LATENCY;
	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = serinfo.baud_base / BAUDRATE;

	if(ioctl(m_fd, TIOCSSERIAL, &serinfo) < 0)
	{
		perror("[CM730] Could not set serial attributes");

		close(m_fd);
		m_fd = -1;
		return -2;
	}

// 	if(fcntl(m_fd, F_SETFL, O_NONBLOCK) != 0)
// 	{
// 		perror("[CM730] Could not switch to non-blocking I/O");
// 		return -2;
// 	}

	m_readThread.setFile(m_fd);
	pthread_create(&m_readThread_thread, 0, &io::ReadThread::start, &m_readThread);

	flushPort();
	return 0;
}

int CM730::ping(int id)
{
	unsigned char txp[MAX_TXPAR] = {0, };
	unsigned char rxp[MAX_RXPAR + 10] = {0, };
	rxp[INSTRUCTION] = INST_PING;

	txp[ID] = (unsigned char)id;
	txp[INSTRUCTION] = INST_PING;
	txp[LENGTH] = 2;

	if (txPacket(txp) != SUCCESS)
	{
		return TX_FAIL;
	}

	int rxError;
	if ((rxError = rxPacket(rxp, 6)) != SUCCESS)
	{
		return rxError;
	}
	else
	{
		return rxp[ERRBIT];
	}
}

int CM730::readByte(int id, int address, int* value)
{
	unsigned char txpacket[MAX_TXPAR] = {0, };
	unsigned char rxpacket[MAX_RXPAR] = {0, };

	txpacket[ID]            = (unsigned char)id;
	txpacket[INSTRUCTION]   = INST_READ;
	txpacket[PARAMETER]     = (unsigned char)address;
	txpacket[PARAMETER + 1] = 1;
	txpacket[LENGTH]        = 4;

	if(txPacket(txpacket) != SUCCESS)
		return TX_FAIL;

	if(rxPacket(rxpacket, 7) != SUCCESS)
		return RX_FAIL;

	*value = (int)rxpacket[PARAMETER];
	return SUCCESS;

}

int CM730::readWord(int id, int address, int* value)
{
	unsigned char txpacket[MAX_TXPAR] = {0, };
	unsigned char rxpacket[MAX_RXPAR] = {0, };

	txpacket[ID]            = (unsigned char)id;
	txpacket[INSTRUCTION]   = INST_READ;
	txpacket[PARAMETER]     = (unsigned char)address;
	txpacket[PARAMETER + 1] = 2;
	txpacket[LENGTH]        = 4;

	if (txPacket(txpacket) != SUCCESS)
		return TX_FAIL;

	if (rxPacket(rxpacket, 8) != SUCCESS)
		return RX_FAIL;

	*value = (int)rxpacket[PARAMETER] | ((int)rxpacket[PARAMETER + 1] << 8);
	return SUCCESS;
}

int CM730::readData(int id, int address, void* data, size_t size)
{
	unsigned char txpacket[MAX_TXPAR] = {0, };
	unsigned char rxpacket[MAX_RXPAR] = {0, };

	txpacket[ID]            = (unsigned char)id;
	txpacket[INSTRUCTION]   = INST_READ;
	txpacket[PARAMETER]     = (unsigned char)address;
	txpacket[PARAMETER + 1] = size;
	txpacket[LENGTH]        = 4;

	if (txPacket(txpacket) != SUCCESS)
		return TX_FAIL;

	if (rxPacket(rxpacket, 8) != SUCCESS)
		return RX_FAIL;

	memcpy(data, rxpacket + PARAMETER, size);
	return SUCCESS;
}

int CM730::writeByte(int id, int address, int value)
{
	unsigned char txpacket[MAX_TXPAR + 10] = {0, };
	unsigned char rxpacket[MAX_RXPAR] = {0, };

	txpacket[ID]           = (unsigned char)id;
	txpacket[INSTRUCTION]  = INST_WRITE;
	txpacket[PARAMETER]    = (unsigned char)address;
	txpacket[PARAMETER+1]  = (unsigned char)value;
	txpacket[LENGTH]       = 4;

	if(txPacket(txpacket) != SUCCESS)
		return TX_FAIL;

	if(rxPacket(rxpacket, 6) != SUCCESS)
		return RX_FAIL;

	return rxpacket[ERRBIT];
}

int CM730::writeWord(int id, int address, int value)
{
	unsigned char txpacket[MAX_TXPAR + 10] = {0, }; 

	txpacket[ID]           = (unsigned char)id;
	txpacket[INSTRUCTION]  = INST_WRITE;
	txpacket[PARAMETER]    = (unsigned char)address;
	txpacket[PARAMETER+1]  = (unsigned char)(value & 0x00ff);
	txpacket[PARAMETER+2]  = (unsigned char)(value & 0xff00);
	txpacket[LENGTH]       = 5;

	return txPacket(txpacket);
}

int CM730::syncWrite(int startAddress, int paramLength, int paramNumber, const uint8_t* param)
{
	unsigned char txpacket[MAX_TXPAR + 10] = {0, };

	txpacket[ID]            = ID_BROADCAST;
	txpacket[INSTRUCTION]   = INST_SYNC_WRITE;
	txpacket[PARAMETER]     = (unsigned char)startAddress;
	txpacket[PARAMETER + 1] = (unsigned char)paramLength - 1;

	memcpy(txpacket + PARAMETER + 2, param, paramNumber * paramLength);

	txpacket[LENGTH]        = paramNumber*paramLength + 4;

	return txPacket(txpacket);
}

int CM730::bulkRead(std::vector<BRData>* data, BRBoard* cmdata)
{
#if BULKREAD_DETAILED_TIMESTAMPS
	std::vector<ros::Time> ts;
	ts.push_back(ros::Time::now());
#endif

	unsigned char rxpacket[MAX_RXPAR + 10] = {0, };

	struct timespec timeout_time;
	getabs_timeout(&timeout_time);

#ifndef NDEBUG
	int length = m_TxBulkRead[LENGTH] + 4;
	assert(length < MAX_TXPAR + 6);
#endif

	//Send BulkRead TxPacket
	if (txPacket(m_TxBulkRead) < 0)
		return TX_FAIL;

#if BULKREAD_DETAILED_TIMESTAMPS
	ts.push_back(ros::Time::now());
#endif


	// Give OS some time
	usleep(4 * 1000);

	int numberOfRequests = (m_TxBulkRead[LENGTH] - 3) / 3;

	//write data headers
	int i;
	for (i = 0; i < 3 * numberOfRequests; i += 3)
	{
		int len  = m_TxBulkRead[PARAMETER + i + 1];
		int id   = m_TxBulkRead[PARAMETER + i + 2];
		int addr = m_TxBulkRead[PARAMETER + i + 3];

		if (id == ID_CM)
		{
			cmdata->id = id;
			cmdata->length = len;
			cmdata->startAddress = addr;
		}
		else
		{
			BRData* servoInfo = &data->at(id - 1); //shift to 0

			servoInfo->id = ID;
			servoInfo->length = len;
			servoInfo->startAddress = addr;
		}
	}

	m_lastFailedID = 0;

	for(int i = 0; i < numberOfRequests; ++i)
	{
		int id = m_TxBulkRead[PARAMETER + 3*i + 2];
		int len = m_TxBulkRead[PARAMETER + 3*i + 1];
		if(len + 6 >= (int)sizeof(rxpacket))
		{
			ROS_ERROR_THROTTLE(0.1, "Invalid read size for ID %d: %d", id, len + 6);
			return RX_CORRUPT;
		}
		int rxError = rxPacket(rxpacket, len + 6, &timeout_time);
		if(rxError != SUCCESS)
		{
#if BULKREAD_DETAILED_TIMESTAMPS
			ts.push_back(ros::Time::now());
#endif
// 			ROS_ERROR_THROTTLE(0.1, "Could not read answer for ID %d: error %d", id, rxError);

			if(id != ID_CM)
				m_lastFailedID = id;

#if BULKREAD_DETAILED_TIMESTAMPS
			if(ts.back() - ts[0] > ros::Duration(0.008))
			{
				ROS_WARN("total time: %lf", (ts.back() - ts[0]).toSec());
				for(size_t i = 1; i < ts.size(); ++i)
				{
					ROS_WARN("delta: %lf (%lf)", (ts[i] - ts[i-1]).toSec(), (ts[i] - ts[0]).toSec());
				}
			}
#endif

			return rxError;
		}

#if BULKREAD_DETAILED_TIMESTAMPS
		ts.push_back(ros::Time::now());
#endif
		parseBRPacket(rxpacket, len + 6, data, cmdata);
#if BULKREAD_DETAILED_TIMESTAMPS
		ts.push_back(ros::Time::now());
#endif
	}

	return SUCCESS;
}

int CM730::txPacket(unsigned char* txpacket)
{
	int length = txpacket[LENGTH] + 4;
	txpacket[0] = 0xff;
	txpacket[1] = 0xff;
	txpacket[length -1] = checksum(txpacket);

	flushPort();
#if DEBUG
	dump("TX", txpacket, length);
#endif
	if (write(m_fd, txpacket, length) == length)
	{
		flushPort();
		return SUCCESS;
	}
	else
		return TX_FAIL;
}

int CM730::rxPacket(unsigned char* rxpacket, int size, struct timespec* abstime)
{
	int readErr, readSize = 0;

	struct timespec timeout;

	if(!abstime)
	{
		getabs_timeout(&timeout);
		abstime = &timeout;
	}

	while(1)
	{
		readErr = m_readThread.read(rxpacket + readSize, size - readSize, abstime);

		if(readErr == -ETIMEDOUT)
			return RX_TIMEOUT;

		if (readErr < 0)
		{
			return RX_FAIL;
		}

#if DEBUG
		dump("RX", rxpacket + readSize, readErr);
#endif
		readSize += readErr;
		syncRxPackets(rxpacket, &readSize);

		if (readSize >= size)
		{
#if DEBUG
			dump("RX packet", rxpacket, readSize);
#endif
			int txChecksum = checksum(rxpacket);

			if(rxpacket[LENGTH] != size - 4)
			{
				ROS_ERROR_THROTTLE(0.1, "Wrong length: %d (expected %d)", rxpacket[LENGTH], size-4);
				return RX_CORRUPT;
			}

			if(txChecksum != rxpacket[LENGTH + rxpacket[LENGTH]])
			{
				ROS_ERROR_THROTTLE(0.1, "Wrong checksum: %d (expected %d)", rxpacket[LENGTH + rxpacket[LENGTH]], txChecksum);
				return RX_CORRUPT;
			}

			return SUCCESS;
		}
	}
}

unsigned char CM730::checksum(unsigned char* txpacket)
{
	unsigned char checksum = 0x00;
	int i;
	for (i = 2; i < txpacket[LENGTH] + 3; i++)
		checksum += txpacket[i];
	return (~checksum);
}
     
void CM730::flushPort()
{
	m_readThread.flush();
// 	tcflush(m_fd, TCIFLUSH);
}

void CM730::parseBRPacket(unsigned char* pos, int maxLength, std::vector<BRData>* data, BRBoard* cmdata)
{
	int start;
	if (pos[ID] == ID_CM)
	{
		start = cmdata->startAddress;

		int offset = PARAMETER - start;
		cmdata->power     = pos[offset + P_DXL_POWER];
		cmdata->button    = pos[offset + P_BUTTON];
		cmdata->voltage   = pos[offset + P_VOLTAGE];
		cmdata->ledPannel = pos[offset + P_LED_PANNEL];

		cmdata->compassX = *((int16_t*)(pos + offset + P_CMP_X_L));
		cmdata->compassY = *((int16_t*)(pos + offset + P_CMP_Y_L));
		cmdata->compassZ = *((int16_t*)(pos + offset + P_CMP_Z_L));

		cmdata->gyroX = makeBRWord(pos + offset + P_GYRO_X_L);
		cmdata->gyroY = makeBRWord(pos + offset + P_GYRO_Y_L);
		cmdata->gyroZ = makeBRWord(pos + offset + P_GYRO_Z_L);

		cmdata->accelX = makeBRWord(pos + offset + P_ACCEL_X_L);
		cmdata->accelY = makeBRWord(pos + offset + P_ACCEL_Y_L);
		cmdata->accelZ = makeBRWord(pos + offset + P_ACCEL_Z_L);

		cmdata->ledEye  = makeBRWord(pos + offset + P_LED_EYE_L);
		cmdata->ledHead = makeBRWord(pos + offset + P_LED_HEAD_L);
	}
	else
	{
		BRData* servoInfo = &data->at(pos[ID] - 1); //shift to zero
		start = servoInfo->startAddress;

		int offset = PARAMETER - start;
		servoInfo->position = makeBRWord(pos + offset + DynamixelMX::P_PRESENT_POSITION_L);
	}
}

void CM730::setQuerySet(const std::vector< int >& servos)
{
	int num = 1;

	m_TxBulkRead[ID]              = (unsigned char)ID_BROADCAST;
	m_TxBulkRead[INSTRUCTION]     = INST_BULK_READ;
	m_TxBulkRead[PARAMETER]       = (unsigned char)0x0;

	m_TxBulkRead[PARAMETER + num++] = 30;
	m_TxBulkRead[PARAMETER + num++] = ID_CM;
	m_TxBulkRead[PARAMETER + num++] = P_DXL_POWER;

	for(size_t i = 0; i < servos.size(); ++i)
	{
		m_TxBulkRead[PARAMETER + num++] = 2; // length
		m_TxBulkRead[PARAMETER + num++] = servos[i]; //ID
		m_TxBulkRead[PARAMETER + num++] = DynamixelMX::P_PRESENT_POSITION_L; // start Address
	}

	m_TxBulkRead[LENGTH] = num + 2;
}

void CM730::syncRxPackets(unsigned char* rxpacket, int* readSize)
{
	if (rxpacket[0] == 0xFF && rxpacket[1] == 0xFF)
		return;

	if(*readSize < 3)
		return;

	if (rxpacket[*readSize - 1] == 0xFF && rxpacket[*readSize - 2] != 0xFF)
	{
		rxpacket[0] = 0xFF;
		*readSize = 1;
		return;
	}

	int i;
	for (i = 0; i < (*readSize - 1); i++)
	{
		if (rxpacket[i] == 0xFF && rxpacket[i + 1] == 0xFF)
			break;
	}
	size_t copySize = *readSize - i;
	memmove(rxpacket, rxpacket + i, copySize);
	*readSize = copySize;
	return;
}
