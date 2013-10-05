// IDS uEye camera driver
// Author: Max Schwarz <Max@x-quadraht.de>

#include "camera_ueye.h"

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>

#include <string.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(openplatform, camera_ueye, CameraUEye, nodelet::Nodelet)

const int WIDTH = 1280;
const int HEIGHT = 1024;

template<class T>
static inline
T cap(const T& min, const T& max, const T& value)
{
	if(value < min)
		return min;
	if(value > max)
		return max;
	return value;
}

CameraUEye::CameraUEye()
{
}

CameraUEye::~CameraUEye()
{
}

void CameraUEye::onInit()
{
	ros::NodeHandle nh = getPrivateNodeHandle();

	m_pub_image = getNodeHandle().advertise<sensor_msgs::Image>("image", 10);

	m_cam = 0;
	int ret = is_InitCamera(&m_cam, NULL);
	if(ret != IS_SUCCESS)
	{
		NODELET_FATAL("Camera error");
		return;
	}

	ret = is_SetPixelClock(m_cam, 35);
	if(ret != IS_SUCCESS)
	{
		NODELET_FATAL("Could not set pixel clock");
		return;
	}

	ret = is_SetColorMode(m_cam, IS_CM_BAYER_RG8);
	if(ret != IS_SUCCESS)
	{
		NODELET_FATAL("Could not set color mode to Bayer RGB");
		return;
	}

	for(int i = 0; i < NUM_BUFFERS; ++i)
	{
		ret = is_AllocImageMem(m_cam, WIDTH, HEIGHT, 8, &m_buf[i], &m_bufID[i]);
		if(ret != IS_SUCCESS)
		{
			NODELET_FATAL("Could not allocate image buffer");
			return;
		}

		ret = is_AddToSequence(m_cam, m_buf[i], m_bufID[i]);
		if(ret != IS_SUCCESS)
		{
			NODELET_FATAL("Could not add buffer to sequence");
			return;
		}

		is_UnlockSeqBuf(m_cam, m_bufID[i], m_buf[i]);
	}

	ret = is_InitImageQueue(m_cam, 0);
	if(ret != IS_SUCCESS)
	{
		NODELET_FATAL("Could not init image queue");
		return;
	}

	double frameRate;
	ret = is_SetFrameRate(m_cam, 18.0, &frameRate);
	if(ret != IS_SUCCESS)
	{
		NODELET_FATAL("Could not set frame rate");
		return;
	}
	NODELET_INFO("Got frame rate %lf", frameRate);

	ret = is_GetImageMemPitch(m_cam, &m_stepSize);
	if(ret != IS_SUCCESS)
	{
		NODELET_FATAL("Could not get line step size");
		return;
	}
	NODELET_INFO("Got line step size: %d", m_stepSize);

	ret = is_CaptureVideo(m_cam, IS_DONT_WAIT);
	if(ret != IS_SUCCESS)
	{
		NODELET_FATAL("Could not start live capture");
		return;
	}

	m_timer = getMTNodeHandle().createTimer(
		ros::Duration(1.0 / 35.0),
		boost::bind(&CameraUEye::update, this)
	);
}

const int PRECISION = 1024*1024;
inline uint FIXED_PREC(uint x)
{
	return x * PRECISION / 1000;
}

inline void rgb_to_uyvy(uint8_t r, uint8_t g, uint8_t b, uint8_t* dest)
{
	uint8_t y = (FIXED_PREC(299)*r + FIXED_PREC(587)*g + FIXED_PREC(114)*b) / PRECISION;
	dest[0] = (FIXED_PREC(493)*(b-y)) / PRECISION + 127;
	dest[2] = (FIXED_PREC(877)*(r-y)) / PRECISION + 127;
	dest[1] = dest[3] = y;
}

void CameraUEye::update()
{
	sensor_msgs::Image::Ptr img = boost::make_shared<sensor_msgs::Image>();

	char* buf;
	int bufID;
	int ret = is_WaitForNextImage(m_cam, 1000, &buf, &bufID);
	if(ret != IS_SUCCESS)
	{
		NODELET_FATAL("Could not get image (ret == %d)", ret);
		return;
	}

	img->encoding = "UYVY";
	img->width = WIDTH;
	img->height = HEIGHT;
	img->step = WIDTH*2;
	img->data.resize(WIDTH*HEIGHT*2);
	img->header.stamp = ros::Time::now();

	for(int y = 0; y < HEIGHT; y += 2)
	{
		for(int x = 0; x < WIDTH-1; x += 2)
		{
			uint8_t* base = (uint8_t*)buf + (y*m_stepSize + x);
			uint8_t r = base[0];
			uint8_t g1 = base[1];
			uint8_t g2 = base[m_stepSize];
			uint8_t b = base[m_stepSize+1];

			uint8_t* dbase = &img->data[0] + y*img->step + x*2;
			rgb_to_uyvy(r, g1, b, dbase);

			dbase += img->step;
			rgb_to_uyvy(r, g2, b, dbase);
		}
	}

	if(is_UnlockSeqBuf(m_cam, bufID, buf) != IS_SUCCESS)
	{
		NODELET_FATAL("Could not unlock buffer");
		return;
	}

	m_pub_image.publish(img);
}
