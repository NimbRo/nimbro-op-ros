// IDS uEye camera driver
// Author: Max Schwarz <Max@x-quadraht.de>

#ifndef CAMERA_UEYE_H
#define CAMERA_UEYE_H

#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>

#include <vector>
#include <stdint.h>

#include <ueye.h>

const int NUM_BUFFERS = 4;

class CameraUEye : public nodelet::Nodelet
{
public:
	CameraUEye();
	virtual ~CameraUEye();

	virtual void onInit();
	virtual void update();
private:
	ros::Publisher m_pub_image;
	ros::Timer m_timer;
	HIDS m_cam;
	char* m_buf[NUM_BUFFERS];
	int m_bufID[NUM_BUFFERS];
	int m_stepSize;
};

#endif
