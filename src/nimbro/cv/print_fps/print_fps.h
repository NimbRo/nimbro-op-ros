// FPS printer
// Author: Max Schwarz <Max@x-quadraht.de>

#ifndef CAMERA_V4L2_H
#define CAMERA_V4L2_H

#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>


class PrintFPS : public nodelet::Nodelet
{
public:
	PrintFPS();
	virtual ~PrintFPS();

	virtual void onInit();

	void processImage(const sensor_msgs::Image::Ptr& img);
private:
	ros::Subscriber m_sub_input;

	int m_counter;
	ros::Time m_lastTime;
};

#endif
