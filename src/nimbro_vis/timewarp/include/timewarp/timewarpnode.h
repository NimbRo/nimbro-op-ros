// Provides a time-warped view on selected topics
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TIMEWARPNODE_H
#define TIMEWARPNODE_H

#include <ros/node_handle.h>

#include "timewarp/tfhandler.h"
#include "topicthread.h"

#include "timewarp/TimeWarpControl.h"

namespace timewarp
{

class TopicHandler;

class TimeWarpNode
{
public:
	TimeWarpNode();
	virtual ~TimeWarpNode();

	inline ros::NodeHandle* nodeHandle()
	{ return &m_nh; }

	inline bool live() const
	{ return m_live; }

	inline ros::Time time() const
	{ return m_time; }
private:
	bool handleTimeWarpControl(TimeWarpControlRequest& req, TimeWarpControlResponse& resp);
	void update();
	void updateTopics(const ros::master::V_TopicInfo& list);

	int findHandler(const std::string& name);

	ros::NodeHandle m_nh;

	bool m_live;
	ros::Time m_time;

	TFHandler m_tfHandler;

	ros::ServiceServer m_srv;
	ros::Timer m_timer;

	ros::Publisher m_pub_clock;

	std::vector<TopicHandler*> m_handlers;

	boost::thread m_topicThread;
};

}

#endif
