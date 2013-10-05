// Provides a time-warped view on selected topics
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "timewarp/timewarpnode.h"
#include "timewarp/topichandler.h"
#include "timewarp/topicthread.h"

#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>

namespace timewarp
{

TimeWarpNode::TimeWarpNode()
 : m_nh("~")
 , m_live(true)
 , m_tfHandler(this)
{
	m_srv = m_nh.advertiseService("control", &TimeWarpNode::handleTimeWarpControl, this);

	m_pub_clock = m_nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

	m_timer = m_nh.createTimer(ros::Duration(0.1), boost::bind(&TimeWarpNode::update, this));
	m_timer.start();

	XmlRpc::XmlRpcValue list;
	if(m_nh.getParam("extra_topics", list))
	{
		ROS_INFO("Using extra_topics parameter");
		ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

		for(int i = 0; i < list.size(); ++i)
		{
			std::string name = static_cast<std::string>(list[i]);

			ROS_INFO("Warping extra topic '%s'", name.c_str());

			m_handlers.push_back(
				new TopicHandler(&m_nh, name)
			);
		}
	}

	m_handlers.push_back(
		new SmartTopicHandler<sensor_msgs::JointState>(&m_nh, "/joint_states")
	);

	m_topicThread = TopicThread::launch(m_nh, boost::bind(&TimeWarpNode::updateTopics, this, _1));
}

TimeWarpNode::~TimeWarpNode()
{
	for(size_t i = 0; i < m_handlers.size(); ++i)
		delete m_handlers[i];
}

bool TimeWarpNode::handleTimeWarpControl(TimeWarpControlRequest& req, TimeWarpControlResponse& resp)
{
	m_live = req.live;
	m_time = req.time;

	m_tfHandler.setTime(req.time);

	for(size_t i = 0; i < m_handlers.size(); ++i)
	{
		m_handlers[i]->setLive(req.live);
		m_handlers[i]->setTime(req.time);
	}

	return true;
}

void TimeWarpNode::update()
{
	rosgraph_msgs::Clock clock;
	ros::Time now = ros::Time::now();

	if(m_live)
		clock.clock = now;
	else
		clock.clock = m_time;

	m_pub_clock.publish(clock);

	if(!m_live)
	{
		m_tfHandler.publish(now);

		for(size_t i = 0; i < m_handlers.size(); ++i)
		{
			m_handlers[i]->publish(now);
		}
	}
}

int TimeWarpNode::findHandler(const std::string& name)
{
	for(size_t i = 0; i < m_handlers.size(); ++i)
	{
		TopicHandler* handler = m_handlers[i];

		if(handler->name() == name)
			return i;
	}

	return -1;
}


void TimeWarpNode::updateTopics(const ros::master::V_TopicInfo& topics)
{
	const char* vis_type = ros::message_traits::DataType<visualization_msgs::MarkerArray>::value();
	for(ros::master::V_TopicInfo::const_iterator it = topics.begin(); it != topics.end(); ++it)
	{
		const ros::master::TopicInfo& info = *it;

		if(info.datatype == vis_type)
		{
			if(info.name.substr(0, 5) == "/vis/")
				continue;

			int idx = findHandler(info.name);

			if(idx != -1)
				continue;

			ROS_WARN("Creating handler for topic '%s'", info.name.c_str());
			m_handlers.push_back(
				new SmartTopicHandler<visualization_msgs::MarkerArray>(&m_nh, info.name)
			);
		}
	}
}


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "timewarp");

	timewarp::TimeWarpNode n;

	ros::spin();
	return 0;
}
