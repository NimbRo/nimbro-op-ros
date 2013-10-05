// Handles one ROS topic
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "timewarp/topichandler.h"

namespace timewarp
{

TopicHandler::TopicHandler(ros::NodeHandle* nh, const std::string& name)
 : m_buf(4096)
 , m_live(true)
 , m_nh(nh)
 , m_pubInit(false)
{
	m_sub = nh->subscribe(name, 3, &TopicHandler::handleData, this);
}

TopicHandler::~TopicHandler()
{
}

void TopicHandler::setLive(bool live)
{
	m_live = live;
}

void TopicHandler::handleData(const boost::shared_ptr<topic_tools::ShapeShifter>& data)
{
	if(!m_live)
		return;

	if(!m_pubInit)
	{
		m_pub = data->advertise(*m_nh, "/vis/" + m_sub.getTopic(), 1);
		m_pubInit = true;
	}

	Entry entry;
	entry.captureTime = ros::Time::now();
	entry.data = data;

	m_buf.push_back(entry);
	m_pub.publish(data);
}

struct EntryComp
{
	bool operator()(const TopicHandler::Entry& a, const TopicHandler::Entry& b)
	{
		return a.captureTime < b.captureTime;
	}
};

boost::circular_buffer< TopicHandler::Entry >::iterator TopicHandler::findPacketForTime(const ros::Time& time)
{
	Entry cmp;
	cmp.captureTime = time;

	return std::lower_bound(m_buf.begin(), m_buf.end(), cmp, EntryComp());
}

void TopicHandler::setTime(const ros::Time& time)
{
	CircBuf::iterator it = findPacketForTime(time);

	if(it == m_buf.end())
		return;

	m_msg = (*it).data;
}

void TopicHandler::publish(const ros::Time& now)
{
	if(!m_pubInit)
		return;

	if(m_msg)
		m_pub.publish(m_msg);
}



}
