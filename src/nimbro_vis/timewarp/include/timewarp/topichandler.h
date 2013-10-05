// Handles one ROS topic
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TOPICHANDLER_H
#define TOPICHANDLER_H

#include <ros/node_handle.h>
#include <string>

#include <boost/circular_buffer.hpp>

#include <topic_tools/shape_shifter.h>
#include "timeextractor.h"

namespace timewarp
{

class TopicHandler
{
public:
	TopicHandler(ros::NodeHandle* nh, const std::string& name);
	virtual ~TopicHandler();

	void setLive(bool live);
	void setTime(const ros::Time& time);
	void publish(const ros::Time& now);

	inline std::string name() const
	{ return m_sub.getTopic(); }
protected:
	struct Entry
	{
		ros::Time captureTime;
		boost::shared_ptr<topic_tools::ShapeShifter> data;
	};
	friend class EntryComp;

	typedef boost::circular_buffer<Entry> CircBuf;
	CircBuf m_buf;

	bool m_live;

	virtual CircBuf::iterator findPacketForTime(const ros::Time& time);
private:
	void handleData(const boost::shared_ptr<topic_tools::ShapeShifter>& data);

	ros::NodeHandle* m_nh;

	bool m_pubInit;
	ros::Subscriber m_sub;
	ros::Publisher m_pub;

	boost::shared_ptr<topic_tools::ShapeShifter> m_msg;
};

template<class MsgType>
class SmartTopicHandler : public TopicHandler
{
public:
	SmartTopicHandler(ros::NodeHandle* nh, const std::string& name)
	 : TopicHandler(nh, name)
	{}

	virtual ~SmartTopicHandler()
	{}
protected:
	virtual CircBuf::iterator findPacketForTime(const ros::Time& time);
};

template<class MsgType>
boost::circular_buffer< TopicHandler::Entry >::iterator SmartTopicHandler<MsgType>::findPacketForTime(const ros::Time& time)
{
	CircBuf::iterator it, first, last;

	first = m_buf.begin();
	last = m_buf.end();

	std::iterator_traits<CircBuf::iterator>::difference_type count, step;
	count = std::distance(first, last);

	while(count > 0)
	{
		it = first;
		step = count/2;
		std::advance(it, step);

		ros::Time msgTime = extractTime<MsgType>((*it).data);

		if(msgTime < time)
		{
			first = ++it;
			count -= step+1;
		}
		else
			count = step;
	}

	return first;
}


}

#endif
