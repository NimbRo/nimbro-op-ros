// Handles tf data
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "timewarp/tfhandler.h"
#include "timewarp/timewarpnode.h"

#include <tf/tfMessage.h>

#include <boost/algorithm/string/predicate.hpp>

#include <ros/names.h>

namespace timewarp
{

TFHandler::TFHandler(timewarp::TimeWarpNode* node)
 : m_node(node)
 , m_tf(true, ros::Duration(60.0))
{
	m_sub_tf = node->nodeHandle()->subscribe("/tf", 100, &TFHandler::handleMsg, this);
	m_pub_tf = node->nodeHandle()->advertise<tf::tfMessage>("/vis/tf", 100);
}

TFHandler::~TFHandler()
{
}

void TFHandler::handleMsg(const tf::tfMessage& msg)
{
	if(!m_node->live())
		return;

	for(size_t i = 0; i < msg.transforms.size(); ++i)
	{
		tf::StampedTransform trans;
		tf::transformStampedMsgToTF(msg.transforms[i], trans);

		try {
			m_tf.setTransform(trans);
		}
		catch(tf::TransformException&)
		{
			continue;
		}
	}

	m_pub_tf.publish(msg);
}

void TFHandler::setTime(const ros::Time& now)
{
	std::vector<std::string> frames;
	m_tf.getFrameStrings(frames);

	m_msg.transforms.clear();

	for(size_t i = 0; i < frames.size(); ++i)
	{
		const std::string& frame = frames[i];
		std::string parentFrame;

		if(!m_tf.getParent(frame, now, parentFrame))
			continue;

		tf::StampedTransform transform;
		try
		{
			m_tf.lookupTransform(
				parentFrame,
				frame,
				now,
				transform
			);
		}
		catch(tf::TransformException&)
		{
			continue;
		}

		geometry_msgs::TransformStamped m;
		tf::transformStampedTFToMsg(transform, m);

		m_msg.transforms.push_back(m);
	}
}

void TFHandler::publish(const ros::Time& now)
{
	m_pub_tf.publish(m_msg);
}

}
