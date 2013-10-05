// Handles tf data
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TFHANDLER_H
#define TFHANDLER_H

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace timewarp
{
class TimeWarpNode;

class TFHandler
{
public:
	explicit TFHandler(TimeWarpNode* node);
	virtual ~TFHandler();

	void setTime(const ros::Time& now);
	void publish(const ros::Time& now);
private:
	void handleMsg(const tf::tfMessage& msg);

	TimeWarpNode* m_node;
	tf::Transformer m_tf;
	ros::Subscriber m_sub_tf;
	ros::Publisher m_pub_tf;

	tf::tfMessage m_msg;
};

}

#endif
