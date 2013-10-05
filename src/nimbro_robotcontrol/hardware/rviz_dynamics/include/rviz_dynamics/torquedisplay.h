// Display for torques (based on JointState::effort)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TORQUEDISPLAY_H
#define TORQUEDISPLAY_H

#include <rviz/display.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

namespace rviz {
class Arrow;
class FloatProperty;
}

namespace Ogre {
class SceneNode;
}

class TorqueDisplay : public rviz::Display
{
public:
	TorqueDisplay();
	virtual ~TorqueDisplay();

	virtual void onInitialize();
	virtual void update(float wall_dt, float ros_dt);
	virtual void onDisable();

	void handleJointStates(const sensor_msgs::JointStatePtr& js);
private:
	Ogre::SceneNode* m_rootNode;
	ros::Subscriber m_sub_js;
	sensor_msgs::JointStatePtr m_js;
	boost::shared_ptr<urdf::Model> m_model;
	std::map<std::string, rviz::Arrow*> m_arrows;

	rviz::FloatProperty* m_scale_prop;
};

#endif
