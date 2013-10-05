// Display for torques (based on JointState::effort)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <rviz_dynamics/torquedisplay.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/float_property.h>

#include <OGRE/OgreSceneManager.h>

#include <pluginlib/class_list_macros.h>

using namespace Ogre;

TorqueDisplay::TorqueDisplay()
{
	ros::NodeHandle nh;
	m_sub_js = nh.subscribe("/joint_states", 1, &TorqueDisplay::handleJointStates, this);

	m_model = boost::make_shared<urdf::Model>();
	if(!m_model->initParam("robot_description"))
	{
		ROS_ERROR("Could not get URDF model");
	}

	m_scale_prop = new rviz::FloatProperty("scale", 0.2, "Torques and forces will be scaled with this factor", this);
}

TorqueDisplay::~TorqueDisplay()
{
	onDisable();
}

void TorqueDisplay::onInitialize()
{
	m_rootNode = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode("torque_root");
}

void TorqueDisplay::onDisable()
{
	rviz::Display::onDisable();
	for(std::map<std::string, rviz::Arrow*>::iterator it = m_arrows.begin(); it != m_arrows.end(); ++it)
	{
		delete it->second;
	}

	m_arrows.clear();
}


void TorqueDisplay::handleJointStates(const sensor_msgs::JointStatePtr& js)
{
	m_js = js;
}

void TorqueDisplay::update(float wall_dt, float ros_dt)
{
	if(!context_ || !isEnabled())
		return;

	rviz::FrameManager* fmgr = context_->getFrameManager();

	if(!fmgr || !m_js)
		return;

	for(size_t i = 0; i < m_js->effort.size(); ++i)
	{
		boost::shared_ptr<const urdf::Joint> joint = m_model->getJoint(m_js->name[i]);
		if(!joint)
			continue;

		std::map<std::string, rviz::Arrow*>::iterator it = m_arrows.find(m_js->name[i]);

		geometry_msgs::Pose pose;
		pose.position.x = pose.position.y = pose.position.z = 0;
		pose.orientation.w = 1;
		pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;

		Ogre::Vector3 pos;
		Ogre::Quaternion rot;
		if(!fmgr->transform(joint->child_link_name, ros::Time(0), pose, pos, rot))
			continue;

		rviz::Arrow* arrow;

		if(it == m_arrows.end())
		{
			Ogre::SceneNode* sn = m_rootNode->createChildSceneNode();

			arrow = new rviz::Arrow(scene_manager_, sn);

			m_arrows[m_js->name[i]] = arrow;
		}
		else
			arrow = it->second;

		arrow->getSceneNode()->getParentSceneNode()->setPosition(pos);
		arrow->getSceneNode()->getParentSceneNode()->setOrientation(rot);

		double effort = m_js->effort[i] * m_scale_prop->getFloat();
		Vector3 jointAxis(joint->axis.x, joint->axis.y, joint->axis.z);
		if(effort > 0)
		{
			arrow->setColor(0.0, 1.0, 0.0, 1.0);
			arrow->setDirection(jointAxis);
		}
		else
		{
			arrow->setColor(1.0, 0.0, 0.0, 1.0);
			arrow->setDirection(-jointAxis);
		}

		arrow->set(fabs(effort), 0.01, 0.03, 0.02);
	}
}

PLUGINLIB_EXPORT_CLASS(TorqueDisplay, rviz::Display);
