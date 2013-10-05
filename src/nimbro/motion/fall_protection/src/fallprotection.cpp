// Provides fall protection
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "fallprotection.h"
#include <robotcontrol/model/robotmodel.h>

#include <tf/transform_datatypes.h>

#include <pluginlib/class_list_macros.h>

namespace fall_protection
{

FallProtection::FallProtection()
 : m_enabled(true)
{
}

FallProtection::~FallProtection()
{
}

bool FallProtection::init(robotcontrol::RobotModel* model)
{
	robotcontrol::MotionModule::init(model);

	for(size_t i = 0; i < model->numJoints(); ++i)
		setJointCommand(i, 0, 0);

	m_state_relaxed = model->registerState("relaxed");
	m_state_init = model->registerState("init");
	m_state_falling = model->registerState("falling");
	m_state_prone = model->registerState("prone");
	m_state_supine = model->registerState("supine");
	m_state_standingUp = model->registerState("standing_up");
	m_state_sitting = model->registerState("sitting");
	m_state_kicking = model->registerState("kicking");

	return true;
}

bool FallProtection::isTriggered()
{
	double angle = model()->robotOrientation().getAngle();
	double angLimit = 35.0 * M_PI / 180.0;
	robotcontrol::RobotModel::State state = model()->state();

	if(state == m_state_kicking)
		angLimit = 45.0 * M_PI / 180.0;

	bool trigger = fabs(angle) > angLimit;

	if(trigger
		&& state != m_state_relaxed
		&& state != m_state_init
		&& state != m_state_falling
		&& state != m_state_prone
		&& state != m_state_supine
		&& state != m_state_standingUp
		&& state != m_state_sitting
	)
	{
		ROS_WARN("Fall protection triggered! angle was %8.3lf degrees", angle * 180.0 / M_PI);
		m_triggerTime = ros::Time::now();
		model()->setState(m_state_falling);
		return true;
	}

	return state == m_state_falling;
}

void FallProtection::step()
{
	ros::Duration duration = ros::Time::now() - m_triggerTime;

	model()->setRelaxed(true);

	if(duration > ros::Duration(2.0))
	{
		tf::Quaternion orientation = model()->robotOrientation();
		tfScalar roll, pitch, yaw;
		tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

		if(pitch > 0)
			model()->setState(m_state_prone);
		else
			model()->setState(m_state_supine);

		model()->setRelaxed(false);
	}
}

}

PLUGINLIB_EXPORT_CLASS(fall_protection::FallProtection, robotcontrol::MotionModule)
