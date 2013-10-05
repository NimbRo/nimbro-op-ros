// Motion of the robot head
// Author: Philipp Allgeuer

// Ensure header is only included once
#ifndef HEADCONTROLNODE_H
#define HEADCONTROLNODE_H

// Includes
#include <robotcontrol/motionmodule.h>
#include <robotcontrol/model/robotmodel.h>
#include <config_server/parameter.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <head_control/LookAtTarget.h>
#include <math.h>

// headcontrol namespace
namespace headcontrol
{

// HeadControl class
class HeadControl : public robotcontrol::MotionModule
{
	//
	// Public functions
	//
public:
	HeadControl();
    virtual ~HeadControl();
	virtual bool init(robotcontrol::RobotModel* model);
	virtual void step();
	virtual bool isTriggered();

	//
	// Private functions
	//
private:
	void handleFocalData(const geometry_msgs::PointStamped::ConstPtr& msg);

	void handleLookAtTarget(const head_control::LookAtTargetConstPtr& msg);

	//
	// Member variables
	//
private:
	robotcontrol::RobotModel* m_model;

	// Config server parameters
	config_server::Parameter<bool> m_head_enabled;
	config_server::Parameter<float> m_settling_time;
	config_server::Parameter<float> m_max_head_vel;
	config_server::Parameter<float> m_search_vel;

	// Publishers and subscribers
	ros::Subscriber m_ball_focal_vec;
	ros::Subscriber m_sub_target;
	ros::Time m_lastBallSeenTime;
	bool m_searchLeft;
	
	// Joints
	robotcontrol::Joint::Ptr m_head_yaw;
	robotcontrol::Joint::Ptr m_head_pitch;
	double m_target_yaw;
	double m_target_pitch;
	
	// Internal variables
	Eigen::Vector3d m_focal_vec;

	robotcontrol::RobotModel::State m_state_kicking;
	robotcontrol::RobotModel::State m_state_standUp;

// 	Eigen::Vector3d m_trans; // TODO: Uncomment these if you need the transforms (see init function)
// 	Eigen::Vector3d m_rot;
};

// End headcontrol namespace
}

#endif
// EOF