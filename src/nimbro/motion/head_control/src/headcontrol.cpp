// Motion of the robot head
// Author: Philipp Allgeuer

// Includes
#include "head_control/headcontrol.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PointStamped.h>

// Defines
#define TINC	0.008 // Step time

#define DIRECT_VISION 0

// Namespaces
using namespace headcontrol;
using namespace robotcontrol;

//
// HeadControl class
//

// Constructor
HeadControl::HeadControl()
 : m_head_enabled("/headcontrol/head_enabled",true) // TODO: Change back to false
 , m_settling_time("/headcontrol/settling_time", 0.2, 0.05, 3.0, 0.50)
 , m_max_head_vel("/headcontrol/max_head_vel", 0.5, 0.05, 10.0, 4.0) // rad/s
 , m_search_vel("/headcontrol/search_vel", 0.01, 0.01, 10.0, M_PI/2.0)
{
	// Create node handle
	ros::NodeHandle nh("~");

	// Advertise and subscribe to ROS node(s)
#if DIRECT_VISION
	m_ball_focal_vec = nh.subscribe("/ball/focal_plane_ball_vector", 1, &HeadControl::handleFocalData, this);
#endif

	m_sub_target = nh.subscribe("headcontrol/target", 1, &HeadControl::handleLookAtTarget, this);
	
	// Initialise variables
	m_focal_vec << 0.0, 0.0, 0.3;
	m_target_yaw = 0.0;
	m_target_pitch = M_PI / 4.0;
}

// Destructor
HeadControl::~HeadControl()
{
}

// Initialisation function
bool HeadControl::init(RobotModel* model)
{
	// Save pointer to RobotModel
	m_model=model;
	
	// Initialise the motion module
	MotionModule::init(m_model);
  
	// Get references to the head joints
	m_head_yaw = m_model->getJoint("neck_yaw");	
	m_head_pitch = m_model->getJoint("head_pitch");

	// TODO: If you want to do anything more complicated in head control then you might need these transforms!
// 	// Get transform from head frame to camera frame (constant in time)
// 	boost::shared_ptr<const urdf::Link> cam_link = m_model->urdf()->getLink("camera_optical");
// 	urdf::Pose T = cam_link->parent_joint->parent_to_joint_origin_transform;
// 	
// 	// Extract the translation vector and rotation matrix from the transform
// 	Eigen::Quaterniond RotQuat(T.rotation.w,T.rotation.x,T.rotation.y,T.rotation.z);
// 	m_rot = RotQuat.matrix();
// 	m_trans << T.position.x, T.position.y, T.position.z;
// 	
// 	// TODO: Remove this - Display the translation and rotation used
// 	ROS_INFO_STREAM("Trans" << Trans.transpose());
// 	ROS_INFO_STREAM("Rot" << Rot);

	m_state_kicking = model->registerState("kicking");
	m_state_standUp = model->registerState("standing_up");
	
	// Return successful initialisation
	return true;
}

bool HeadControl::isTriggered()
{
	return model()->state() != m_state_kicking
	  && model()->state() != m_state_standUp;
}

// Step function
void HeadControl::step()
{
	// Only move the head if it is enabled
	if(m_head_enabled())
	{
		// TODO: Print current read data to console to verify it's arriving properly
//		ROS_INFO_THROTTLE(1,"HeadControl says: (%lf,%lf,%lf)...",m_focal_vec.x(),m_focal_vec.y(),m_focal_vec.z());
		
		// Calculate an alpha that gives you (approx) the desired settling time
		double alpha = 1/(1+m_settling_time()/(16.0*TINC));
		
		// Calculate appropriate head yaw and pitch position setpoints (first order low pass filter ball tracking)
		double yaw_cur = m_head_yaw->feedback.pos;
		double pitch_cur = m_head_pitch->feedback.pos;
		double yaw_cmd = yaw_cur + alpha * (m_target_yaw - yaw_cur);
		double pitch_cmd = pitch_cur + alpha * (m_target_pitch - pitch_cur);

#if DIRECT_VISION
		if((ros::Time::now() - m_lastBallSeenTime) > ros::Duration(2.0))
		{
			const double searchBoundary = M_PI/2.0;

			if(m_searchLeft)
			{
				yaw_cmd = m_head_yaw->cmd.pos + TINC * m_search_vel();
				if(yaw_cmd > searchBoundary)
					m_searchLeft = false;
			}
			else
			{
				yaw_cmd = m_head_yaw->cmd.pos - TINC * m_search_vel();
				if(yaw_cmd < -searchBoundary)
					m_searchLeft = true;
			}
		}
#endif

		// Apply head velocity limiting
		if(fabs(yaw_cmd-yaw_cur) > TINC * m_max_head_vel())
		{
			if(yaw_cmd<yaw_cur)
				yaw_cmd = yaw_cur - TINC * m_max_head_vel();
			else
				yaw_cmd = yaw_cur + TINC * m_max_head_vel();
		}
		if(fabs(pitch_cmd-pitch_cur) > TINC * m_max_head_vel())
		{
			if(pitch_cmd<pitch_cur)
				pitch_cmd = pitch_cur - TINC * m_max_head_vel();
			else
				pitch_cmd = pitch_cur + TINC * m_max_head_vel();
		}

		if(yaw_cmd > M_PI/2.0)
			yaw_cmd = M_PI/2.0;
		else if(yaw_cmd < -M_PI/2.0)
			yaw_cmd = -M_PI/2.0;

		if(pitch_cmd < 0.0)
			pitch_cmd = 0.0;

		// Command the required new head joint positions
		ros::Time notime; // Dummy parameter for setFromPos
		m_head_yaw->cmd.setFromPos(notime,yaw_cmd);
		m_head_pitch->cmd.setFromPos(notime,pitch_cmd);
		m_head_yaw->cmd.raw = true;
		m_head_pitch->cmd.raw = true;
		m_head_yaw->cmd.effort = 1.0;
		m_head_pitch->cmd.effort = 1.0;

		// TODO: Remove - print the current control output
//		ROS_INFO_THROTTLE(1.0, "CUR - Yaw: %lf / Pitch: %lf",yaw_cur,pitch_cur);
//		ROS_INFO_THROTTLE(1.0, "CMD - Yaw: %lf / Pitch: %lf",yaw_cmd,pitch_cmd);
//		ROS_INFO_THROTTLE(1, "PITCH CURRENT -> DELTA = %lf -> %lf", yaw_cur, yaw_cmd-yaw_cur);
	}
}

// Callback for focal data topic
void HeadControl::handleFocalData(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	// Store the received data locally
	m_focal_vec << msg->point.x, msg->point.y, msg->point.z;

	// Calculate approximate target head yaw and pitch positions
	m_target_yaw   = m_head_yaw->feedback.pos + atan2(-m_focal_vec.x(),m_focal_vec.z());
	m_target_pitch = m_head_pitch->feedback.pos + atan2(m_focal_vec.y(),m_focal_vec.z());

	m_lastBallSeenTime = msg->header.stamp;
}

void HeadControl::handleLookAtTarget(const head_control::LookAtTargetConstPtr& msg)
{
	if(msg->is_angular_data)
	{
		m_target_yaw = msg->vec.z;
		m_target_pitch = msg->vec.y;
	}
	else
	{
		m_target_yaw = atan2(-msg->vec.x, msg->vec.z);
		m_target_pitch = atan2(msg->vec.y, msg->vec.z);
	}

	if(msg->is_relative)
	{
		m_target_yaw += m_head_yaw->feedback.pos;
		m_target_pitch += m_head_pitch->feedback.pos;
	}
}

PLUGINLIB_EXPORT_CLASS(headcontrol::HeadControl, robotcontrol::MotionModule)
// EOF