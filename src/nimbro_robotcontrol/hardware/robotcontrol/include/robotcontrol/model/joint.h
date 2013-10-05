// Joint information
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef JOINT_H
#define JOINT_H

#include <ros/time.h>
#include <urdf/model.h>
#include <string>

#include "golay.h"

namespace robotcontrol
{

struct JointCommand
{
	JointCommand()
	 : pos(0)
	 , effort(0)
	{}

	double pos;
	double effort;
};

/**
 * @brief Single joint representation
 *
 * Contains command values and feedback information for a single joint.
 **/
struct Joint
{
	//! Joint-level command
	class Command
	{
	public:
		Command();

		double pos;  //!< Goal position
		double vel;  //!< Velocity
		double acc;  //!< Acceleration

		double effort; //!< Joint effort (stiffness), range 0..1
		bool raw;      //!< Disable the motor model

		double rawCmd; //!< Sent servo command (rad)

		void resetDeriv();

		//! Position command
		void setFromPos(const ros::Time& newTime, double newPos);

		//! Position & velocity command
		void setFromPosVel(const ros::Time& newTime, double newPos, double newVel);

		void setFromPos_unsmoothed(double newPos);

		void set(double pos, double vel, double acc);

		/**
		 * @brief Global velocity limit
		 *
		 * The commanded velocity is limited by this global velocity limit. If a
		 * limitation occurs, the goal position is also limited to the position
		 * reachable with the limited velocity.
		 **/
		static double velLimit;

		/**
		 * @brief Global acceleration limit
		 *
		 * This parameter simply limits the calculated acceleration. It does
		 * not, however, constrain the position or velocity commands.
		 **/
		static double accLimit;
	private:
		GolayDerivative<double, 1, 5> m_dev_vel;
		GolayDerivative<double, 2, 5> m_dev_acc;
	};

	//! Joint-level feedback
	struct Feedback
	{
		Feedback();

		ros::Time stamp;     //!< Feedback timestamp
		double pos;          //!< Current servo position (rad)
		double modelTorque;  //!< Calculated servo torque (Nm)
		double torque;       //!< Estimated torque from position displacement (Nm)
	};

	//! Smart pointer type (use this!)
	typedef boost::shared_ptr<Joint> Ptr;

	//! URDF joint name
	std::string name;

	//! URDF joint
	boost::shared_ptr<urdf::Joint> modelJoint;

	//! Joint command (written by the MotionModule)
	Command cmd;

	//! Feedback (written by the HardwareInterface)
	Feedback feedback;
};

}

#endif
