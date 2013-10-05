// Joint information
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <robotcontrol/model/joint.h>

#include <ros/console.h>

namespace robotcontrol
{

double Joint::Command::velLimit = 0.0;
double Joint::Command::accLimit = 0.0;

Joint::Command::Command()
 : pos(0)
 , vel(0)
 , acc(0)
 , effort(0.0)
 , raw(false)
{
}

void Joint::Command::resetDeriv()
{
	m_dev_acc.reset();
	m_dev_vel.reset();
}


/**
 * This method calculates the vel and acc fields using
 * Savitzky-Golay smoothed differentiation.
 *
 * The velocity and acceleration limits are enforced here (see velLimit,
 * accLimit).
 *
 * @warning This method currently assumes that it is called every 8ms.
 *
 * @param newTime current System time (currently unused)
 * @param newPos Goal position (rad)
 **/
void Joint::Command::setFromPos(const ros::Time& newTime, double newPos)
{
	// FIXME: Measuring the time gives very bad results.
// 	double deltaT = (newTime - stamp).toSec();
	double deltaT = 0.008;

	if(deltaT == 0)
	{
		pos = newPos;
		return;
	}

	m_dev_vel.put(newPos);

	double newVel = m_dev_vel.value() / 0.008;

// 	if(newVel > velLimit)
// 	{
// 		newVel = velLimit;
// 		newPos = pos + newVel * deltaT;
// 	}
// 	else if(newVel < -velLimit)
// 	{
// 		newVel = -velLimit;
// 		newPos = pos + newVel * deltaT;
// 	}

	m_dev_acc.put(newPos);
	double newAcc = m_dev_acc.value() / 0.008 / 0.008;

// 	if(newAcc > accLimit)
// 		newAcc = accLimit;
// 	else if(newAcc < -accLimit)
// 		newAcc = -accLimit;

	pos = newPos;
	vel = newVel;
	acc = newAcc;
}

void Joint::Command::setFromPos_unsmoothed(double newPos)
{
	double newVel = (newPos - pos) / 0.008;
	m_dev_vel.put(newPos);

	acc = (newVel - vel) / 0.008;
	m_dev_acc.put(newPos);

	pos = newPos;
	vel = newVel;
}


/**
 * If you have goal position and velocity available, use this method instead
 * of setFromPos().
 **/
void Joint::Command::setFromPosVel(const ros::Time& newTime, double newPos, double newVel)
{
// 	double deltaT = (newTime - stamp).toSec();
	double deltaT = 0.008;

	if(deltaT == 0)
	{
		pos = newPos;
		vel = newVel;
		return;
	}

	m_dev_acc.put(newVel);

	acc = m_dev_acc.value();
	vel = newVel;
	pos = newPos;
}

void Joint::Command::set(double npos, double nvel, double nacc)
{
	pos = npos;
	vel = nvel;
	acc = nacc;

	m_dev_vel.put(npos);
	m_dev_acc.put(npos);
}

Joint::Feedback::Feedback()
 : pos(0)
 , modelTorque(0)
{
}


}
