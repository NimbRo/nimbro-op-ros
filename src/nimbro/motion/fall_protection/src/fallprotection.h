// Provides fall protection
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef FALLPROTECTION_H
#define FALLPROTECTION_H

#include <robotcontrol/motionmodule.h>
#include <robotcontrol/model/robotmodel.h>

namespace fall_protection
{

class FallProtection : public robotcontrol::MotionModule
{
public:
	FallProtection();
	virtual ~FallProtection();

	virtual bool init(robotcontrol::RobotModel* model);
	virtual bool isTriggered();
	virtual void step();
private:
	ros::Time m_triggerTime;

	robotcontrol::RobotModel::State m_state_relaxed;
	robotcontrol::RobotModel::State m_state_init;
	robotcontrol::RobotModel::State m_state_falling;
	robotcontrol::RobotModel::State m_state_prone;
	robotcontrol::RobotModel::State m_state_supine;
	robotcontrol::RobotModel::State m_state_standingUp;
	robotcontrol::RobotModel::State m_state_sitting;
	robotcontrol::RobotModel::State m_state_kicking;

	bool m_enabled;
};

}

#endif
