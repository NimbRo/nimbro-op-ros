// Interface for motion plugins
// Author: Sebastian Schüller

#include "motionmodule.h"
#include <model/robotmodel.h>

namespace robotcontrol
{

MotionModule::~MotionModule()
{
}

void MotionModule::publishTransforms()
{
}

bool MotionModule::init(RobotModel* model)
{
	m_model = model;
	return true;
}

bool MotionModule::isTriggered()
{
	return true;
}

void MotionModule::setJointCommand(int index, double pos, double effort)
{
	Joint::Ptr joint = m_model->joint(index);
	joint->cmd.setFromPos(ros::Time(0), pos);
	joint->cmd.effort = effort;
}

}
