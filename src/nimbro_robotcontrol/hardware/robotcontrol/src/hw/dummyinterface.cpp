// Dummy hardware interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "robotcontrol/hw/dummyinterface.h"
#include "robotcontrol/model/robotmodel.h"
#include <boost/make_shared.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

namespace robotcontrol
{

DummyInterface::DummyInterface()
 : m_dataBuf(5)
{
}

DummyInterface::~DummyInterface()
{
}

bool DummyInterface::init(RobotModel* model)
{
	m_model = model;
	return true;
}

void DummyInterface::getDiagnostics(robotcontrol::DiagnosticsPtr ptr)
{
	ptr->header.stamp = ros::Time::now();
	ptr->batteryVoltage = 14.0;
	ptr->servoTemperature = 30.0;
}

boost::shared_ptr< Joint > DummyInterface::createJoint(const std::string& name)
{
	Joint::Ptr joint = boost::make_shared<Joint>();
	joint->name = name;

	return joint;
}

bool DummyInterface::readJointStates()
{
	ros::Time now = ros::Time::now();

	std::vector<double> cmds = *m_dataBuf.begin();

	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		const boost::shared_ptr<Joint>& joint = m_model->joint(i);
		joint->feedback.pos = cmds[i] + (drand48() - 0.5) * 0.05 / 180.0 * M_PI;
		joint->feedback.stamp = now;
	}

	return true;
}

bool DummyInterface::sendJointTargets()
{
	std::vector<double> cmds(m_model->numJoints());
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		const boost::shared_ptr<Joint>& joint = m_model->joint(i);
		cmds[i] = joint->cmd.pos;
	}

	m_dataBuf.push_back(cmds);

	return true;
}

bool DummyInterface::setStiffness(float torque)
{
	return true;
}


}

PLUGINLIB_EXPORT_CLASS(robotcontrol::DummyInterface, robotcontrol::HardwareInterface)
