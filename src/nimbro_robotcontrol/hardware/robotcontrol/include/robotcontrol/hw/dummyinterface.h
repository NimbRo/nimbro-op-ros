// Dummy hardware interface
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DUMMYINTERFACE_H
#define DUMMYINTERFACE_H

#include <robotcontrol/hw/hardwareinterface.h>

#include <boost/circular_buffer.hpp>

namespace robotcontrol
{

/**
 * @brief Dummy hardware interface
 *
 * This provides a simple loopback hardware interface. The joint commands
 * are simply stored and returned as position feedback.
 **/
class DummyInterface : public HardwareInterface
{
public:
	DummyInterface();
    virtual ~DummyInterface();

	virtual bool init(RobotModel* model);
	virtual boost::shared_ptr< Joint > createJoint(const std::string& name);
	virtual void getDiagnostics(robotcontrol::DiagnosticsPtr ptr);
	virtual bool readJointStates();
	virtual bool sendJointTargets();
	virtual bool setStiffness(float torque);
private:
	typedef boost::circular_buffer<std::vector<double> > DataBuf;
	DataBuf m_dataBuf;

	RobotModel* m_model;
};

}

#endif
