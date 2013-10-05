// Robot Interface implements Hardware Interface
// Author: Sebastian Schüller

#ifndef RC_ROBOTINTERFACE_H
#define RC_ROBOTINTERFACE_H

#include <robotcontrol/hw/hardwareinterface.h>
#include <robotcontrol/hw/angleestimator.h>
#include <robotcontrol/hw/compassfilter.h>
#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/FadeTorqueAction.h>
#include <robotcontrol/ServoDiag.h>
#include <robotcontrol/LEDCommand.h>
#include <robotcontrol/ReadOffset.h>

#include "CM730.h"

#include <config_server/parameter.h>

#include <ros/timer.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>

#include <actionlib/client/simple_action_client.h>

namespace urdf { class Joint; }

namespace robotcontrol
{
class RobotModel;
class DynamicCommandGenerator;
typedef boost::shared_ptr<DynamicCommandGenerator> CommandGeneratorPtr;

/**
 * @brief Real robot interface
 *
 * Connects to a NimbRo-OP robot directly connected to the machine.
 *
 * Parameters on the config_server (under the robotcontrol group):
 *
 * Name                | Meaning
 * ------------------- | ---------------------------------
 * useModel            | Enable the servo model for command generation
 * effortSlope         | Maximum delta of the joint effort in 8ms
 * joints/X/type       | Actuator type of joint X
 * joints/X/id         | Address of joint X on the Dynamixel bus
 * joints/X/invert     | Invert the joint direction
 * offsets/X           | Offset (in servo ticks) of joint X
 *
 * The RobotInterface also advertises a single service called *read_offsets*
 * which resets the joint offsets so that the current measured robot pose
 * becomes the zero pose. This is useful for initial calibration (with a
 * relaxed robot) before fine-tuning the offsets.
 **/
class RobotInterface : public HardwareInterface
{
public:
	RobotInterface();
	virtual ~RobotInterface();

	virtual bool init(RobotModel* model);

	virtual boost::shared_ptr< Joint > createJoint(const std::string& name);

	virtual bool sendJointTargets();
	virtual bool readJointStates();
	virtual bool setStiffness(float torque);

	virtual void getDiagnostics(robotcontrol::DiagnosticsPtr ptr);

private:
	/**
	 * @brief Joint class with hardware information
	 **/
	struct DXLJoint : public Joint
	{
		DXLJoint(const std::string& name);

		//! Servo command generator appropiate for the servo type
		CommandGeneratorPtr commandGenerator;

		//! @name config_server joint parameters
		//@{
		config_server::Parameter<std::string> type;  //!< Actuator type
		config_server::Parameter<int> id;            //!< Actuator address
		config_server::Parameter<int> tickOffset;    //!< Joint offset in ticks
		config_server::Parameter<bool> invert;       //!< Invert the joint direction
		config_server::Parameter<bool> readFeedback; //!< Read enabled?
		//@}

		//! @name Statistics
		//@{
		int voltage;
		int temperature;
		//@}

		//! @name Control values smoothing
		//@{
		double realEffort;
		double rawState;
		//@}

		//! Statistics message
		robotcontrol::ServoDiag diag;
	};

	//! Process changed joint settings
	void updateJointSettings(DXLJoint* joint);

	//! Get an appropiate command generator for a servo type
	CommandGeneratorPtr commandGenerator(const std::string& type);

	//! Do a periodic statistics sweep
	void handleStatistics(const ros::TimerEvent&);

	void handleLEDCommand(const LEDCommand& cmd);

	//! Get the DXLJoint for an index
	inline DXLJoint* dxlJoint(size_t idx)
	{ return (DXLJoint*)m_model->joint(idx).get(); }

	void sendCM730Data();

	DXLJoint* dxlJointForID(int id);

	//! Our hardware driver
	boost::shared_ptr<CM730> m_board;

	//! The robot model
	RobotModel* m_model;

	//! All servo types and generators we know about
	std::map<std::string, CommandGeneratorPtr> m_generators;

	//! All joints relaxed?
	bool m_relaxed;


	//! Timer used for statistics callback
	ros::Timer m_statisticsTimer;

	//! Servo to ask for statistics next
	size_t m_statIndex;

	//! Mean servo voltage
	double m_statVoltage;

	//! BulkRead buffer for servo feedback
	std::vector<BRData> m_servoData;

	//! BulkRead buffer for CM730 feedback (e.g. IMU)
	BRBoard m_boardData;

	//! Service to read offsets from hardware
	ros::ServiceServer m_srv_readOffsets;
	ros::ServiceServer m_srv_readOffset;
	bool handleReadOffsets(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
	bool handleReadOffset(ReadOffsetRequest& req, ReadOffsetResponse& resp);

	AngleEstimator m_angleEstimator;
	CompassFilter m_compassFilter;

	//! Use servo model for command generation or just send position commands?
	config_server::Parameter<bool> m_useModel;

	//! Maximum delta in one period for effort setting
	config_server::Parameter<float> m_effortSlope;

	//! Maximum delta in one period for raw setting
	config_server::Parameter<float> m_rawSlope;

	//! Publish estimated angle plots
	ros::Publisher m_pub_angle_plot;

	//! Publish button events
	ros::Publisher m_pub_buttons;

	//! Last button mask
	uint8_t m_lastButtons;

	//! Action client for the torque fading
	actionlib::SimpleActionClient<FadeTorqueAction> m_act_fadeTorque;

	std::vector<int> m_cm730_queryset;

	bool m_skipStep;

	LEDCommand m_ledCommand;
	ros::Subscriber m_sub_led;
};

}

#endif
