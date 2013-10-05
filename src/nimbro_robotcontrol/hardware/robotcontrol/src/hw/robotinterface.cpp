// Robot interface
// Authors: Sebastian Schüller, Max Schwarz <max.schwarz@uni-bonn.de>

#include <robotcontrol/hw/robotinterface.h>
#include <robotcontrol/hw/dynamiccommandgenerator.h>
#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/hw/slopelimited.h>
#include <robotcontrol/Button.h>

#include <ros/console.h>
#include <ros/node_handle.h>
#include <urdf/model.h>

#include <plot_msgs/Plot.h>

#include <boost/make_shared.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <servomodel/servocommandgenerator.h>

#include <pluginlib/class_list_macros.h>

#include <boost/foreach.hpp>

namespace robotcontrol
{

/**
 * DXLJoint constructor. Simply creates parameters on the config server
 * for the joint.
 **/
RobotInterface::DXLJoint::DXLJoint(const std::string& _name)
 : type("joints/" + _name + "/type", "default_type")
 , id("joints/" + _name + "/id", 1, 1, 253, 1)
 , tickOffset("offsets/" + _name, 0, 1, 4095, 2048)
 , invert("joints/" + _name + "/invert", false)
 , readFeedback("joints/" + _name + "/readFeedback", true)
 , realEffort(cmd.effort)
 , rawState(1.0)
{
	name = _name;

	diag.name = name;
}

RobotInterface::RobotInterface()
 : m_relaxed(true)
 , m_statIndex(0)
 , m_useModel("useModel", false)
 , m_effortSlope("effortSlope", 0.0, 0.01, 1.0, 0.02)
 , m_rawSlope("rawSlope", 0.0, 0.01, 1.0, 0.02)
 , m_lastButtons(0)
 , m_act_fadeTorque("/robotcontrol/fade_torque")
 , m_skipStep(false)
{
	ros::NodeHandle nh("~");

	m_statisticsTimer = nh.createTimer(ros::Duration(0.2), &RobotInterface::handleStatistics, this);
	m_statisticsTimer.stop();

	m_pub_angle_plot = nh.advertise<plot_msgs::Plot>("/plot", 1);
	m_pub_buttons = nh.advertise<robotcontrol::Button>("/button", 1);

	m_sub_led = nh.subscribe("/led", 2, &RobotInterface::handleLEDCommand, this);

	m_srv_readOffsets = nh.advertiseService("read_offsets", &RobotInterface::handleReadOffsets, this);
	m_srv_readOffset = nh.advertiseService("read_offset", &RobotInterface::handleReadOffset, this);
}

RobotInterface::~RobotInterface()
{
}

/**
 * Factory method for DXLJoint structs
 **/
boost::shared_ptr<Joint> RobotInterface::createJoint(const std::string& name)
{
	boost::shared_ptr<DXLJoint> joint = boost::make_shared<DXLJoint>(name);

	joint->commandGenerator = commandGenerator(joint->type());
	joint->voltage = 120;
	joint->temperature = 0;

	boost::function<void()> updateCb = boost::bind(&RobotInterface::updateJointSettings, this, joint.get());
	joint->type.setCallback(boost::bind(updateCb));
	joint->readFeedback.setCallback(boost::bind(updateCb));
	updateJointSettings(joint.get());

	return joint;
}

RobotInterface::DXLJoint* RobotInterface::dxlJointForID(int id)
{
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		DXLJoint* joint = dxlJoint(i);
		if(joint->id() == id)
			return joint;
	}

	return 0;
}

void RobotInterface::updateJointSettings(RobotInterface::DXLJoint* joint)
{
	// The only thing that is not polled in every iteration is the command
	// generator, so update it here depending on the joint type.
	joint->commandGenerator = commandGenerator(joint->type());

	std::vector<int>::iterator it = std::find(
		m_cm730_queryset.begin(), m_cm730_queryset.end(), joint->id()
	);

	if(joint->readFeedback())
	{
		if(it == m_cm730_queryset.end())
			m_cm730_queryset.push_back(joint->id());
	}
	else
	{
		if(it != m_cm730_queryset.end())
			m_cm730_queryset.erase(it);
	}
}

/**
 * Connect to the robot
 **/
bool RobotInterface::init(RobotModel* model)
{
	m_model = model;

	int max_addr = 0;
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		const DXLJoint* joint = dxlJoint(i);

		if(joint->id() <= 0)
		{
			ROS_ERROR("Invalid joint id %d for joint '%s', ignoring this joint for now", joint->id(), joint->name.c_str());
			continue;
		}

		if(joint->id() > max_addr)
			max_addr = joint->id();
	}

	std::sort(m_cm730_queryset.begin(), m_cm730_queryset.end());

	// Initialize the CM730
	m_board.reset(new CM730);
	m_board->setQuerySet(m_cm730_queryset);
	m_servoData.resize(max_addr);

	if(m_board->connect() < 0)
	{
		ROS_ERROR("Could not initialize CM730");
		return false;
	}

	// Enable power for the arms (they have a power MOSFET on the CM730 board).
	m_board->writeByte(200, 24, 1);

	m_statisticsTimer.start();

	return true;
}


bool RobotInterface::readJointStates()
{
	if(m_skipStep)
	{
		m_skipStep = false;
		return true;
	}

	int ret = m_board->bulkRead(&m_servoData, &m_boardData);
	if(ret != 0)
	{
		switch(ret)
		{
			case CM730::RX_CORRUPT:
			{
				ROS_ERROR_THROTTLE(0.1, "bulk read failed: corrupt");
				m_skipStep = true;

				int id = m_board->lastFailedID();
				DXLJoint* joint = dxlJointForID(id);
				if(joint)
					joint->diag.checksum_errors++;

				return false;
				break;
			}
			case CM730::RX_FAIL:
				ROS_ERROR_THROTTLE(0.1, "bulk read failed: fail");
				return false;
				break;
			case CM730::RX_TIMEOUT:
			{
// 				std::stringstream ss;
// 				BOOST_FOREACH(int id, m_cm730_queryset)
// 				{
// 					ss << id << " ";
// 				}
//
// 				ROS_ERROR_THROTTLE(0.1, "bulk read failed: timeout (id %d), order was %s", m_board->lastFailedID(), ss.str().c_str());

				int id = m_board->lastFailedID();

				if(id == 0)
				{
					ROS_ERROR("bulk read timeout for CM730!");
				}
				else
				{
					DXLJoint* joint = dxlJointForID(id);
					if(joint)
						joint->diag.timeouts++;
				}

// 				if(id != 0)
// 				{
// 					// Move the ID to the end of the chain.
// 					// NOT: The ID before that might also be faulty, so move it as well.
//
// 					std::vector<int>::iterator it = std::find(
// 						m_cm730_queryset.begin(), m_cm730_queryset.end(), id
// 					);
// 					assert(it != m_cm730_queryset.end());
//
// // 					if(it != m_cm730_queryset.begin())
// // 					{
// // 						std::vector<int>::iterator itb = it - 1;
// // 						int idb = *itb;
// // 						it = m_cm730_queryset.erase(itb);
// //
// // 						m_cm730_queryset.push_back(idb);
// // 					}
//
// 					m_cm730_queryset.erase(it);
// 					m_cm730_queryset.push_back(id);
//
// 					m_board->setQuerySet(m_cm730_queryset);
// 				}

				m_skipStep = true;
			}
				break;
			default:
				ROS_ERROR_THROTTLE(0.1, "bulk read failed: unknown error");
				return false;
		}
	}

	ros::Time stamp = ros::Time::now();

	// Write everything into the joint structs
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		DXLJoint* joint = dxlJoint(i);
		int idx = joint->id()-1;
		const BRData& data = m_servoData[idx];

		joint->feedback.stamp = stamp;

		if(joint->readFeedback())
		{
			joint->feedback.pos = (M_PI / 2048.0) * (data.position - joint->tickOffset());

			if(joint->invert())
				joint->feedback.pos = -joint->feedback.pos;

			// Give the command generator the information it needs
			int pValue = joint->cmd.effort * 32.0;
			if(pValue == 0)
				pValue = 2;

			joint->commandGenerator->setPValue(pValue);
			joint->commandGenerator->setVoltage(m_statVoltage);

			// Estimate the produced torque using the position displacement
			joint->feedback.torque = joint->commandGenerator->servoTorqueFromCommand(
				joint->cmd.rawCmd, joint->feedback.pos, joint->cmd.vel
			);
		}
		else
		{
			joint->feedback.pos = joint->cmd.pos;
			joint->feedback.torque = joint->feedback.modelTorque;
		}
	}

	Eigen::Vector3d cmp;

	cmp << m_boardData.compassX, m_boardData.compassY, m_boardData.compassZ;
	cmp /= 860;

	m_compassFilter.update(cmp);

	m_model->setMagneticFieldVector(m_compassFilter.value());


	// IMU data
	const double UPDATE_RATE = 0.008;
	const double GYRO_TO_RAD = (M_PI/180.0) * 250.0/512.0;

	double gyro_pitch = -GYRO_TO_RAD * (m_boardData.gyroY - 512);
	double gyro_roll = GYRO_TO_RAD * (m_boardData.gyroX - 512);
	double gyro_yaw = GYRO_TO_RAD * (m_boardData.gyroZ - 512);

	m_model->setRobotAngularVelocity(tf::Vector3(-gyro_roll, gyro_pitch, -gyro_yaw));

	m_angleEstimator.predict(
		UPDATE_RATE * gyro_pitch,
		UPDATE_RATE * gyro_roll,
		0
	);

	int accX = -(m_boardData.accelY - 512);
	int accZ = -(m_boardData.accelZ - 512);
	int accY =  (m_boardData.accelX - 512);

	m_angleEstimator.update(
		accX,
		accY,
		accZ
	);

	tf::Quaternion quat;
	quat.setEuler(
		m_angleEstimator.pitch(),
		m_angleEstimator.roll(),
		0
	);

	// Plot the angles
	// FIXME: Maybe at a lower update rate?
	plot_msgs::Plot plot;

	plot.header.stamp = ros::Time::now();
	plot.points.resize(4);

	plot.points[0].name = "AngleEstimator/Pitch";
	plot.points[0].value = m_angleEstimator.pitch();

	plot.points[1].name = "AngleEstimator/Roll";
	plot.points[1].value = m_angleEstimator.roll();

	plot.points[2].name = "AngleEstimator/GPitch";
	plot.points[2].value = GYRO_TO_RAD * (m_boardData.gyroY - 512);

	plot.points[3].name = "AngleEstimator/GRoll";
	plot.points[3].value = GYRO_TO_RAD * (m_boardData.gyroX - 512);

	m_pub_angle_plot.publish(plot);

	m_model->setRobotOrientation(quat);

	// Handle button presses
	for(int i = 0; i <= 2; ++i)
	{
		bool state = (m_boardData.button & (1 << i));
		bool lastState = m_lastButtons & (1 << i);

		if(!state && lastState)
		{
			Button btn;
			btn.button = i;
			btn.time = ros::Time::now();

			m_pub_buttons.publish(btn);
		}
	}
	m_lastButtons = m_boardData.button;

	return true;
}

/**
 * Get command generator for dynamixel type
 **/
CommandGeneratorPtr RobotInterface::commandGenerator(const std::string& type)
{
	if(m_generators.count(type))
		return m_generators[type];

	CommandGeneratorPtr gen;
	gen.reset(new DynamicCommandGenerator("models/" + type + "/"));

	m_generators[type] = gen;

	return gen;
}

void RobotInterface::handleLEDCommand(const LEDCommand& cmd)
{
	for(int i = 0; i < 3; ++i)
	{
		if(cmd.mask & (1 << i))
		{
			if(cmd.state & (1 << i))
				m_ledCommand.state |= (1 << i);
			else
				m_ledCommand.state &= ~(1 << i);
		}
	}

	if(cmd.mask & LEDCommand::LED5)
		m_ledCommand.rgb5 = cmd.rgb5;

	if(cmd.mask & LEDCommand::LED6)
		m_ledCommand.rgb6 = cmd.rgb6;
}

struct CM730SyncWriteData
{
	uint8_t id;
	uint8_t panel_leds;
	uint16_t led5;
	uint16_t led6;
} __attribute__((packed));

/**
 * @internal
 * Send CM730 data like LED commands
 **/
void RobotInterface::sendCM730Data()
{
	std::vector<uint8_t> params(sizeof(CM730SyncWriteData));
	CM730SyncWriteData* paramData = (CM730SyncWriteData*)&params[0];

	paramData->id = CM730::ID_CM;
	paramData->panel_leds = 0;

	for(int i = 0; i < 3; ++i)
	{
		if(m_ledCommand.state & (1 << i))
			paramData->panel_leds |= (1 << i);
	}

	paramData->led5 =
		  ((int)(m_ledCommand.rgb5.r * 31) << 0)
		| ((int)(m_ledCommand.rgb5.g * 31) << 5)
		| ((int)(m_ledCommand.rgb5.b * 31) << 10)
	;

	paramData->led6 =
		  ((int)(m_ledCommand.rgb6.r * 31) << 0)
		| ((int)(m_ledCommand.rgb6.g * 31) << 5)
		| ((int)(m_ledCommand.rgb6.b * 31) << 10)
	;

	if(m_board->syncWrite(CM730::P_LED_PANNEL, sizeof(CM730SyncWriteData), 1, &params[0]) != 0)
	{
		ROS_ERROR("SyncWrite for CM730 failed");
	}
}


/**
 * @internal
 * This is a helper struct for the SyncWrite command which transfers all
 * command values to the servo actuators. The memory layout reflects the
 * actual layout in the message and in the Dynamixel hardware, hence the
 * `__attribute__((packed))`.
 **/
struct SyncWriteData
{
	uint8_t id;
	uint8_t pGain;
	uint8_t nothing;
	uint16_t goalPosition;
} __attribute__((packed));

bool RobotInterface::sendJointTargets()
{
	// Get a uint8_t buffer with enough bytes for our packet
	std::vector<uint8_t> params(m_model->numJoints() * sizeof(SyncWriteData));

	// And map the SyncWriteData struct on it
	SyncWriteData* paramData = (SyncWriteData*)&params[0];

	// Fill in sync write buffer
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		DXLJoint* joint = dxlJoint(i);

		SyncWriteData* data = paramData + i;

		// The effort slope is limited.
		joint->realEffort = slopeLimited<double>(joint->realEffort, joint->cmd.effort, m_effortSlope());

		uint8_t pValue = joint->realEffort * 32.0;
		if(pValue == 0)
			pValue = 2;

		// Give the command generator the information it needs
		joint->commandGenerator->setPValue(pValue);
// 		joint->commandGenerator->setVoltage(m_statVoltage);
		joint->commandGenerator->setVoltage(0.1 * m_boardData.voltage);

		double cmd = joint->cmd.pos;

		// If requested, use the servo model to generate the final position command
		// We create a linear mix between the raw command (goal position) and
		// the command generated by the servo model. The slope of the coefficient
		// is limited by the m_rawSlope().
		double rawGoal = (!joint->cmd.raw && m_useModel()) ? 1 : 0;
		joint->rawState = slopeLimited<double>(joint->rawState, rawGoal, m_rawSlope());

		double modelCmd = joint->commandGenerator->servoCommandFor(
			joint->cmd.pos, joint->cmd.vel, joint->cmd.acc,
			joint->feedback.modelTorque
		);

		cmd = joint->rawState * modelCmd + (1.0 - joint->rawState) * cmd;

		// Save the generated command for plotting and logging
		joint->cmd.rawCmd = cmd;

		// Inversion and offset stuff
		if(joint->invert())
			cmd = -cmd;

		int goal = (2048.0 / M_PI) * cmd + joint->tickOffset();

		// Capping
		// FIXME: Increase P if saturation occurs
		// FIXME: Provide some sort of statistics/warnings if this occurs
		if(goal >= 4096)
			goal = 4095;
		else if(goal < 0)
			goal = 0;

		// Write the calculated values into the buffer
		data->id = joint->id();
		data->pGain = pValue;
		data->goalPosition = goal;
	}

	if(m_relaxed)
		return true; // Relaxed robots won't do anything. They are lazy.

	if(m_board->syncWrite(28, sizeof(SyncWriteData), m_model->numJoints(), &params[0]) != 0)
	{
		ROS_ERROR("SyncWrite failed");
		return false;
	}

	return true;
}

/**
 * @internal
 * Reflects the message layout of the SyncWrite used for torque fading.
 **/
struct TorqueSyncWriteData
{
	uint8_t id;
	uint16_t torque;
} __attribute__((packed));

struct TorqueOffWriteData
{
	uint8_t id;
	uint8_t off;
} __attribute__((packed));

bool RobotInterface::setStiffness(float torque)
{
	ROS_INFO_THROTTLE(1.0, "Fading is active (%f)", torque);
	if(torque == 0)
	{
		// Really relax everything (switch torque off)
		std::vector<uint8_t> param;
		param.resize(m_model->numJoints() * sizeof(TorqueOffWriteData));
		TorqueOffWriteData* data = (TorqueOffWriteData*)&param[0];

		for(size_t i = 0; i < m_model->numJoints(); ++i)
		{
			data[i].id = dxlJoint(i)->id();
			data[i].off = 0.0;
		}

		if(m_board->syncWrite(24, sizeof(TorqueOffWriteData), m_model->numJoints(), &param[0]) != 0)
		{
			ROS_ERROR("Could not switch torque off");
			return false;
		}
		m_relaxed = true;
	}
	else
	{
// 		m_board->writeByte(CM730::ID_CM, 24, 1);
		m_relaxed = false;
	}

	// Send out a SyncWrite for the MAX_TORQUE register
	std::vector<uint8_t> param;
	param.resize(m_model->numJoints() * sizeof(TorqueSyncWriteData));
	TorqueSyncWriteData* data = (TorqueSyncWriteData*)&param[0];

	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		data[i].id = dxlJoint(i)->id();
		data[i].torque = 1023.0 * torque;
	}
	if(m_board->syncWrite(34, sizeof(TorqueSyncWriteData), m_model->numJoints(), &param[0]) != 0)
	{
		ROS_ERROR("Torque SyncWrite failed");
		return false;
	}

	return true;
}

/**
 * @internal
 * Layout of the statistics data requested from the servo
 **/
struct StatData
{
	uint8_t voltage;
	uint8_t temperature;
} __attribute__((packed));

/**
 * We query a single joint for its statistics data (voltage + temperature).
 * In the next callback, the next joint is asked.
 **/
void RobotInterface::handleStatistics(const ros::TimerEvent& )
{
// 	DXLJoint* joint = dxlJoint(m_statIndex);
//
// 	StatData statData;
// 	if(m_board->readData(joint->id(), 42, &statData, sizeof(statData)) == 0)
// 	{
// 		joint->temperature = statData.temperature;
// 		joint->voltage = statData.voltage;
// 	}
//
// 	m_statVoltage = 0;
// 	for(size_t i = 0; i < m_model->numJoints(); ++i)
// 	{
// 		m_statVoltage += 0.1 * dxlJoint(i)->voltage;
// 	}
// 	m_statVoltage /= m_model->numJoints();
//
// 	if(++m_statIndex == m_model->numJoints())
// 		m_statIndex = 0;

	sendCM730Data();
}

/**
 * Reports battery voltage (mean of all servo voltages) and the maximum
 * temperature measured in the statistics callback over all servos.
 **/
void RobotInterface::getDiagnostics(robotcontrol::DiagnosticsPtr ptr)
{
	ptr->batteryVoltage = 0.1 * m_boardData.voltage;
	ptr->servoTemperature = 0;
	ptr->servos.clear();

	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		DXLJoint* joint = dxlJoint(i);
		if(ptr->servoTemperature < joint->temperature)
			ptr->servoTemperature = joint->temperature;

		ptr->servos.push_back(joint->diag);
	}
}

bool RobotInterface::handleReadOffsets(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	for(size_t i = 0; i < m_model->numJoints(); ++i)
	{
		DXLJoint* joint = dxlJoint(i);

		int present_pos;

		// Get current raw position
		if(m_board->readWord(joint->id(), 36, &present_pos) != 0)
			continue;

		// .. and set it as the offset.
		joint->tickOffset.set(present_pos);
	}
	return true;
}

bool RobotInterface::handleReadOffset(ReadOffsetRequest& req, ReadOffsetResponse& resp)
{
	DXLJoint* joint = (DXLJoint*)m_model->getJoint(req.joint).get();
	if(!joint)
		return false;

	int present_pos;

	// Get current raw position
	if(m_board->readWord(joint->id(), 36, &present_pos) != 0)
		return false;

	ROS_INFO("%s raw position: %d", req.joint.c_str(), present_pos);

	resp.ticks = joint->tickOffset() - present_pos;

	// .. and set it as the offset.
	joint->tickOffset.set(present_pos);

	return true;
}

}

PLUGINLIB_EXPORT_CLASS(robotcontrol::RobotInterface, robotcontrol::HardwareInterface);
