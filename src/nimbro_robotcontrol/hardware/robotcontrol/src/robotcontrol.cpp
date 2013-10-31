// Robot control node
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <robotcontrol/robotcontrol.h>
#include <robotcontrol/hw/robotinterface.h>
#include <robotcontrol/hw/dummyinterface.h>
#include <motiontimer.h>
#include <nanosleeptimer.h>

#include <sched.h>
#include <signal.h>
#include <sys/timerfd.h>

#include <pluginlib/class_loader.h>

#include <visualization_msgs/MarkerArray.h>

#include <XmlRpcValue.h>

#if defined(__i386__)
#define __NR_ioprio_set         289
#define __NR_ioprio_get         290
#elif defined(__ppc__)
#define __NR_ioprio_set         273
#define __NR_ioprio_get         274
#elif defined(__x86_64__)
#define __NR_ioprio_set         251
#define __NR_ioprio_get         252
#elif defined(__ia64__)
#define __NR_ioprio_set         1274
#define __NR_ioprio_get         1275
#else
#error "Unsupported arch"
#endif

static inline int ioprio_set(int which, int who, int ioprio)
{
        return syscall(__NR_ioprio_set, which, who, ioprio);
}

static inline int ioprio_get(int which, int who)
{
        return syscall(__NR_ioprio_get, which, who);
}

enum {
        IOPRIO_CLASS_NONE,
        IOPRIO_CLASS_RT,
        IOPRIO_CLASS_BE,
        IOPRIO_CLASS_IDLE,
};

enum {
        IOPRIO_WHO_PROCESS = 1,
        IOPRIO_WHO_PGRP,
        IOPRIO_WHO_USER,
};

#define IOPRIO_CLASS_SHIFT      13

namespace robotcontrol
{

RobotControl::RobotControl()
 : m_nh("~")
 , m_hwLoader("robotcontrol", "robotcontrol::HardwareInterface")
 , m_pluginLoader("robotcontrol", "robotcontrol::MotionModule")
 , m_pub_js_counter(0)
 , m_fadeTorqueServer(m_nh, "fade_torque", false)
 , m_fadeTorqueClient("/robotcontrol/fade_torque")
 , m_fadeTorqueState(0)
 , m_velLimit("velLimit", 0, 0.05, 25.0, 0.5)
 , m_accLimit("accLimit", 0, 0.05, 25.0, 0.5)
 , m_publishCommand("publishCommand", false)
{
	// Setup publishers & subscribers for our topics
	m_pub_js = m_nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
	m_pub_js_cmd = m_nh.advertise<sensor_msgs::JointState>("sent_joint_cmds", 1);

	m_sub_js = m_nh.subscribe("/joint_state_cmds", 1, &RobotControl::handleJointStateCommand, this);
	m_sub_js_raw = m_nh.subscribe("raw_cmds", 1, &RobotControl::handleRawJointCommand, this);

	m_pub_diag = m_nh.advertise<robotcontrol::Diagnostics>("diagnostics", 1);

	m_pub_markers = m_nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);

	m_sub_btn = m_nh.subscribe("/button", 1, &RobotControl::handleButton, this);

	m_nh.param("publish_tf", m_publishTF, true);

	// The diagnostics message is sent out in a periodic timer
	m_diagnosticsTimer = m_nh.createTimer(ros::Duration(1.0), &RobotControl::sendDiagnostics, this);
	m_diagnosticsTimer.start();

	// Start handling fade in/out requests
	m_fadeTorqueServer.start();

	// We want to be notified if the joint limits change
	m_velLimit.setCallback(boost::bind(&RobotControl::handleVelLimitUpdate, this, _1));
	m_accLimit.setCallback(boost::bind(&RobotControl::handleAccLimitUpdate, this, _1));

	m_state_init = m_robotModel.registerState("init");
	m_state_relaxed = m_robotModel.registerState("relaxed");
	m_robotModel.setState(m_state_relaxed);
}

RobotControl::~RobotControl()
{
	// This is a clean shutdown, so relax the robot. DANGEROUS!
// 	m_hw->setStiffness(0);
}

/**
 * The enabled modules are described in a "motion_modules"
 * list on the parameter server.
 *
 * @return true on success
 **/
bool RobotControl::initModules()
{
	XmlRpc::XmlRpcValue list;
	m_nh.getParam("motion_modules", list);
	ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int i = 0; i < list.size(); ++i)
	{
		std::string name = static_cast<std::string>(list[i]);

		boost::shared_ptr<MotionModule> module;

		ROS_INFO("Loading plugin '%s'", name.c_str());

		try
		{
			module = m_pluginLoader.createInstance(name);
			if(!module)
			{
				ROS_ERROR("Could not load plugin '%s'", name.c_str());
				ROS_ERROR("I'm going to continue anyway...");
				continue;
			}
		}
		catch(pluginlib::LibraryLoadException& exc)
		{
			ROS_ERROR("Could not load plugin '%s': %s", name.c_str(), exc.what());
			ROS_ERROR("I'm going to continue anyway...");
			continue;
		}

		if(!module->init(&m_robotModel))
		{
			ROS_ERROR("Could not initialize plugin '%s'", name.c_str());
			ROS_ERROR("I'm going to continue anyway...");
			continue;
		}
		m_modules.push_back(module);
	}

	return true;
}

/**
 * Initializes the hardware interface, the robot model and the motion modules.
 *
 * @return true on success
 **/
bool RobotControl::init()
{
	// Hardware interface
	std::string hwInterface;
	m_nh.param("hw_interface", hwInterface, std::string("robotcontrol::RobotInterface"));

	m_hw = m_hwLoader.createInstance(hwInterface);
	if(!m_hw)
	{
		ROS_ERROR("Could not load hw interface plugin");
		return false;
	}

	// Robot model (from URDF model on parameter server)
	m_model = boost::make_shared<urdf::Model>();
	if(!m_model->initParam("robot_description"))
	{
		ROS_ERROR("Could not get URDF model");
		return false;
	}

	m_robotModel.setModel(m_model);

	// Create robotcontrol::Joint (or subclasses, this is decided by the
	// factory method on the HardwareInterface) for every joint in the model.
	std::vector<boost::shared_ptr<urdf::Link> > links;
	m_model->getLinks(links);

	for(size_t i = 0; i < links.size(); ++i)
	{
		// The urdf link belonging to the joint
		const boost::shared_ptr<urdf::Link>& link = links[i];

		// The urdf joint corresponding to the joint
		const boost::shared_ptr<urdf::Joint>& modelJoint = link->parent_joint;

		// Ignore joints we don't need to handle (e.g. fixed joints)
		if(!modelJoint || (
				   modelJoint->type != urdf::Joint::CONTINUOUS
				&& modelJoint->type != urdf::Joint::REVOLUTE))
			continue;

		boost::shared_ptr<Joint> joint = m_hw->createJoint(modelJoint->name.c_str());
		if(!joint)
		{
			ROS_ERROR("Could not create Joint for name '%s'", modelJoint->name.c_str());
			return false;
		}

		joint->modelJoint = modelJoint;
		joint->name = modelJoint->name;

		m_robotModel.addJoint(joint);
	}

	if(m_robotModel.numJoints() == 0)
	{
		ROS_ERROR("No joints created. Something is wrong with the URDF model.");
		return false;
	}

	// Initialize the robot model's kinematic trees for each support foot
	m_robotModel.initTrees();

	// Hardware interface initialization
	if(!m_hw->init(&m_robotModel))
	{
		ROS_ERROR("Could not initialize hw interface");
		return false;
	}
	// The robot is relaxed initially to prevent accidents
	m_hw->setStiffness(0);

	// Set a default joint effort
	double defaultEffort;
	m_nh.param("default_effort", defaultEffort, 0.2);
	for(size_t i = 0; i < m_robotModel.numJoints(); ++i)
	{
		m_robotModel.joint(i)->cmd.effort = defaultEffort;
	}

	// Initialize all loaded motion modules
	if(!initModules())
	{
		ROS_ERROR("Could not initialize motion modules");
		return false;
	}

	// Initialize the velocity/acceleration limits on the joints
	handleVelLimitUpdate(m_velLimit());
	handleAccLimitUpdate(m_accLimit());

	ROS_INFO("Initialization finished.");

	return true;
}

/**
 * This gets called when a JointState message arrives on the /joint_state_cmds
 * topic. This topic is mainly used for quick tests (e.g. quickly send a
 * position to a single joint) and is not recommended for real robot operation.
 *
 * Use a MotionModule instead.
 *
 * @param cmd the command in JointState format
 * @param raw set the Joint::Command::raw flag, which indicates that the servo
 *   command should not be translated by the HardwareInterface
 **/
void RobotControl::doHandleJSCommand(const sensor_msgs::JointStatePtr& cmd, bool raw)
{
	for(size_t i = 0; i < cmd->name.size(); ++i)
	{
		boost::shared_ptr<Joint> joint = m_robotModel.getJoint(cmd->name[i]);
		if(!joint)
		{
			ROS_WARN("Got joint state command for unknown joint: '%s'", cmd->name[i].c_str());
			continue;
		}

		double pos = cmd->position[i];

		// Have a quick look at the URDF limits
		boost::shared_ptr<urdf::JointLimits> limits = joint->modelJoint->limits;
		if(!raw && limits)
		{
			if(pos > limits->upper)
			{
				ROS_WARN("Joint '%s' is above upper limit: %5.3lf > %5.3lf",
					joint->name.c_str(), pos, limits->upper
				);
				pos = limits->upper;
			}
			else if(pos < limits->lower)
			{
				ROS_WARN("Joint '%s' is below lower limit: %5.3lf < %5.3lf",
					joint->name.c_str(), pos, limits->lower
				);
				pos = limits->lower;
			}
		}

		joint->cmd.raw = raw;

		// If we got velocity information, use it.
		if(cmd->velocity.size())
			joint->cmd.setFromPosVel(cmd->header.stamp, pos, cmd->velocity[i]);
		else
			joint->cmd.setFromPos(cmd->header.stamp, pos);
	}
}

void RobotControl::handleJointStateCommand(const sensor_msgs::JointStatePtr& cmd)
{
	doHandleJSCommand(cmd, false);
}

void RobotControl::handleRawJointCommand(const sensor_msgs::JointStatePtr& cmd)
{
	doHandleJSCommand(cmd, true);
}

/**
 * This takes care of all real-time specific stuff that needs to be done
 * in every iteration.
 **/
void RobotControl::step()
{
	// Handle torque fade requests
	if(m_robotModel.isRelaxed())
	{
		m_hw->setStiffness(0.0);
		m_fadeTorqueState = 0.0;
	}

	if(m_fadeTorqueServer.isActive())
	{
		const float MAX_DELTA = 0.005;
		float diff = m_fadeTorqueGoal->torque - m_fadeTorqueState;
		if(diff > MAX_DELTA)
			diff = MAX_DELTA;
		else if(diff < -MAX_DELTA)
			diff = -MAX_DELTA;

		if(fabs(diff) < 0.001)
		{
			m_fadeTorqueState = m_fadeTorqueGoal->torque;
			m_fadeTorqueServer.setSucceeded();

			if(m_fadeTorqueState > 0.5)
				m_robotModel.setState(m_state_init);
			else
				m_robotModel.setState(m_state_relaxed);
		}
		else
			m_fadeTorqueState += diff;

		m_hw->setStiffness(m_fadeTorqueState);

		FadeTorqueFeedbackPtr fb = boost::make_shared<FadeTorqueFeedback>();
		fb->current_torque = m_fadeTorqueState;
		m_fadeTorqueServer.publishFeedback(fb);
	}
	else if(m_fadeTorqueServer.isNewGoalAvailable())
		m_fadeTorqueGoal = m_fadeTorqueServer.acceptNewGoal();

	ros::Time t0 = ros::Time::now();
	// Ask all loaded motion modules for their input.
	for(size_t i = 0; i < m_modules.size(); ++i)
	{
		if(m_modules[i]->isTriggered())
			m_modules[i]->step();
	}

	// Calculate the inverse dynamics (i.e. the needed torques) on our
	// robot model
	m_robotModel.doInverseDynamics();

	// Do the hardware communication cycle
	ros::Time t1 = ros::Time::now();
	m_dur_motion = t1 - t0;
	m_hw->sendJointTargets();
	ros::Time t2 = ros::Time::now();
	m_dur_tx = t2 - t1;

	m_hw->readJointStates();
	ros::Time t3 = ros::Time::now();
	m_dur_rx = t3 - t2;

	// Publish joint states and other information
	if(m_pub_js_counter == 1)
	{
		publishJointStates();
		publishJointStateCommands();

		if(m_publishTF)
		{
			if(m_publishCommand())
			{
				ROS_WARN_THROTTLE(1.0, "robotcontrol/publishCommand is active! This will send invalid transforms to other nodes!");
				m_robotModel.publishTF(false);
			}
			else
				m_robotModel.publishTF(true);
		}

		for(size_t i = 0; i < m_modules.size(); ++i)
			m_modules[i]->publishTransforms();

		// Publish visualization markers
		visualization_msgs::MarkerArray markers;
		m_robotModel.visualizeData(&markers);
		m_pub_markers.publish(markers);

		m_pub_js_counter = 0;
	}
	else
		m_pub_js_counter++;
}

void RobotControl::publishJointStates()
{
	sensor_msgs::JointStatePtr js = boost::make_shared<sensor_msgs::JointState>();

	size_t nj = m_robotModel.numJoints();
	js->name.resize(nj);
	js->effort.resize(nj);
	js->position.resize(nj);
	js->velocity.resize(nj);
	js->header.stamp = m_robotModel[0]->feedback.stamp;

	for(size_t i = 0; i < nj; ++i)
	{
		const Joint& joint = *m_robotModel[i];

		js->name[i] = joint.modelJoint->name.c_str();

		if(m_publishCommand())
			js->position[i] = joint.cmd.pos;
		else
			js->position[i] = joint.feedback.pos;

		js->velocity[i] = 0; // FIXME
		js->effort[i] = joint.feedback.torque;
	}

	m_pub_js.publish(js);
}

void RobotControl::publishJointStateCommands()
{
	sensor_msgs::JointStatePtr js = boost::make_shared<sensor_msgs::JointState>();

	size_t nj = m_robotModel.numJoints();
	js->name.resize(nj);
	js->effort.resize(nj);
	js->position.resize(nj);
	js->velocity.resize(nj);
	js->header.stamp = m_robotModel[0]->feedback.stamp;

	for(size_t i = 0; i < nj; ++i)
	{
		const Joint& joint = *m_robotModel[i];

		js->name[i] = joint.modelJoint->name.c_str();
		js->position[i] = joint.cmd.pos;
		js->velocity[i] = joint.cmd.vel;
		js->effort[i] = joint.cmd.rawCmd;
	}

	m_pub_js_cmd.publish(js);
}

/**
 * This just asks the hardware interface to fill in the Diagnostics message
 * before sending it off.
 **/
void RobotControl::sendDiagnostics(const ros::TimerEvent& )
{
	robotcontrol::DiagnosticsPtr msg = boost::make_shared<robotcontrol::Diagnostics>();

	msg->header.stamp = ros::Time::now();
	msg->state = m_robotModel.currentStateLabel();

	m_hw->getDiagnostics(msg);

	m_pub_diag.publish(msg);
}

void RobotControl::handleVelLimitUpdate(float value)
{
	Joint::Command::velLimit = value;
}

void RobotControl::handleAccLimitUpdate(float value)
{
	Joint::Command::accLimit = value;
}

void RobotControl::handleButton(const Button& btn)
{
	if(btn.button != 0)
		return;

	RobotModel::State state = m_robotModel.state();

	if(state == m_state_relaxed)
	{
		FadeTorqueGoal goal;
		goal.torque = 1.0;
		m_fadeTorqueClient.sendGoal(goal);
		return;
	}

	FadeTorqueGoal goal;
	goal.torque = 0.0;
	m_fadeTorqueClient.sendGoal(goal);
}

}

bool g_shouldShutdown = false;

void signal_handler(int sig)
{
	g_shouldShutdown = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotcontrol", ros::init_options::NoSigintHandler);
	robotcontrol::RobotControl ctrl;
	if(!ctrl.init())
	{
		ROS_ERROR("Could not initialize RobotControl");
		return 1;
	}

	// Handle SIGINT ourselves for a clean and controlled shutdown
	signal(SIGINT, signal_handler);

	// Get realtime priority
	sched_param schedparm;
	memset(&schedparm, 0, sizeof(schedparm));
	schedparm.sched_priority = 1;
	if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedparm) != 0)
	{
		ROS_ERROR("Could not get realtime priority");
		ROS_ERROR("I'm going to run without realtime priority!");
	}

	// Set I/O priority to realtime
	ioprio_set(IOPRIO_WHO_PROCESS, getpid(), (IOPRIO_CLASS_RT << 13) | 0);

	// FIXME: Have the 0.008s somewhere available as a constant
	config_server::Parameter<float> duration("timerDuration", 0.008, 0.0001, 0.050, 0.008);
	MotionTimer timer(duration());
	config_server::Parameter<bool> pause("pause", false);

	ros::Duration execTime;

	while(!g_shouldShutdown)
	{
		// Do all ROS callbacks before sleeping, so the next step() iteration
		// starts right after the next timer tick
		ros::spinOnce();

		uint64_t expirations = timer.sleep();

		if(expirations > 1)
		{
			if(expirations > 2)
			{
				ROS_WARN("Missed %lu timer cycles, exec took %lf secs", expirations-1, execTime.toSec());
				ctrl.printTimeDiagnostics();
			}
			else
				ROS_INFO_THROTTLE(0.1, "Missed a timer cycle, exec took %lf secs", execTime.toSec());
		}

		ros::Time start = ros::Time::now();
		if(!pause())
			ctrl.step();
		execTime = (ros::Time::now() - start);
	}

	return 0;
}

void robotcontrol::RobotControl::printTimeDiagnostics()
{
	ROS_INFO("timings: motion %lf, tx %lf, rx %lf", m_dur_motion.toSec(), m_dur_tx.toSec(), m_dur_rx.toSec());
}

