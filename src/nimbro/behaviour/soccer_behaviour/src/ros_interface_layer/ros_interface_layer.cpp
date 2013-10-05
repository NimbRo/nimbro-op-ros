// Behaviour Control Framework - ROS Interface Layer
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <soccer_behaviour/ros_interface_layer/ros_interface_layer.h>
#include <soccer_behaviour/soccer_manager/soccer_manager.h>
#include <soccer_behaviour/control_layer/control_layer.h>

// Namespaces
using namespace std;
using namespace soccerbehaviour;

//
// RosInterfaceLayer class
//

// Contructor
RosInterfaceLayer::RosInterfaceLayer(SoccerManager* M) : BehaviourLayer(M, "RosInterfaceLayer", true), M(M)
{
	// Create sensor and actuator managers
	SM = new RosInterfaceLayerSM(this);
	AM = new RosInterfaceLayerAM(this);
}

// Destructor
RosInterfaceLayer::~RosInterfaceLayer()
{
	halt();

	// Delete sensor and actuator managers
	delete SM;
	delete AM;
}

// Initialisation function
ret_t RosInterfaceLayer::init()
{
	// Return that initialisation was successful
	return RET_OK;
}

// Update function
void RosInterfaceLayer::update()
{
}

// Post-execute callback
void RosInterfaceLayer::postExecuteCallback()
{
}

void RosInterfaceLayer::halt()
{
	SM->halt();
}

// Extra stuff
/* TODO: Define other functions that you need here */

//
// RosInterfaceLayerSM class
//

// Initialisation function
ret_t RosInterfaceLayerSM::init()
{
	// Retrieve the ROS node handle from the layer
	ros::NodeHandle* nh = &L->nh;

	// Advertise the required data topics
	m_pub_gaitCommand = nh->advertise<gait::GaitCommand>("/gaitCmd", 1);
	m_pub_headControlTarget = nh->advertise<head_control::LookAtTarget>("/robotcontrol/headcontrol/target", 1);

	// Subscribe to the required services
	m_srv_playMotion.setServiceClient(nh->serviceClient<motion_player::PlayMotion>("/motion_player/play"));

	// Return that initialisation was successful
	return RET_OK;
}

// Write external data function
void RosInterfaceLayerSM::writeExternalData()
{
	// Publish the required sensor data
	m_pub_gaitCommand.publish(gaitCommand.read());
	m_pub_headControlTarget.publish(headControlTarget.read());


	// Call the required services
	if(playMotion.wasWrittenTo()) // Normally this is inside the RosServiceCaller, but in this special case our sensor has a different type to our data type so we can't pass it in
	{
		KeyMotionEnum motionType = playMotion.read();
		if((motionType > KM_NO_MOTION) && (motionType < KM_NUM_MOTIONS))
		{
			motion_player::PlayMotion motion;
			motion.request.name = KeyMotionName[motionType];
			m_srv_playMotion.callService(motion);
		}
	}
}

void RosInterfaceLayerSM::halt()
{
	// Send a command to stop the robot from walking
	head_control::LookAtTarget head;
	head.is_angular_data = true;
	head.is_relative = false;
	head.vec.x = 0;
	head.vec.y = 0;
	head.vec.z = 0;
	gait::GaitCommand halt;
	halt.walk = false;
	m_pub_gaitCommand.publish(halt);
	m_pub_headControlTarget.publish(head);
}

//
// RosInterfaceLayerAM class
//

// Initialisation function
ret_t RosInterfaceLayerAM::init()
{
	// Retrieve the ROS node handle from the layer
	ros::NodeHandle* nh = &L->nh;

	// Subscribe to the required data topics
	m_sub_ballPosition = nh->subscribe("/ball/filteredPosition", 1, &RosInterfaceLayerAM::handleBallPosition, this);
	m_sub_gameControlData = nh->subscribe("/game_controller/data", 1, &RosInterfaceLayerAM::handleGameControlData, this);
	m_sub_compassHeading = nh->subscribe("/compass/heading", 1, &RosInterfaceLayerAM::handleCompassHeading, this);
	m_sub_visionDetections = nh->subscribe("/vision/detections", 1, &RosInterfaceLayerAM::handleVisionDetections, this);
	m_sub_button = nh->subscribe("/button", 1, &RosInterfaceLayerAM::handleButton, this);
	m_sub_robotState = nh->subscribe("/robotcontrol/state", 1, &RosInterfaceLayerAM::handleRobotState, this);
	m_sub_ballFocalPosition = nh->subscribe("/ball/focal_plane_ball_vector", 1, &RosInterfaceLayerAM::handleBallFocalPosition, this);

	// Return that initialisation was successful
	return RET_OK;
}

// ROS topic event handlers
void RosInterfaceLayerAM::handleBallPosition(const geometry_msgs::PointStampedConstPtr& msg)
{
	// Write the message data to the required actuator
	ballPosition.writeHard(msg);
}
void RosInterfaceLayerAM::handleBallFocalPosition(const geometry_msgs::PointStampedConstPtr& msg)
{
	// Write the message data to the required actuator
	ballFocalPosition.writeHard(msg);
}
void RosInterfaceLayerAM::handleVisionDetections(const soccer_vision::DetectionsConstPtr& msg)
{
	// Write the message data to the required actuator
	visionDetections.writeHard(msg);
}
void RosInterfaceLayerAM::handleButton(const robotcontrol::ButtonConstPtr& msg)
{
	// Write the message data to the required actuator
	button.writeHard(msg);

	if(msg->button != 1)
		return;

	// Toggle the enable state of the node if the middle button is pressed (button 1)
	switch(M->state())
	{
		case SoccerManager::STATE_HALT:
			M->setState(SoccerManager::STATE_WAIT);
			break;
		case SoccerManager::STATE_WAIT:
			M->setState(SoccerManager::STATE_RUN_MANUAL);
			break;
		case SoccerManager::STATE_RUN:
			M->setState(SoccerManager::STATE_HALT);
			break;
		case SoccerManager::STATE_RUN_MANUAL:
			M->setState(SoccerManager::STATE_HALT);
			break;
	}
}
void RosInterfaceLayerAM::handleCompassHeading(const robotcontrol::CompassHeadingConstPtr& msg)
{
	// Write the message data to the required actuator
	compassHeading.writeHard(msg);
}
void RosInterfaceLayerAM::handleGameControlData(const rcup_game_controller::GCDataConstPtr& msg)
{
	// Write the message data to the required actuator
	gameControlData.writeHard(msg);
}
void RosInterfaceLayerAM::handleRobotState(const robotcontrol::StateConstPtr& msg)
{
	// Write the message data to the required actuator
	robotState.writeHard(msg);
}

// Read external data function
void RosInterfaceLayerAM::readExternalData()
{
	// Note: The latest ROS topic data is read in the ros::spinOnce() function, which calls the required event handlers.
	//       These event handlers write to the required actuators, so we have nothing more that we need to do here.
	//       The ros::spinOnce() function is called in the main loop, NOT in the step() function of the soccer manager,
	//       so this asynchronicity does not break the strict ordering requirement of the behaviour control framework.
	//       In short, do NOT call ros::spinOnce() from within the callbacks of the framework - leave it in main()!
}
// EOF