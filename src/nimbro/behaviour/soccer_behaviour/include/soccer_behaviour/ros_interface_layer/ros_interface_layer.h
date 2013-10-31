// Soccer Behaviour - ROS Interface Layer
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file ros_interface_layer.h
* @brief Defines the ROS Interface Layer for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef ROS_INTERFACE_LAYER_H
#define ROS_INTERFACE_LAYER_H

// Includes - Sensor message types
#include <gait/GaitCommand.h>
#include <head_control/LookAtTarget.h>
//#include <keyframe_player_hack/PlayMotion.h> // TODO: remove all instances of the keyframplayer hack
#include <motion_player/PlayMotion.h>

// Includes - Actuator message types
#include <geometry_msgs/PointStamped.h>
#include <rcup_game_controller/GCData.h>
#include <robotcontrol/CompassHeading.h>
#include <soccer_vision/Detections.h>
#include <robotcontrol/Button.h>
#include <robotcontrol/State.h>

// Includes - Common
#include <soccer_behaviour/soccer_common.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	/**
	* @class RosInterfaceLayer
	*
	* @brief Implements the ROS Interface Layer
	**/
	class RosInterfaceLayer : public behaviourcontrol::BehaviourLayer
	{
	public:
		// Constructors
		explicit RosInterfaceLayer(SoccerManager* M);
		virtual ~RosInterfaceLayer();

		// Parent manager
		SoccerManager* const M;

		// Sensor and actuator managers
		RosInterfaceLayerSM* SM;
		RosInterfaceLayerAM* AM;

		// Function overrides
		virtual ret_t init();
		virtual void update();
		virtual void postExecuteCallback();

		// Halt function
		void halt();

		//
		// Extra stuff
		//

		// ROS node handle
		ros::NodeHandle nh;
	};

	/**
	* @class RosInterfaceLayerSM
	*
	* @brief Implements the sensor manager for the ROS Interface Layer
	**/
	class RosInterfaceLayerSM : public behaviourcontrol::SensorManager
	{
	public:
		// Constructors
		explicit RosInterfaceLayerSM(RosInterfaceLayer* L) : SensorManager(L), L(L), M(L->M)
			, gaitCommand(this, "ControlLayer/gaitCommand")
			, headControlTarget(this, "ControlLayer/headControlTarget")
			, playMotion(this, "ControlLayer/playMotion")
			, m_rsc_playMotion(2.0, 0.3)
		{}

		// Parent layer and manager
		RosInterfaceLayer* const L;
		SoccerManager* const M;

		// Layer sensors
		Sensor<gait::GaitCommand> gaitCommand;
		Sensor<head_control::LookAtTarget> headControlTarget;
		Sensor<KeyMotionEnum> playMotion;

		// Function overrides
		virtual ret_t init();
		virtual void writeExternalData();

		// Halt function
		void halt();

	private:
		// ROS topic publishers
		ros::Publisher m_pub_gaitCommand;
		ros::Publisher m_pub_headControlTarget;

		// ROS services
		BCFRosServiceCaller<motion_player::PlayMotion> m_rsc_playMotion;
	};

	/**
	* @class RosInterfaceLayerAM
	*
	* @brief Implements the actuator manager for the ROS Interface Layer
	**/
	class RosInterfaceLayerAM : public behaviourcontrol::ActuatorManager
	{
	public:
		// Constructors
		explicit RosInterfaceLayerAM(RosInterfaceLayer* L) : ActuatorManager(L), L(L), M(L->M)
			, ballPosition(this, "RosInterfaceLayer/ballPosition", NOT_AGGREGATABLE)
			, ballFocalPosition(this, "RosInterfaceLayer/ballFocalPosition", NOT_AGGREGATABLE)
			, gameControlData(this, "RosInterfaceLayer/gameControlData", NOT_AGGREGATABLE)
			, compassHeading(this, "RosInterfaceLayer/compassHeading", NOT_AGGREGATABLE)
			, visionDetections(this, "RosInterfaceLayer/visionDetections", NOT_AGGREGATABLE)
			, button(this, "RosInterfaceLayer/button", NOT_AGGREGATABLE)
			, robotState(this, "RosInterfaceLayer/robotState", NOT_AGGREGATABLE)
		{}

		// Parent layer and manager
		RosInterfaceLayer* const L;
		SoccerManager* const M;

		// Layer actuators
		Actuator<geometry_msgs::PointStampedConstPtr> ballPosition;
		Actuator<geometry_msgs::PointStampedConstPtr> ballFocalPosition;
		Actuator<rcup_game_controller::GCDataConstPtr> gameControlData;
		Actuator<robotcontrol::CompassHeadingConstPtr> compassHeading;
		Actuator<soccer_vision::DetectionsConstPtr> visionDetections;
		Actuator<robotcontrol::ButtonConstPtr> button;
		Actuator<robotcontrol::StateConstPtr> robotState;

		// Function overrides
		virtual ret_t init();
		virtual void readExternalData();

	private:
		// ROS topic event handlers
		void handleBallPosition(const geometry_msgs::PointStampedConstPtr& msg);
		void handleBallFocalPosition(const geometry_msgs::PointStampedConstPtr& msg);
		void handleGameControlData(const rcup_game_controller::GCDataConstPtr& msg);
		void handleCompassHeading(const robotcontrol::CompassHeadingConstPtr& msg);
		void handleVisionDetections(const soccer_vision::DetectionsConstPtr& msg);
		void handleButton(const robotcontrol::ButtonConstPtr& msg);
		void handleRobotState(const robotcontrol::StateConstPtr& msg);

		// ROS topic subscribers
		ros::Subscriber m_sub_ballPosition;
		ros::Subscriber m_sub_ballFocalPosition;
		ros::Subscriber m_sub_gameControlData;
		ros::Subscriber m_sub_compassHeading;
		ros::Subscriber m_sub_visionDetections;
		ros::Subscriber m_sub_button;
		ros::Subscriber m_sub_robotState;
	};
}

#endif /* ROS_INTERFACE_LAYER_H */
// EOF