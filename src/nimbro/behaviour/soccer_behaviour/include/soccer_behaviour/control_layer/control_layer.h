// Soccer Behaviour - Control Layer
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file control_layer.h
* @brief Defines the Control Layer for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef CONTROL_LAYER_H
#define CONTROL_LAYER_H

// Includes - Message types (for reading RosInterfaceLayer's actuators)
#include <geometry_msgs/PointStamped.h>
#include <rcup_game_controller/GCData.h>
#include <robotcontrol/CompassHeading.h>
#include <soccer_vision/Detections.h>
#include <robotcontrol/Button.h>
#include <robotcontrol/State.h>
#include <head_control/LookAtTarget.h>
#include <gait/GaitCommand.h>

#include <Eigen/Core>
#include <config_server/parameter.h>

// Includes - Common
#include <soccer_behaviour/soccer_common.h>

// Includes - Child behaviours
#include <soccer_behaviour/control_layer/kick.h>
#include <soccer_behaviour/control_layer/dribble.h>
#include <soccer_behaviour/control_layer/go_behind_ball.h>
#include <soccer_behaviour/control_layer/search_for_ball.h>
#include <soccer_behaviour/control_layer/control_head.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	/**
	* @class ControlLayer
	*
	* @brief Implements the soccer Control Layer
	**/
	class ControlLayer : public behaviourcontrol::BehaviourLayer
	{
	public:
		// Constructors
		explicit ControlLayer(SoccerManager* M);
		virtual ~ControlLayer();

		// Parent manager
		SoccerManager* const M;

		// Sensor and actuator managers
		ControlLayerSM* SM;
		ControlLayerAM* AM;

		// Child behaviours
		GameControl* gameControl;
		Kick* kick;
		Dribble* dribble;
		GoBehindBall* goBehindBall;
		SearchForBall* searchForBall;
		ControlHead* controlHead;

		Eigen::Vector2d goBehindBallTarget();
		Eigen::Vector2d dribbleTarget(bool& isGoal);
		Eigen::Vector2d kickTarget(bool& isGoal);

		// Function overrides
		virtual ret_t init();
		virtual void update();
		virtual void postExecuteCallback();

		void halt();
		double compassGoalTarget(bool& haveCompass);
		double minGoalPostAngle();
		double maxGoalPostAngle();

		inline double goalSign() const
		{ return m_param_positive() ? 1.0 : -1.0; }

		bool goalIsValid();


		Eigen::Vector2d m_lastGoalPos;
		ros::Time m_lastGoalPosStamp;

		// Tuneable constants
		static const double targetBallOffsetX;
		static const double targetBallOffsetY;
		static const double maxBallXWeak;

		inline int teamNumber() const
		{ return m_param_teamNumber(); }

		inline int playerNumber() const
		{ return m_param_robotNumber(); }
	private:
		void findGoalInVision();
		void updateGCInfo();

		//
		// Extra stuff
		//

		/* TODO: Add everything else you need here */
		double m_minGoalPostAngle;    //a.k.a. right Goal Post
		double m_maxGoalPostAngle;    //a.k.a. left Goal Post

		config_server::Parameter<int> m_param_teamNumber;
		config_server::Parameter<int> m_param_robotNumber;
		config_server::Parameter<bool> m_param_positiveIsYellow;
		config_server::Parameter<bool> m_param_positive;
	};

	/**
	* @class ControlLayerSM
	*
	* @brief Implements the sensor manager for the Control Layer
	**/
	class ControlLayerSM : public behaviourcontrol::SensorManager
	{
	public:
		// Constructors
		explicit ControlLayerSM(ControlLayer* L) : SensorManager(L), L(L), M(L->M)
			, ballFocalPosition(this, "RosInterfaceLayer/ballFocalPosition")
			, ballPosition(this, "RosInterfaceLayer/ballPosition")
			, visionDetections(this, "RosInterfaceLayer/visionDetections")
			, gameControlData(this, "RosInterfaceLayer/gameControlData")
			, compassHeading(this, "RosInterfaceLayer/compassHeading")
		{}

		// Parent layer and manager
		ControlLayer* const L;
		SoccerManager* const M;

		// Layer sensors
		Sensor<geometry_msgs::PointStampedConstPtr> ballFocalPosition;
		Sensor<geometry_msgs::PointStampedConstPtr> ballPosition;
		Sensor<soccer_vision::DetectionsConstPtr> visionDetections;
		Sensor<rcup_game_controller::GCDataConstPtr> gameControlData;
		Sensor<robotcontrol::CompassHeadingConstPtr> compassHeading;

		// Function overrides
		virtual ret_t init() { return RET_OK; }
	};

	/**
	* @class ControlLayerAM
	*
	* @brief Implements the actuator manager for the Control Layer
	**/
	class ControlLayerAM : public behaviourcontrol::ActuatorManager
	{
	public:
		// Constructors
		explicit ControlLayerAM(ControlLayer* L) : ActuatorManager(L), L(L), M(L->M)
			, headControlTarget(this, "ControlLayer/headControlTarget", NOT_AGGREGATABLE)
			, gaitCommand(this, "ControlLayer/gaitCommand", NOT_AGGREGATABLE)
			, playMotion(this, "ControlLayer/playMotion", NOT_AGGREGATABLE)
		{}

		// Parent layer and manager
		ControlLayer* const L;
		SoccerManager* const M;

		// Layer actuators
		Actuator<head_control::LookAtTarget> headControlTarget;
		Actuator<gait::GaitCommand> gaitCommand;
		Actuator<KeyMotionEnum> playMotion;

		// Function overrides
		virtual ret_t init() { return RET_OK; }
	};
}

#endif /* CONTROL_LAYER_H */
// EOF