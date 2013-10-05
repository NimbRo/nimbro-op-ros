// Soccer Behaviour - Common definitions
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file soccer_common.h
* @brief Common definitions include file for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef SOCCER_COMMON_H
#define SOCCER_COMMON_H

// Includes - ROS
#include <ros/ros.h>
#include <ros/console.h>

// Includes - NimbRo packages
#include <behaviour_control/behaviour_control.h>
//#include <state_controller/state_controller.h>
#include <test_utilities/test_utilities.h>
//#include <keyframe_player_hack/PlayMotion.h> // TODO: Remove once fully migrated to new motion player

// Includes - C++ Standard Library
#include <iostream>
#include <string>
#include <math.h>

// Includes - Other
#include <Eigen/Core>
#include <soccer_behaviour/soccer_utilities.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	//
	// Namespaces
	//
	using namespace behaviourcontrol;
	//using namespace statecontroller;
	using namespace testutilities;

	//
	// Class declarations
	//

	// Soccer manager
	class SoccerManager;

	// ROS interface layer
	class RosInterfaceLayer;
	class RosInterfaceLayerSM;
	class RosInterfaceLayerAM;

	// Soccer layer
	class SoccerLayer;
	class SoccerLayerSM;
	class SoccerLayerAM;
	class PlaySoccer;

	// Control layer
	class ControlLayer;
	class ControlLayerSM;
	class ControlLayerAM;
	class GameControl;
	class Kick;
	class Dribble;
	class GoBehindBall;
	class SearchForBall;
	class ControlHead;

	//
	// Enumerations
	//

	//! Keyframe player motion enumeration
	enum KeyMotionEnum
	{
		KM_NO_MOTION = 0,
		KM_STAND_UP,
		KM_RIGHT_KICK,
		KM_LEFT_KICK,
		KM_SIT,
		KM_PRONE_GETUP,
		KM_SUPINE_GETUP,
		KM_SCOOP_KICK,
		KM_NEW_RIGHT_KICK,
		KM_NEW_LEFT_KICK,
		KM_NUM_MOTIONS //!< @brief Used to automatically retrieve the number of valid motions
	};

	//
	// Type definitions
	//

	/* None yet */

	//
	// Constants
	//

	//! Keyframe player motion names (must match up with KeyMotionEnum!)
	const std::string KeyMotionName[KM_NUM_MOTIONS + 1] = {
		"NO_MOTION",
		"STAND_UP",
		"RIGHT_KICK",
		"LEFT_KICK",
		"SIT",
		"PRONE_GETUP",
		"SUPINE_GETUP",
		"scoopkick",
		"right_kick_straight",
		"left_kick_straight",
		"NUM_MOTIONS"
	};

	//
	// Functions
	//

	/* None yet */
}

#endif /* SOCCER_COMMON_H */
// EOF
