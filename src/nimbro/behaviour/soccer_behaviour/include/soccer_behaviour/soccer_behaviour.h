// Soccer Behaviour - Main header file
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file soccer_behaviour.h
* @brief Implements the Soccer Behaviour package.
**/

/**
* @defgroup SoccerBehaviour Soccer Behaviour
*
* @author
* Philipp Allgeuer (<pallgeuer@ais.uni-bonn.de>),
* Max Schwarz (<max.schwarz@uni-bonn.de>),
* Sebastian Schueller (<schuell1@cs.uni-bonn.de>)
*
* @section sec1 Overview
* The Soccer Behaviour package makes use of the @ref BehaviourControlFramework "Behaviour Control Framework"
* to implement a behaviour controller that makes the robot play soccer. The basic behaviours include the
* @ref soccerbehaviour::ControlHead "Control Head", @ref soccerbehaviour::SearchForBall "Search For Ball",
* @ref soccerbehaviour::GoBehindBall "Go Behind Ball", @ref soccerbehaviour::Dribble "Dribble",
* @ref soccerbehaviour::Kick "Kick" and @ref soccerbehaviour::GameControl "Game Control" behaviours.
* Dynamic inhibitions between these basic behaviours are used to make the robot choose the appropriate
* behaviour(s) for each situation. Refer to each of these individual behaviours as well as the description
* of the @ref BehaviourControlFramework "Behaviour Control Framework" for more information.
*
* @section sec2 Dependencies
* The Soccer Behaviour package has the same external @ref bcfsec2 "dependencies" as the
* Behaviour Control Framework, in addition to its dependencies
* on standard ROS (including various standard message types) and the
* <a href="http://eigen.tuxfamily.org/"><b>Eigen Library</b></a>.
*
* @see @link soccerbehaviour soccerbehaviour Namespace @endlink
* @see @link BehaviourControlFramework Behaviour Control Framework @endlink
* @see @link StateControllerLibrary State Controller Library @endlink
* @see @link soccerbehaviour::SoccerManager SoccerManager Class @endlink
* @see @link soccerbehaviour::RosInterfaceLayer ROS Interface Layer @endlink
* @see @link soccerbehaviour::ControlLayer Control Layer @endlink
* @see @link soccerbehaviour::SoccerLayer Soccer Layer @endlink
**/

// Ensure header is only included once
#ifndef SOCCER_BEHAVIOUR_H
#define SOCCER_BEHAVIOUR_H

/**
* @namespace soccerbehaviour
*
* @ingroup SoccerBehaviour
*
* @brief This namespace defines everything that is required for the @ref SoccerBehaviour "Soccer Behaviour" package.
**/
namespace soccerbehaviour {}

// Include common definitions
#include <soccer_behaviour/soccer_common.h>

// Include the soccer behaviour layers
#include <soccer_behaviour/ros_interface_layer/ros_interface_layer.h>
#include <soccer_behaviour/soccer_layer/soccer_layer.h>
#include <soccer_behaviour/control_layer/control_layer.h>

// Include the soccer behaviour manager
#include <soccer_behaviour/soccer_manager/soccer_manager.h>

#endif /* SOCCER_BEHAVIOUR_H */
// EOF