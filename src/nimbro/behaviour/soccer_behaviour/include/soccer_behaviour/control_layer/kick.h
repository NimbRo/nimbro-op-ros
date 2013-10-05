// Soccer Behaviour - Kick behaviour
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file kick.h
* @brief Defines the kick behaviour for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef KICK_H
#define KICK_H

// Includes
#include <soccer_behaviour/soccer_common.h>
#include <soccer_behaviour/control_layer/control_layer.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	/**
	 * @class Kick
	 *
	 * @brief Implements the kick behaviour
	 **/
	class Kick : public behaviourcontrol::Behaviour
	{
	public:
		// Constructors
		explicit Kick(ControlLayer* L);

		// Parent layer and manager
		ControlLayer* const L;
		SoccerManager* const M;

		// Sensor and actuator manager pointers
		ControlLayerSM* const SM;
		ControlLayerAM* const AM;

		// Function overrides
		virtual ret_t init();
		virtual void update();
		virtual level_t computeActivationLevel();
		virtual void execute();
		virtual void inhibited();

		//
		// Extra stuff
		//

		// Internal variables
		bool m_locked_on;
		geometry_msgs::PointStampedConstPtr bI;
		RosTimeMarker lockedOnTime;
	};
}

#endif /* KICK_H */
// EOF
