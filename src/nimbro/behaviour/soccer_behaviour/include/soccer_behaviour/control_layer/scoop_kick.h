// Soccer Behaviour - Scoop kick behaviour
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file scoop_kick.h
* @brief Defines the scoop kick behaviour for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef SCOOP_KICK_H
#define SCOOP_KICK_H

// Includes
#include <soccer_behaviour/soccer_common.h>
#include <soccer_behaviour/control_layer/control_layer.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	/**
	 * @class ScoopKick
	 *
	 * @brief Implements the scoop kick behaviour
	 **/
	class ScoopKick : public behaviourcontrol::Behaviour
	{
	public:
		// Constructors
		explicit ScoopKick(ControlLayer* L);

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

#endif /* SCOOP_KICK_H */
// EOF
