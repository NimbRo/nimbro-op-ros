// Soccer Behaviour - Dribble behaviour
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file dribble.h
* @brief Defines the dribble behaviour for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef DRIBBLE_H
#define DRIBBLE_H

// Includes
#include <soccer_behaviour/soccer_common.h>
#include <soccer_behaviour/control_layer/control_layer.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	/**
	 * @class Dribble
	 *
	 * @brief Implements the dribble behaviour
	 **/
	class Dribble : public behaviourcontrol::Behaviour
	{
	public:
		// Constructors
		explicit Dribble(ControlLayer* L);

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

		/* TODO: Add everything else you need here */
		bool m_locked_on;
		geometry_msgs::PointStampedConstPtr bI;
		RosTimeMarker lastToggle;
	};
}

#endif /* DRIBBLE_H */
// EOF
