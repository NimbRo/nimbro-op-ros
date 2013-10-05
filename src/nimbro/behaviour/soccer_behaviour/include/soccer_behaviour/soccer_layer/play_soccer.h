// Soccer Behaviour - Play Soccer behaviour
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file play_soccer.h
* @brief Defines the Play Soccer behaviour for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef PLAY_SOCCER_H
#define PLAY_SOCCER_H

// Includes
#include <soccer_behaviour/soccer_common.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	/**
	* @class PlaySoccer
	*
	* @brief Implements the Play Soccer behaviour
	**/
	class PlaySoccer : public behaviourcontrol::Behaviour
	{
	public:
		// Constructors
		explicit PlaySoccer(SoccerLayer* L);

		// Parent layer and manager
		SoccerLayer* const L;
		SoccerManager* const M;

		// Sensor and actuator manager pointers
		SoccerLayerSM* const SM;
		SoccerLayerAM* const AM;

		// Function overrides
		virtual ret_t init();
		virtual void update();
		virtual level_t computeActivationLevel();
		virtual void execute();

		//
		// Extra stuff
		//

		/* TODO: Add everything else you need here */
	};
}

#endif /* PLAY_SOCCER_H */
// EOF