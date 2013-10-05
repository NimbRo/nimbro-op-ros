// Soccer Behaviour - Game control listens to the referee box
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file game_control.h
* @brief Defines the game control behaviour for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef GAME_CONTROL_H
#define GAME_CONTROL_H

// Includes
#include <soccer_behaviour/soccer_common.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	/**
	 * @class GameControl
	 *
	 * @brief Implements the game control behaviour
	 **/
	class GameControl : public behaviourcontrol::Behaviour
	{
	public:
		// Constructors
		explicit GameControl(ControlLayer* L);

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
	private:
		int m_state;
	};
}

#endif /* GAME_CONTROL_H */
// EOF
