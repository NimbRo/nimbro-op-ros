// Soccer Behaviour - Control Head behaviour
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file control_head.h
* @brief Defines the control head behaviour for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef CONTROL_HEAD_H
#define CONTROL_HEAD_H

// Includes
#include <soccer_behaviour/soccer_common.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	/**
	 * @class ControlHead
	 *
	 * @brief Implements the control head behaviour
	 **/
	class ControlHead : public behaviourcontrol::Behaviour
	{
	public:
		// Constructors
		explicit ControlHead(ControlLayer* L);

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
	};
}

#endif /* CONTROL_HEAD_H */
// EOF
