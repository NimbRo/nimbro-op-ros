// Soccer Behaviour - Search For Ball behaviour
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file search_for_ball.h
* @brief Defines the Search For Ball behaviour for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef SEARCH_FOR_BALL_H
#define SEARCH_FOR_BALL_H

// Includes
#include <soccer_behaviour/soccer_common.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	/**
	* @class SearchForBall
	*
	* @brief Implements the Search For Ball behaviour
	**/
	class SearchForBall : public behaviourcontrol::Behaviour
	{
	public:
		// Constructors
		explicit SearchForBall(ControlLayer* L);

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

	private:
		ros::Duration m_delay;
		ros::Time m_startTime;
	};
}

#endif /* SEARCH_FOR_BALL_H */
// EOF