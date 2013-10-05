// Soccer Behaviour - Go Behind Ball behaviour
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file go_behind_ball.h
* @brief Defines the ball approach behaviour for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef GO_BEHIND_BALL_H
#define GO_BEHIND_BALL_H

// Includes
#include <soccer_behaviour/soccer_common.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	/**
	 * @class GoBehindBall
	 *
	 * @brief Implements the Go Behind Ball behaviour
	 **/
	class GoBehindBall : public behaviourcontrol::Behaviour
	{
	public:
		// Constructors
		explicit GoBehindBall(ControlLayer* L);

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

		double adjustCmdForObstacle(gait::GaitCommand& cmd, Eigen::Vector2d targetVec, Eigen::Vector2d obstacleVec, bool calcOnly);
	};
}

#endif /* GO_BEHIND_BALL_H */
// EOF
