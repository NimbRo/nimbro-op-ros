// Behaviour Control Framework - Scoop kick behaviour
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <soccer_behaviour/control_layer/scoop_kick.h>
#include <soccer_behaviour/control_layer/control_layer.h>

// Namespaces
using namespace std;
using namespace soccerbehaviour;

//
// Constructors
//

// Constructor
ScoopKick::ScoopKick(ControlLayer* L) : Behaviour(L, "ScoopKick"), L(L), M(L->M), SM(L->SM), AM(L->AM)
	, m_locked_on(false)
{
}

//
// Function overrides
//

// Initialisation function
ret_t ScoopKick::init()
{
	// Return that initialisation was successful
	return RET_OK;
}

// Update function
void ScoopKick::update()
{
}

// Compute activation level function
level_t ScoopKick::computeActivationLevel()
{
	// Get ball location (don't activate if we can't see it)
	bI = SM->ballPosition.read();
	if(!bI)
		return false; // TODO: Does this have bad effect with not resetting locked_on? YES (FIX! Has been observed) (timeout to reset m_locked_on?)

	// Retrieve dribble target
	bool isGoal = true;
	Eigen::Vector2d target = L->dribbleTarget(isGoal);

	// Get goal target angles
	const double goalTargetAngle = atan2(target.y(), target.x());
	const double goalTargetAngleRange = L->maxGoalPostAngle() - L->minGoalPostAngle();
	const double goalTargetAngleLBnd = L->minGoalPostAngle() + 0.1 * goalTargetAngleRange;
	const double goalTargetAngleUBnd = L->maxGoalPostAngle() - 0.1 * goalTargetAngleRange;

	// Lock-on constants
	const double ballTolY = 0.05;
	const double ballTolYWeak = 0.14;
	const double minBallX = 0.05;
	const double maxBallXKick = 0.5;
	const double maxBallXKickWeak = 0.6;
	const double targetTolYNonGoal = 1.8;
	const double targetTolAngle = 0.7;
	const double targetMaxNorm = 0.85; // TODO: Tune this value (dictates when to attempt the scoop kick)

	// Check conditions
	bool xCondition = (bI->point.x >= minBallX) && (bI->point.x <= maxBallXKick);
	bool xConditionWeak = (bI->point.x >= minBallX) && (bI->point.x <= maxBallXKickWeak);
	bool yCondition = (fabs(bI->point.y - (-L->targetBallOffsetY)) <= ballTolY); // Kick modified
	bool yConditionWeak = (fabs(bI->point.y - (-L->targetBallOffsetY)) <= ballTolYWeak); // Kick modified
	bool targetXCondition = (target.x() > bI->point.x);
	bool lineUpGoalCondition = goalTargetAngleLBnd < 0 && goalTargetAngleUBnd > 0;
	bool lineUpNonGoalCondition = (fabs(target.y()) <= 0.5 * targetTolYNonGoal) && (fabs(goalTargetAngle) <= 0.5 * targetTolAngle);
	bool lineUpCondition = (isGoal && lineUpGoalCondition) || (!isGoal && lineUpNonGoalCondition);
	bool nearCondition = !isGoal || (isGoal && (target.norm() <= targetMaxNorm));

// 	ROS_INFO_STREAM_THROTTLE(0.1, "SCOOP KICK: X: " << xCondition << " | Y: " << yCondition << " | targetX: " << targetXCondition << " | lineUpNonGoal: " << lineUpNonGoalCondition << " | lineUpGoal: " << lineUpGoalCondition << " |" );

// 	if (lineUpGoalCondition)
// 		ROS_INFO_STREAM("SCOOP KICK: min: " << L->minGoalPostAngle() << "max: " << L->maxGoalPostAngle() << "goal: " << goalTargetAngle);
	// Work out whether to activate kick lock
	if(!m_locked_on && xCondition && yCondition && targetXCondition && lineUpCondition && nearCondition)
	{
// 		ROS_INFO_STREAM("SCOOP KICK: X: " << xCondition << " | Y: " << yCondition << " | targetX: " << targetXCondition << " | lineUpNonGoal: " << lineUpNonGoalCondition << " | lineUpGoal: " << lineUpGoalCondition << " |" );

// 		ROS_INFO_STREAM("Scoop Kick lock toggled ON!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl << endl << endl << endl << endl);

		lockedOnTime.setMarker();
		m_locked_on = true;
		return true;
	}

	// Work out whether to deactivate scoop kick lock
	if(m_locked_on && lockedOnTime.hasElapsed(3.0))
	{
		// Final check: Is the ball still where we want it?
		if(xConditionWeak && yConditionWeak && targetXCondition && lineUpCondition)
		{
			AM->playMotion.write(KM_SCOOP_KICK, this);
		}
		else
			m_locked_on = false;
		return m_locked_on;
	}

	// Return the current lock state
	return m_locked_on;
}

// Execute function
void ScoopKick::execute()
{
	if(wasJustActivated())
		ROS_WARN("     SCOOP KICK just activated!");

	gait::GaitCommand cmd;
	cmd.gcvX = -0.7;
	cmd.gcvY = 0.0;
	cmd.gcvZ = 0.0;
	cmd.walk = false;
	AM->gaitCommand.write(cmd, this);
}

// Inhibited function
void ScoopKick::inhibited()
{
	if(wasJustDeactivated())
		ROS_WARN("   SCOOP KICK just deactivated! XXXXXXXXXXXXXXXXXXXXXXXXXX");
}
// EOF