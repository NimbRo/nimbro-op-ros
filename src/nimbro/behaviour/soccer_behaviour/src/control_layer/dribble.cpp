// Behaviour Control Framework - Dribble behaviour
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <soccer_behaviour/control_layer/dribble.h>

// Namespaces
using namespace std;
using namespace soccerbehaviour;

//
// Constructors
//

// Constructor
Dribble::Dribble(ControlLayer* L) : Behaviour(L, "Dribble"), L(L), M(L->M), SM(L->SM), AM(L->AM)
	, m_locked_on(false)
{
}

//
// Function overrides
//

// Initialisation function
ret_t Dribble::init()
{
	// Return that initialisation was successful
	return RET_OK;
}

// Update function
void Dribble::update()
{
}

// Compute activation level function
level_t Dribble::computeActivationLevel()
{
	// Get ball location (don't activate if we can't see it)
	bI = SM->ballPosition.read();
	if(!bI)
		return false; // TODO: Does this have bad effect with not resetting locked_on? (timeout to reset m_locked_on?)

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
	const double minBallX = 0.05;
	const double maxBallX = 0.65;
	const double maxBallYExtra = 0.05;
	const double maxBallYWeak = L->targetBallOffsetY + ballTolY + maxBallYExtra;
	const double targetTolYNonGoal = 1.8;
	const double targetTolAngle = 0.7;

	// Check conditions
	bool xCondition =     (bI->point.x >= minBallX) && (bI->point.x <= maxBallX);
	bool xConditionWeak = (bI->point.x >= minBallX) && (bI->point.x <= L->maxBallXWeak);
	bool yCondition = (fabs(fabs(bI->point.y) - L->targetBallOffsetY) <= ballTolY);
	bool yConditionWeak = (fabs(bI->point.y) <= maxBallYWeak);
	bool targetXCondition = (target.x() > bI->point.x);
	bool lineUpGoalCondition = goalTargetAngleLBnd < 0 && goalTargetAngleUBnd > 0;
	bool lineUpNonGoalCondition = (fabs(target.y()) <= 0.5 * targetTolYNonGoal) && (fabs(goalTargetAngle) <= 0.5 * targetTolAngle);
	bool lineUpCondition = (isGoal && lineUpGoalCondition) || (!isGoal && lineUpNonGoalCondition);
	bool timeCondition = lastToggle.hasElapsed(0.3);

	// TODO: DEBUG - Print our boolean conditions
// 	ROS_INFO_STREAM_THROTTLE(0.1, "X: " << xCondition << " | Y: " << yCondition << " | targetX: " << targetXCondition << " | lineUpNonGoal: " << lineUpNonGoalCondition << " | lineUpGoal: " << lineUpGoalCondition << " | time: " << timeCondition << " |");


	// Lock on if in narrow area plus other checks
	if(!m_locked_on && xCondition && yCondition && targetXCondition && lineUpCondition && timeCondition)
	{
// 		ROS_INFO_STREAM("DRIBBLE: X: " << xCondition << " | Y: " << yCondition << " | targetX: " << targetXCondition << " | lineUpNonGoal: " << lineUpNonGoalCondition << " | lineUpGoal: " << lineUpGoalCondition << " | time: " << timeCondition << " |");

// 		ROS_INFO_STREAM("Dribble lock toggled ON!" << endl << endl << endl << endl << endl);
		lastToggle.setMarker();
		m_locked_on = true;
		return true;
	}

	// Lock off if not in wider area (plus other checks) anymore
	if(m_locked_on && !(xConditionWeak && yConditionWeak && targetXCondition && lineUpCondition) && timeCondition)
	{
// 		ROS_INFO_STREAM("Dribble lock toggled OFF!" << endl << endl << endl << endl << endl);
		lastToggle.setMarker();
		m_locked_on = false;
		return false;
	}

	// Return current behaviour lock state if no toggle is necessary
	return m_locked_on;
}

// Execute function
void Dribble::execute()
{
	if(wasJustActivated())
		ROS_WARN("        DRIBBLE just activated!");
	
	// Constants
	const double maxGaitCommandY = 0.65;

	// Declare variables
	gait::GaitCommand cmd;

	// Set base gait command
	cmd.gcvX = 0.5;
	cmd.gcvY = 0;
	cmd.gcvZ = 0;
	cmd.walk = true;
	
	// Retrieve ball position if we can
	if((bool) bI)
	{
		if(fabs(bI->point.y) > L->targetBallOffsetY)
		{
			double deltaY = 0.0;
			if(bI->point.y > 0.0)
				deltaY = bI->point.y - L->targetBallOffsetY;
			else
				deltaY = bI->point.y + L->targetBallOffsetY;
			if(bI->point.x - L->targetBallOffsetX > 0)
			{
				cmd.gcvY = (cmd.gcvX - (-0.7)) * deltaY / (bI->point.x - L->targetBallOffsetX);
				if(cmd.gcvY >  maxGaitCommandY) cmd.gcvY =  maxGaitCommandY;
				if(cmd.gcvY < -maxGaitCommandY) cmd.gcvY = -maxGaitCommandY;
			}
		}
	}

	// Publish the required gait command
	AM->gaitCommand.write(cmd, this);
}

// Inhibited function
void Dribble::inhibited()
{
	if(wasJustDeactivated())
		ROS_WARN("      DRIBBLE just deactivated! XXXXXXXXXXXXXXXXXXXXXXXXXX");
}

//
// Extra stuff
//

/* TODO: Define other functions that you need here */
// EOF