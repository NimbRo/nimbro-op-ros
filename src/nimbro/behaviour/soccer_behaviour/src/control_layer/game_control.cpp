// Behaviour Control Framework - Game control listens to the referee box
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <soccer_behaviour/control_layer/game_control.h>

#include <soccer_behaviour/control_layer/control_layer.h>
#include <soccer_behaviour/soccer_manager/soccer_manager.h>

#include <boost/foreach.hpp>

// Namespaces
using namespace std;
using namespace soccerbehaviour;

//
// Constructors
//

// Constructor
GameControl::GameControl(ControlLayer* L) : Behaviour(L, "GameControl"), L(L), M(L->M), SM(L->SM), AM(L->AM)
{
	m_state = rcup_game_controller::GCData::STATE_INITIAL;
}

//
// Function overrides
//

// Initialisation function
ret_t GameControl::init()
{
	// Return that initialisation was successful
	return RET_OK;
}

// Update function
void GameControl::update()
{
	rcup_game_controller::GCDataConstPtr gc = SM->gameControlData.read();

	if(!gc)
		return;

	bool active = false;
	if(gc->state == rcup_game_controller::GCData::STATE_PLAYING)
	{
		BOOST_FOREACH(const rcup_game_controller::GCTeamInfo& team, gc->teams)
		{
			if(team.teamNumber != L->teamNumber())
				continue;

			for(int i = 0; i < (int) team.player.size(); ++i)
			{
				const rcup_game_controller::GCRobotInfo& robot = team.player[i];

				if(i != L->playerNumber()-1)
					continue;

				ROS_WARN_THROTTLE(0.2, "My penalty state is %d", robot.penalty);
				active = (robot.penalty == rcup_game_controller::GCRobotInfo::NO_PENALTY);
			}
		}
	}

	if(M->state() == SoccerManager::STATE_WAIT && active)
		M->setState(SoccerManager::STATE_RUN);

	if(M->state() == SoccerManager::STATE_RUN && !active)
		M->setState(SoccerManager::STATE_WAIT);
}

// Compute activation level function
level_t GameControl::computeActivationLevel()
{
	return M->state() == SoccerManager::STATE_WAIT;
}

// Execute function
void GameControl::execute()
{
	gait::GaitCommand cmd;
	cmd.walk = false;
	AM->gaitCommand.write(cmd, this);

	head_control::LookAtTarget head;
	head.is_angular_data = true;
	head.is_relative = false;
	head.vec.x = 0;
	head.vec.y = 0;
	head.vec.z = 0;
	AM->headControlTarget.write(head, this);
}

//
// Extra stuff
//

/* TODO: Define other functions that you need here */
// EOF