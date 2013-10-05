// Behaviour Control Framework - Search For Ball behaviour
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <soccer_behaviour/control_layer/search_for_ball.h>
#include <soccer_behaviour/control_layer/control_layer.h>

// Namespaces
using namespace std;
using namespace soccerbehaviour;

//
// Constructors
//

// Constructor
SearchForBall::SearchForBall(ControlLayer* L) : Behaviour(L, "SearchForBall"), L(L), M(L->M), SM(L->SM), AM(L->AM)
{
	m_delay = ros::Duration(2.0);
}

//
// Function overrides
//

// Initialisation function
ret_t SearchForBall::init()
{
	// Return that initialisation was successful
	return RET_OK;
}

// Update function
void SearchForBall::update()
{
}

// Compute activation level function
level_t SearchForBall::computeActivationLevel()
{
// 	geometry_msgs::PointStampedConstPtr ballInfo = SM->ballPosition.read();

//	return !ballInfo || (ros::Time::now() - ballInfo->header.stamp) > m_delay;
	return true; // Search ball is active whenever it is not being inhibited by a higher behaviour
}

// Execute function
void SearchForBall::execute()
{
	if(wasJustActivated())
	{
		ROS_WARN("  SEARCH FOR BALL just activated! (and saved its start time)");
		m_startTime = ros::Time::now();
	}

	// Determine head control target
	head_control::LookAtTarget target;

	double t = (ros::Time::now() - m_startTime).toSec();

	target.is_angular_data = true;
	target.is_relative = false;
	target.vec.x = 0;
	target.vec.y = 0;
	target.vec.z = M_PI/2.0 * sin(2.0 * M_PI * t * 0.1);

	AM->headControlTarget.write(target, this);

	// Determine gait target
	gait::GaitCommand cmd;

	cmd.walk = true;
	cmd.gcvX = 0.0;
	cmd.gcvY = 0;

	if(t < 7.5)
		cmd.gcvZ = 0.0;
	else
		cmd.gcvZ = -0.4;

	AM->gaitCommand.write(cmd, this);
}

// Inhibited function
void SearchForBall::inhibited()
{
	if(wasJustDeactivated())
		ROS_WARN("SEARCH FOR BALL just deactivated! XXXXXXXXXXXXXXXXXXXXXXXXXX");
}

//
// Extra stuff
//

/* TODO: Define other functions that you need here */
// EOF