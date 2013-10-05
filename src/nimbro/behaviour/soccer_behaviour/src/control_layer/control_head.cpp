// Behaviour Control Framework - Head control behaviour
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <soccer_behaviour/control_layer/control_head.h>

#include <soccer_behaviour/control_layer/control_layer.h>
#include <head_control/LookAtTarget.h>

// Namespaces
using namespace std;
using namespace soccerbehaviour;

//
// Constructors
//

// Constructor
ControlHead::ControlHead(ControlLayer* L) : Behaviour(L, "ControlHead"), L(L), M(L->M), SM(L->SM), AM(L->AM)
{
}

//
// Function overrides
//

// Initialisation function
ret_t ControlHead::init()
{
	// Return that initialisation was successful
	return RET_OK;
}

// Update function
void ControlHead::update()
{
}

// Compute activation level function
level_t ControlHead::computeActivationLevel()
{
	// Head control is active by default
	return true; // Control head is active whenever Search For Ball (inhibits this) is being inhibited by a higher behaviour
}

// Execute function
void ControlHead::execute()
{
	if(wasJustActivated())
		ROS_WARN("     CONTROL HEAD just activated!");
	
	geometry_msgs::PointStampedConstPtr focalVec = SM->ballFocalPosition.read();

	if(!focalVec)
		return; // no data yet

	head_control::LookAtTarget target;

	target.is_angular_data = true;
	target.is_relative = true;
	target.vec.z = atan2(-focalVec->point.x, focalVec->point.z);
	target.vec.y = atan2(focalVec->point.y, focalVec->point.z);

	AM->headControlTarget.write(target, this);
}

// Inhibited function
void ControlHead::inhibited()
{
	if(wasJustDeactivated())
		ROS_WARN("   CONTROL HEAD just deactivated! XXXXXXXXXXXXXXXXXXXXXXXXXX");
}

//
// Extra stuff
//

/* TODO: Define other functions that you need here */
// EOF