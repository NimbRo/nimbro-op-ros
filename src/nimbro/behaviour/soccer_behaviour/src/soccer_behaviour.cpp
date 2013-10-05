// Soccer Behaviour - Main source file
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <soccer_behaviour/soccer_behaviour.h>

// Defines
#define BEH_FREQ 15

// Namespaces
using namespace std;
using namespace soccerbehaviour;

//
// Main function
//
int main(int argc, char **argv)
{
	// Process ROS command line arguments
	ros::init(argc, argv, "soccer_behaviour");

	// Create an instance of the SoccerManager class and initialise it
	SoccerManager SMan;
	SMan.initialiseArchitecture();

	// Display the architecture to the user
	ROS_INFO_STREAM_NAMED("behaviour", SMan.toString() << endl);

	// Initialise timer object for loop rate timing
	ros::Rate rate(BEH_FREQ);

	// Keep looping while everything is fine and dandy...
	while(ros::ok())
	{
		// Do all ROS callbacks before sleeping so that the next step starts immediately after the timer tick
		ros::spinOnce();

		// If the manager thinks it should be enabled, perform a cycle step
		if(SMan.getEnabled())
			SMan.step();
		else
			SMan.halt();

		// Sleep for the required duration
		rate.sleep();
	}

	// Return value
	return 0;
}
// EOF