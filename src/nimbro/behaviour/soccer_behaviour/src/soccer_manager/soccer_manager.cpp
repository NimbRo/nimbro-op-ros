// Behaviour Control Framework - Soccer Manager
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <soccer_behaviour/soccer_manager/soccer_manager.h>
#include <soccer_behaviour/ros_interface_layer/ros_interface_layer.h>
#include <soccer_behaviour/soccer_layer/soccer_layer.h>
#include <soccer_behaviour/control_layer/control_layer.h>

// Namespaces
using namespace std;
using namespace soccerbehaviour;

//
// Constructors
//

// Constructor
SoccerManager::SoccerManager() : BehaviourManager("SoccerManager")
{
	// Create child layers
	RosIL = new RosInterfaceLayer(this);
	SL = new SoccerLayer(this);
	CL = new ControlLayer(this);

	ros::NodeHandle nh("~");
	m_ledTimer = nh.createTimer(ros::Duration(0.2), boost::bind(&SoccerManager::handleLEDTimer, this));
	m_pub_leds = nh.advertise<robotcontrol::LEDCommand>("/led", 1);

	led.mask = 255;

	setState(STATE_HALT);
}

// Destructor
SoccerManager::~SoccerManager()
{
	halt();

	// Delete child layers
	delete RosIL;
	delete SL;
	delete CL;
}

//
// Function overrides
//

// Initialisation function
ret_t SoccerManager::init()
{
	// Return that initialisation was successful
	return RET_OK;
}

// Pre-step callback
void SoccerManager::preStepCallback()
{
}

// Post-step callback
void SoccerManager::postStepCallback()
{
}

void SoccerManager::halt()
{
	RosIL->halt();
	CL->halt();
}

bool SoccerManager::getEnabled() const
{
	return m_state != STATE_HALT;
}

void SoccerManager::handleLEDTimer()
{
	m_pub_leds.publish(led);
}

void SoccerManager::setState(SoccerManager::State state)
{
	m_state = state;
	switch(m_state)
	{
		case STATE_HALT:
			led.rgb6.r = 1.0;
			led.rgb6.g = 0.0;
			led.rgb6.b = 0.0;
			break;
		case STATE_WAIT:
			led.rgb6.r = 1.0;
			led.rgb6.g = 0.0;
			led.rgb6.b = 1.0;
			break;
		case STATE_RUN:
			led.rgb6.r = 0.0;
			led.rgb6.g = 1.0;
			led.rgb6.b = 0.0;
			break;
		case STATE_RUN_MANUAL:
			led.rgb6.r = 0.0;
			led.rgb6.g = 1.0;
			led.rgb6.b = 1.0;
			break;
	}
}

//
// Error notification functions
//

// Report error to user
void SoccerManager::reportErrorUser(const std::string& msg, bool fatal, const std::string& funcName, const std::string& fileName, int line)
{
	// Declare variables
	ostringstream out;

	// Convert the error details into a formatted string
	out << "An error occurred in the behaviour architecture!" << endl;
	out << setColour(Attr::RED) << (fatal ? "Fatal error: " : "Non-fatal error: ") << msg << setColour() << endl;
	out << "In function " << setColour(Attr::MAGENTA) << funcName << "()" << setColour();
	out << " at " << setColour(Attr::MAGENTA) << "line " << line << setColour();
	out << " in " << setColour(Attr::MAGENTA) << fileName << setColour() << endl;

	// Display a message to the user regarding the error
	if(fatal)
	{
		ROS_FATAL_STREAM_NAMED("behaviour", out.str());
		ros::shutdown();
	}
	else ROS_ERROR_STREAM_NAMED("behaviour", out.str());
}
// EOF