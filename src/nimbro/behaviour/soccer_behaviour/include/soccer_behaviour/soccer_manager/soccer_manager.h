// Soccer Behaviour - Soccer manager
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file soccer_manager.h
* @brief Defines the Soccer Manager for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef SOCCER_MANAGER_H
#define SOCCER_MANAGER_H

// Includes
#include <soccer_behaviour/soccer_common.h>

#include <robotcontrol/LEDCommand.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	/**
	* @class SoccerManager
	*
	* @brief Implements the Soccer Manager
	**/
	class SoccerManager : public behaviourcontrol::BehaviourManager
	{
	public:
		// Constructors
		SoccerManager();
		virtual ~SoccerManager();

		// Child layers
		RosInterfaceLayer* RosIL;
		SoccerLayer* SL;
		ControlLayer* CL;

		// Function overrides
		virtual ret_t init();
		virtual void preStepCallback();
		virtual void postStepCallback();

		// Error notification functions
		virtual void reportErrorUser(const std::string& msg, bool fatal, const std::string& funcName, const std::string& fileName, int line);

		//
		// Extra stuff
		//

		// Miscellaneous
		bool getEnabled() const;
		void halt();

		void handleLEDTimer();

		enum State
		{
			STATE_HALT,
			STATE_WAIT,
			STATE_RUN,
			STATE_RUN_MANUAL
		};

		inline State state() const
		{ return m_state; }

		void setState(State state);

		robotcontrol::LEDCommand led;
	private:
		State m_state;
		ros::Publisher m_pub_leds;
		ros::Timer m_ledTimer;
	};
}

#endif /* SOCCER_MANAGER_H */
// EOF