// Behaviour Control Framework - Soccer Layer
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <soccer_behaviour/soccer_layer/soccer_layer.h>
#include <soccer_behaviour/soccer_manager/soccer_manager.h>

// Namespaces
using namespace std;
using namespace soccerbehaviour;

//
// SoccerLayer class
//

// Contructor
SoccerLayer::SoccerLayer(SoccerManager* M) : BehaviourLayer(M, "SoccerLayer"), M(M)
{
	// Create sensor and actuator managers
	SM = new SoccerLayerSM(this);
	AM = new SoccerLayerAM(this);

	// Create child behaviours
	playSoccer = new PlaySoccer(this);

	// Add inhibitions
	/* No inhibitions required yet */
}

// Destructor
SoccerLayer::~SoccerLayer()
{
	// Delete child behaviours
	delete playSoccer;

	// Delete sensor and actuator managers
	delete SM;
	delete AM;
}

// Initialisation function
ret_t SoccerLayer::init()
{
	// Return that initialisation was successful
	return RET_OK;
}

// Update function
void SoccerLayer::update()
{
}

// Post-execute callback
void SoccerLayer::postExecuteCallback()
{
}

// Extra stuff
/* TODO: Define other functions that you need here */
// EOF