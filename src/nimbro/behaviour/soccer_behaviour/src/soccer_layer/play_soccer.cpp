// Behaviour Control Framework - Play Soccer behaviour
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <soccer_behaviour/soccer_layer/play_soccer.h>
#include <soccer_behaviour/soccer_layer/soccer_layer.h>

// Namespaces
using namespace std;
using namespace soccerbehaviour;

//
// Constructors
//

// Constructor
PlaySoccer::PlaySoccer(SoccerLayer* L) : Behaviour(L, "PlaySoccer"), L(L), M(L->M), SM(L->SM), AM(L->AM)
{
}

//
// Function overrides
//

// Initialisation function
ret_t PlaySoccer::init()
{
	// Return that initialisation was successful
	return RET_OK;
}

// Update function
void PlaySoccer::update()
{
}

// Compute activation level function
level_t PlaySoccer::computeActivationLevel()
{
	// Return behaviour not active
	return false;
}

// Execute function
void PlaySoccer::execute()
{
}

//
// Extra stuff
//

/* TODO: Define other functions that you need here */
// EOF