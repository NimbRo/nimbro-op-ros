// Soccer Behaviour - Soccer Layer
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file soccer_layer.h
* @brief Defines the Soccer Layer for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef SOCCER_LAYER_H
#define SOCCER_LAYER_H

// Includes
#include <soccer_behaviour/soccer_common.h>
#include <soccer_behaviour/soccer_layer/play_soccer.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	/**
	* @class SoccerLayer
	*
	* @brief Implements the Soccer Layer
	**/
	class SoccerLayer : public behaviourcontrol::BehaviourLayer
	{
	public:
		// Constructors
		explicit SoccerLayer(SoccerManager* M);
		virtual ~SoccerLayer();

		// Parent manager
		SoccerManager* const M;

		// Sensor and actuator managers
		SoccerLayerSM* SM;
		SoccerLayerAM* AM;

		// Child behaviours
		PlaySoccer* playSoccer;

		// Function overrides
		virtual ret_t init();
		virtual void update();
		virtual void postExecuteCallback();

		//
		// Extra stuff
		//

		/* TODO: Add everything else you need here */
	};

	/**
	* @class SoccerLayerSM
	*
	* @brief Implements the sensor manager for the Soccer Layer
	**/
	class SoccerLayerSM : public behaviourcontrol::SensorManager
	{
	public:
		// Constructors
		explicit SoccerLayerSM(SoccerLayer* L) : SensorManager(L), L(L), M(L->M)
			/* TODO: Construct all the child sensors here (e.g. ", inSignal(this, "PublishingLayer/outSignal");") */
		{}

		// Parent layer and manager
		SoccerLayer* const L;
		SoccerManager* const M;

		// Layer sensors
		/* TODO: Add all the sensors you need here (e.g. "SensorFloat inSignal;") */

		// Function overrides
		virtual ret_t init() { return RET_OK; }
	};

	/**
	* @class SoccerLayerAM
	*
	* @brief Implements the actuator manager for the Soccer Layer
	**/
	class SoccerLayerAM : public behaviourcontrol::ActuatorManager
	{
	public:
		// Constructors
		explicit SoccerLayerAM(SoccerLayer* L) : ActuatorManager(L), L(L), M(L->M)
			/* TODO: Construct all the child actuators here (e.g. ", outSignal(this, "SoccerLayer/outSignal");") */
		{}
		
		// Parent layer and manager
		SoccerLayer* const L;
		SoccerManager* const M;

		// Layer actuators
		/* TODO: Add all the actuators you need here (e.g. "ActuatorFloat outSignal;") */

		// Function overrides
		virtual ret_t init() { return RET_OK; }
	};
}

#endif /* SOCCER_LAYER_H */
// EOF