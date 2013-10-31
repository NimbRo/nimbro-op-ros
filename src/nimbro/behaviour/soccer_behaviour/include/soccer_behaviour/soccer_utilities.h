// Soccer Behaviour - Utilities
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

/**
* @file soccer_utilities.h
* @brief Defines some useful utilities for the Soccer Behaviour package.
**/

// Ensure header is only included once
#ifndef SOCCER_UTILITIES_H
#define SOCCER_UTILITIES_H

// Includes - ROS
#include <ros/service_client.h>

// Includes - Libraries
#include <utilities/rostiming.h>
#include <behaviour_control/behaviour_control.h>
#include <boost/utility/enable_if.hpp> // For boost::enable_if
#include <boost/type_traits/is_convertible.hpp> // For boost::is_convertible

// Includes - Template specialisation types
#include <gait/GaitCommand.h>

// Soccer behaviour namespace
namespace soccerbehaviour
{
	// Namespaces (import some of the utilities classes)
	using util::RosTimeMarker;
	using util::RosTimeTracker;
	using util::RosServiceCaller;
	
	/**
	* @class ChangeMonitor
	*
	* @brief Template class to detect changes in a particular data signal
	**/
	template <class T>
	class ChangeMonitor
	{
	public:
		// Constructors
		explicit ChangeMonitor(bool initChanged = true) : initChanged(initChanged), data() { reset(); }

		// Reset function
		void reset() { hasCalled = false; }

		// Data comparison and change functions
		template <typename U>
		bool areEqual(T data1, U data2, typename boost::enable_if<boost::is_convertible<U, T> >::type* dummy = 0) const { return (data1 == ((T) data2)); } // This needs to be specialised for T's that don't define '==', or this means the wrong thing
		bool hasChanged(T currentVal)
		{
			// If this is the first call to hasChanged() then return the default value for this situation
			if(!hasCalled)
			{
				hasCalled = true;
				data = currentVal;
				return initChanged;
			}

			// Return whether the current value is the same as the last one we have stored, then store the new value
			bool ret = !areEqual(data, currentVal);
			data = currentVal;
			return ret;
		}

	protected:
		// Internal variables
		bool hasCalled;
		const bool initChanged;
		T data;
	};

	//
	// ChangeMonitor function specialisations
	//
// 	template <> template <typename U> // <== Use this as a model for new specialisations (replace MYCLASS with your type)!
// 	bool ChangeMonitor<MYCLASS>::areEqual(MYCLASS data1, U data2, typename boost::enable_if<boost::is_convertible<U, MYCLASS> >::type* dummy) const
// 	{
// 		// Return true if data1 is equal to data2
// 		return (data1 == (MYCLASS) data2); // <== Replace this condition with whatever makes sense! (Keep in mind that data2 may be of a different yet convertible data type, so for example int == float would get promoted to a float == float comparison, which may not be wanted)
// 	}
	template <> template <typename U>
	bool ChangeMonitor<gait::GaitCommand>::areEqual(gait::GaitCommand data1, U data2, typename boost::enable_if<boost::is_convertible<U, gait::GaitCommand> >::type* dummy) const
	{
		// Returns true if both walk's are equal and false, or if both walk's are equal and true and the gcv vectors are equal (IGNORES KICK MEMBER)
		return (data1.walk == data2.walk) && ((data1.walk == false) ||
			((data1.walk == true) && (data1.gcvX == data2.gcvX) && (data1.gcvY == data2.gcvY) && (data1.gcvZ == data2.gcvZ)));
	}

	/**
	* @class BCFRosServiceCaller
	*
	* @brief Provides some basic functionality for controlling calls to ROS services (specific to the Behaviour Control Framework)
	**/
	template <class T>
	class BCFRosServiceCaller
	{
	public:
		// Constructors
		BCFRosServiceCaller(double reissueDelay, double failRetryDelay, behaviourcontrol::Sensor<T>* sensor = NULL)
			: sensor(sensor)
			, failRetryDelay(failRetryDelay)
			, reissueDelay(reissueDelay)
		{}

		// Service client set function
		void setServiceClient(const ros::ServiceClient& SC) { m_srv = SC; }

		// Service call functions
		bool callService()
		{
			// Automatically retrieve data from the sensor if we can, else this overload fails (the user should have used the other overload)
			if(sensor != NULL)
				return callService(sensor->read());
			else
				return false;
		}
		bool callService(T data)
		{
			// Declare variables
			bool ret = false;

			// Decide whether to call the service or not (don't want to spam the service by calling it every time!)
			if((sensor == NULL || sensor->wasWrittenTo()) && lastGoodCall.hasElapsed(reissueDelay) && lastBadCall.hasElapsed(failRetryDelay))
			{
				ret = m_srv.call<T>(data);
				if(ret) lastGoodCall.setMarker();
				else lastBadCall.setMarker();
			}

			// Return whether the service call was carried out AND was successful
			return ret;
		}

	private:
		// Internal variables
		behaviourcontrol::Sensor<T>* sensor;
		const double failRetryDelay;
		const double reissueDelay;
		ros::ServiceClient m_srv;
		RosTimeMarker lastGoodCall;
		RosTimeMarker lastBadCall;
	};
}

#endif /* SOCCER_UTILITIES_H */
// EOF