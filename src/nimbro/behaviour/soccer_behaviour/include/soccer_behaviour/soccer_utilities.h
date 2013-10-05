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
#include <ros/time.h>
#include <ros/service_client.h>

// Includes - Libraries
#include <behaviour_control/behaviour_control.h>
#include <boost/utility/enable_if.hpp> // For boost::enable_if
#include <boost/type_traits/is_convertible.hpp> // For boost::is_convertible

// Includes - Template specialisation types
#include <gait/GaitCommand.h>

// Includes - C++ Standard Library
#include <cstddef>

// Soccer behaviour namespace
namespace soccerbehaviour
{
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
	* @class RosTimeMarker
	*
	* @brief Class that facilitates duration timing using the ROS-provided time
	**/
	class RosTimeMarker
	{
	public:
		// Constructors
		RosTimeMarker() : markerTime(0), iHaveMarker(false) {}

		// Timing functions
		void reset() { iHaveMarker = false; }
		void setMarker()
		{
			// Record the current ROS time
			markerTime = ros::Time::now();
			iHaveMarker = true;
		}
		bool haveMarker() const { return iHaveMarker; }
		double getElapsed() const { return (iHaveMarker ? (ros::Time::now() - markerTime).toSec() : -1.0); } //!< Returns the current elapsed time since the marker was set (returns -1.0 if no marker has been set - check this if you must as `getElapsed() < 0.0`)
		bool hasElapsed(double duration) const { return !iHaveMarker || (iHaveMarker && ((ros::Time::now() - markerTime).toSec() >= duration)); } //!< Returns whether a certain time duration has elapsed since the time marker was set (returns true if no marker has been set)

	private:
		// Internal variables
		ros::Time markerTime;
		bool iHaveMarker;
	};

	/**
	* @class RosTimeTracker
	*
	* @brief Class that facilitates multi-duration time tracking using the ROS-provided time
	**/
	class RosTimeTracker
	{
	public:
		// Constructors
		explicit RosTimeTracker(std::size_t N) : N(N)
		{
			// Make sure we have room for at least one marker
			if(N < 1) N = 1;

			// Allocate memory for the marker and flag arrays
			markerTime = new ros::Time[N];
			iHaveMarker = new bool[N];

			// Initialise the arrays explicitly
			for(std::size_t i = 0;i < N;i++)
			{
				markerTime[i].fromSec(0);
				iHaveMarker[i] = false;
			}
		}
		virtual ~RosTimeTracker() { delete[] markerTime; }

		// Timing functions (in all these functions m is the index of the marker, valid values are 0 -> N-1)
		void setMarker(std::size_t m)
		{
			// Error checking
			if(m >= N) return;

			// Record the current ROS time in the appropriate marker
			markerTime[m] = ros::Time::now();
			iHaveMarker[m] = true;
		}
		bool haveMarker(std::size_t m) const { return (m < N ? iHaveMarker[m] : false); }
		double getElapsed(std::size_t m) const
		{
			// Return the elapsed time since marker m was set
			if(m >= N) return -1.0;
			else return (iHaveMarker[m] ? (ros::Time::now() - markerTime[m]).toSec() : -1.0);
		}
		bool hasElapsed(std::size_t m, double duration) const
		{
			// Return whether the given duration has elapsed since marker m was set
			if(m >= N) return false;
			else return !iHaveMarker[m] || (iHaveMarker[m] && ((ros::Time::now() - markerTime[m]).toSec() >= duration));
		}

	private:
		// Internal variables
		ros::Time* markerTime;
		bool* iHaveMarker;
		std::size_t N;
	};

	/**
	* @class RosServiceCaller
	*
	* @brief Provides some basic functionality for controlling calls to ROS services
	**/
	template <class T>
	class RosServiceCaller
	{
	public:
		// Constructors
		RosServiceCaller(double reissueDelay, double failRetryDelay, behaviourcontrol::Sensor<T>* sensor = NULL)
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

			// Return whether the service call was carried out AND was successful (or not)
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