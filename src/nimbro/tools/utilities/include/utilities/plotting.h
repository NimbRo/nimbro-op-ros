// Utilities for plotting to the plotter widget
// File: plotting.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef PLOTTING_H
#define PLOTTING_H

// Includes
#include <string>
#include <ros/node_handle.h>
#include <ros/this_node.h> // Note: ros::this_node::getName() returns the node name with a '/' at the front!
#include <plot_msgs/Plot.h>

// Utilities namespace
namespace util
{
	/**
	* @class PlotManager
	*
	* @brief Class that facilitates the plotting of data to the Plotter widget
	*
	* In general only one PlotManager instance should be used per @b node, and all parts of the
	* node should plot using the functions of that PlotManager. The basic process is this (assuming
	* there is some kind of a step function that gets called in a timed main loop to do all the
	* required processing of the node):
	* @code
	* // In class...
	* util::PlotManager PM;
	* 
	* // In class constructor initialiser list... (Only necessary at all if you don't want to use the default base name of "~", which becomes the node name)
	* , PM("/my_base_name")
	* 
	* // In the step function/main loop...
	* PM.clear(); // <-- This sets the ROS timestamp of the points plotted in the next call to publish()
	* ...
	* myvec = big_calculation();
	* PM.plotVec3d(myvec);
	* ...
	* myresult = another_calculation();
	* PM.plotScalar(myresult);
	* ...
	* PM.publish();
	* @endcode
	* There should be no sleeping/ROS spinning between `PM.clear()` and `PM.publish()` or the
	* timestamps get less accurate.
	**/
	class PlotManager
	{
	public:
		// Constants
		static const std::string DEFAULT_BASENAME; //!< @brief Default base name to use for the plotted data (expands to the node name)
		
		/**
		* @brief Default constructor
		*
		* @param baseName A `std::string` specifying the base name to use for all variables plotted by
		* this class instance. This controls the location of the data in the plotter tree hierarchy.
		* If the first character is @c ~ then it is replaced by the node name (e.g. @c ~foo becomes @c /node_name/foo/).
		* @param enabled This controls the initial enabled state of the PlotManager. The enabled state
		* can later be changed using the enable() and disable() functions, and controls whether the
		* PlotManager actually publishes anything.
		**/
		explicit PlotManager(const std::string& baseName = DEFAULT_BASENAME, bool enabled = true) : enabled(enabled)
		{
			// Terminate both ends of the basename with slashes
			basename = baseName;
			if(basename.empty()) basename = DEFAULT_BASENAME;
			if(basename.at(0) == '~') basename.replace(0, 1, ros::this_node::getName() + "/");
			if(basename.at(0) != '/') basename.insert(0, "/");
			if(basename.at(basename.length()-1) != '/') basename.insert(basename.length(), "/");

			// Work out the vector basenames
			basenamex = basename + "x/";
			basenamey = basename + "y/";
			basenamez = basename + "z/";

			// Advertise on the plot topic
			ros::NodeHandle nh("~");
			m_pub_plot = nh.advertise<plot_msgs::Plot>("/plot", 1);
		}

		// Get functions
		const std::string& getBasename() const { return basename; } //!< @brief Returns the base name in use for all variables plotted by this PlotManager instance. This controls the location of the data in the plotter tree hierarchy.

		// Properties
		void enable()  { enabled = true; } //!< @brief Enables the PlotManager
		void disable() { enabled = false; } //!< @brief Disables the PlotManager

		// Plot functions
		//! @brief Clears the current internal list of plot points. Call this at the start of every plot cycle.
		void clear()
		{
			// Clear the current list of points
			m_plot.points.clear();
			m_plot.header.stamp = ros::Time::now();
		}
		//! @brief Publishes the collected plot points (if the PlotManager is enabled). Call this at the end of every plot cycle (without a sleep between the call to clear and this or the timestamps get inaccurate).
		void publish()
		{
			// Publish the list to the configuration server
			if(enabled)
				m_pub_plot.publish(m_plot);
		}
		//! @brief Plot a scalar value under the given @p name
		void plotScalar(double value, const std::string& name)
		{
			// Package up the point and queue it away
			if(enabled)
			{
				m_point.name = basename + name;
				m_point.value = value;
				m_plot.points.push_back(m_point);
			}
		}
		//! @brief Plot a 2D Eigen vector under the given @p name
		void plotVec2d(const Eigen::Vector2d& vec2d, const std::string& name)
		{
			// Package up the 2D vector and queue it away
			if(enabled)
			{
				m_point.name = basenamex + name;
				m_point.value = vec2d.x();
				m_plot.points.push_back(m_point);
				m_point.name = basenamey + name;
				m_point.value = vec2d.y();
				m_plot.points.push_back(m_point);
			}
		}
		//! @brief Plot a 2D vector `(x,y)` under the given @p name
		void plotVec2d(double x, double y, const std::string& name)
		{
			// Package up the 2D vector and queue it away
			if(enabled)
			{
				m_point.name = basenamex + name;
				m_point.value = x;
				m_plot.points.push_back(m_point);
				m_point.name = basenamey + name;
				m_point.value = y;
				m_plot.points.push_back(m_point);
			}
		}
		//! @brief Plot a 3D Eigen vector under the given @p name
		void plotVec3d(const Eigen::Vector2d& vec3d, const std::string& name)
		{
			// Package up the 3D vector and queue it away
			if(enabled)
			{
				m_point.name = basenamex + name;
				m_point.value = vec3d.x();
				m_plot.points.push_back(m_point);
				m_point.name = basenamey + name;
				m_point.value = vec3d.y();
				m_plot.points.push_back(m_point);
				m_point.name = basenamez + name;
				m_point.value = vec3d.z();
				m_plot.points.push_back(m_point);
			}
		}
		//! @brief Plot a 3D vector `(x,y,z)` under the given @p name
		void plotVec3d(double x, double y, double z, const std::string& name)
		{
			// Package up the 3D vector and queue it away
			if(enabled)
			{
				m_point.name = basenamex + name;
				m_point.value = x;
				m_plot.points.push_back(m_point);
				m_point.name = basenamey + name;
				m_point.value = y;
				m_plot.points.push_back(m_point);
				m_point.name = basenamez + name;
				m_point.value = z;
				m_plot.points.push_back(m_point);
			}
		}

	private:
		// Internal variables
		ros::Publisher m_pub_plot;
		plot_msgs::Plot m_plot;
		plot_msgs::PlotPoint m_point;
		std::string basename, basenamex, basenamey, basenamez;
		bool enabled;
	};

	// PlotManager constants
	const std::string PlotManager::DEFAULT_BASENAME = "~";
}

#endif /* PLOTTING_H */
// EOF