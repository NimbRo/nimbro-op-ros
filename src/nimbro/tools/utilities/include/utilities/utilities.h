// Header file that includes all utilities header files
// File: utilities.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef UTILITIES_H
#define UTILITIES_H

/**
* @namespace util
*
* @brief This namespace defines a set of various utilities, both ROS dependent and ROS agnostic,
* that can be used for common fundamental tasks.
**/
namespace util {}

// Includes
#include <utilities/markers.h>
#include <utilities/mathconv.h>
#include <utilities/plotting.h>
#include <utilities/rostiming.h>

#endif /* UTILITIES_H */
// EOF