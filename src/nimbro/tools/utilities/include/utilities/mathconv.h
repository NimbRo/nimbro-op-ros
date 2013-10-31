// Utilities for simple maths and type conversions
// File: mathconv.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef MATHCONV_H
#define MATHCONV_H

// Includes
#include <cmath>
#include <tf/tf.h>
#include <Eigen/Core>

// Defines
#define M_2PI  (2.0*M_PI)

// Utilities namespace
namespace util
{
	/**
	* @class CubicSpline
	*
	* @brief Class that calculates a desired cubic spline to fit between two boundary conditions
	*
	* To use this class simply instantiate an instance of it, set up the boundary conditions
	* using either the overloaded constructor or the `setParams()` function, and then dereference
	* the class via the () operator to evaluate the spline at a particular time. A static method
	* for one-time calculations has also been implemented.
	*
	* Examples:
	* @code
	* // Sample time we want to calculate the spline at
	* myt = 0.25;
	* 
	* // Params:           xi   vi   xf    vf   dT
	* CubicSpline cspline(0.0, 0.0, 1.0, -1.0, 0.5);
	* myx = cspline(myt);
	* 
	* // OR...
	* 
	* CubicSpline cspline;
	* cspline.setParams(0.0, 0.0, 1.0, -1.0, 0.5);
	* myx = cspline(myt);
	*
	* // OR...
	*
	* myx = CubicSpline::eval(0.0, 0.0, 1.0, -1.0, 0.5, myt);
	* @endcode
	*
	* Note that this class evaluates cubic splines in terms of @c x vs. @c t, as opposed to
	* in the 2D plane between arbitary points.
	**/
	class CubicSpline
	{
	public:
		//! @brief Default constructor. If you use this you need to call `setParams()` manually.
		CubicSpline() : m_xi(0), m_vi(0), m_ai(0), m_jerk(0) {}

		//! @brief Constructor overload: Sets the internal parameters as required, based on the provided boundary conditions. Refer to `setParams()` for more details.
		CubicSpline(double xi, double vi, double xf, double vf, double dT)
		{
			// Evaluate the required parameters
			setParams(xi, vi, xf, vf, dT);
		}

		/**
		* @brief Calculates the required parameters of the spline given the required boundary conditions
		*
		* Passing a zero @p dT results in `NaN` or `Inf` outputs.
		*
		* @param xi The initial position/displacement
		* @param vi The initial velocity/slope
		* @param xf The final position/displacement
		* @param vf The final velocity/slope
		* @param dT The time (horizontal axis when graphing the spline) between the initial and final conditions
		**/
		void setParams(double xi, double vi, double xf, double vf, double dT)
		{
			// Calculate constants C and D
			double C = 6.0*(xf - xi - vi*dT)/(dT*dT);
			double D = 2.0*(vf - vi)/dT;

			// Remember the initial conditions
			m_xi = xi;
			m_vi = vi;

			// Calculate the required initial and final accelerations, and the corresponding jerk (slope of the acceleration ramp)
			m_ai = C - D;
			double af = 2.0*D - C;
			m_jerk = (af - m_ai) / dT;
		}

		//! @brief Evaluates the spline at the required time @p t (@p t is considered to be in the same units as the @c dT that was passed to the `setParams()` function)
		double operator()(double t) const
		{
			// Evaluate the spline at the required time t
			return m_xi + t*(m_vi + t*(m_ai + t*m_jerk/3.0)/2.0);
		}

		/**
		* @brief Static method for one-time evaluation of a cubic spline given the required boundary conditions
		*
		* @param xi The initial position/displacement
		* @param vi The initial velocity/slope
		* @param xf The final position/displacement
		* @param vf The final velocity/slope
		* @param dT The time (horizontal axis when graphing the spline) between the initial and final conditions
		* @param t The time to evaluate the spline at
		**/
		static double eval(double xi, double vi, double xf, double vf, double dT, double t)
		{
			// Calculate constants C and D
			double C = 6.0*(xf - xi - vi*dT)/(dT*dT);
			double D = 2.0*(vf - vi)/dT;

			// Calculate the required initial and final accelerations, and the corresponding jerk (slope of the acceleration ramp)
			double ai = C - D;
			double af = 2.0*D - C;
			double jerk = (af - ai) / dT;
			
			// Evaluate the spline at the required time t
			return xi + t*(vi + t*(ai + t*jerk/3.0)/2.0);
		}

	private:
		// Internal variables
		double m_xi;
		double m_vi;
		double m_ai;
		double m_jerk;
	};

	/**
	* @name Maths Functions (utilities/mathconv.h)
	**/
	///@{

	//! @brief Wrap an angle to (-pi,pi]
	inline double picut(double angle)
	{
		// Return the required result
		return angle + M_2PI*std::floor((M_PI - angle)/M_2PI);
	}

	//! @brief Return the sign of a value (1 or -1)
	inline double sign(double x)
	{
		// Return -1 or 1 as appropriate
		if(x >= 0.0) return  1.0;
		else         return -1.0;
	}
	//! @brief Return the sign of a value (1, 0 or -1)
	inline double sign0(double x)
	{
		// Return -1, 0 or 1 as appropriate
		if     (x >  0.0) return  1.0;
		else if(x == 0.0) return  0.0;
		else              return -1.0;
	}

	//! @brief Coerce the value @p x to be in the range `[min,max]`
	inline double coerce(double x, double min, double max) // Note: The max constraint is considered before the min constraint, in case they conflict (min > max)
	{
		// Coerce the value x to be in the range [min,max]
		if     (x >= max) return max;
		else if(x <= min) return min;
		else              return x;
	}
	//! @brief Coerce the value @p x to be in the range `[-max_abs,max_abs]`
	inline double coerceAbs(double x, double max_abs)
	{
		// Coerce the value x to be in the range [-max_abs,max_abs]
		if     (x >=  max_abs) return  max_abs;
		else if(x <= -max_abs) return -max_abs;
		else                   return  x;
	}
	//! @brief Coerce the value @p x to be in the range `(-&infin;,max]`
	inline double coerceMax(double x, double max)
	{
		// Coerce the value x to (-Inf,max]
		return (x >= max ? max : x);
	}
	//! @brief Coerce the value @p x to be in the range `[min,&infin;)`
	inline double coerceMin(double x, double min)
	{
		// Coerce the value x to [min,Inf)
		return (x <= min ? min : x);
	}
	///@}

	/**
	* @name Conversion Functions (utilities/mathconv.h)
	**/
	///@{

	//! @brief Eigen: Extend a 2D vector into a 3D vector by a zero z component
	inline Eigen::Vector3d zeroZ(const Eigen::Vector2d& vec)
	{
		// Return the required 3D vector
		return Eigen::Vector3d(vec.x(), vec.y(), 0.0);
	}
	//! @brief Eigen: Extend a 2D vector into a 3D vector by a given z component
	inline Eigen::Vector3d withZ(const Eigen::Vector2d& vec, double z)
	{
		// Return the required 3D vector
		return Eigen::Vector3d(vec.x(), vec.y(), z);
	}

	//! @brief Convert an Eigen vector into a TF vector
	inline tf::Vector3 eigenToTF(const Eigen::Vector3d& vec)
	{
		// Return the required tf vector
		return tf::Vector3(vec.x(), vec.y(), vec.z());
	}
	//! @brief Convert a 3x3 Eigen matrix into a 3x3 TF matrix
	inline tf::Matrix3x3 eigenToTF(const Eigen::Matrix3d& mat)
	{
		// Return the required tf matrix
		return tf::Matrix3x3(
			mat(0, 0), mat(0, 1), mat(0, 2),
			mat(1, 0), mat(1, 1), mat(1, 2),
			mat(2, 0), mat(2, 1), mat(2, 2)
		);
	}
	///@}
}

#endif /* MATHCONV_H */
// EOF