// Angle estimator based on gyro velocities
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ANGLEESTIMATOR_H
#define ANGLEESTIMATOR_H
#include <string>

#include <Eigen/Core>

#include <boost/circular_buffer.hpp>

namespace robotcontrol
{
class AngleEstimator
{
public:
	AngleEstimator();
	virtual ~AngleEstimator();

	void predict(double gyro_dpitch, double gyro_droll, double gyro_dyaw = 0);
	void update(double acc_x, double acc_y, double acc_z);

	void setAccGain(double acc_gain);
	void setAccSmoothDecay(double acc_smooth_decay);

	inline double accGain() const
	{ return m_acc_gain; }

	inline double accSmoothDecay() const
	{ return m_acc_smooth_decay; }

	inline double roll() const
	{ return -m_angle.y(); }

	inline double pitch() const
	{ return m_angle.x(); }


private:
	double m_acc_gain;
	double m_acc_smooth_decay;

	double m_acc_x;
	double m_acc_y;
	double m_acc_z;

	Eigen::Vector2d m_acc;

	Eigen::Vector2d m_angle;
	Eigen::Vector2d m_gyro;
	Eigen::Vector2d m_gyroBias;

	typedef boost::circular_buffer<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > VectorBuffer;
	VectorBuffer m_accBuffer;
	VectorBuffer m_gyroBuffer;
	unsigned int m_tikCount;
    double m_lambda;


	void estimateBias();
};

}

#endif
