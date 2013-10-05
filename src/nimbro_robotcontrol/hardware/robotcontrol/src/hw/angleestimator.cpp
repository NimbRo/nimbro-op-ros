// Angle estimator based on gyro velocities
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <robotcontrol/hw/angleestimator.h>

#include <ros/console.h>

#include <math.h>

namespace robotcontrol
{

inline double min(double a, double b)
{return (a < b) ? a : b;}

inline double max(double a, double b)
{return (a > b) ? a : b;}

AngleEstimator::AngleEstimator()
 : m_acc_gain(0.005)
 , m_acc_smooth_decay(1.0)
 , m_acc_x(0)
 , m_acc_y(0)
 , m_acc_z(0)
 , m_acc(0,0)
 , m_angle(0,0)
 , m_gyro(0,0)
 , m_gyroBias(0,0)
 , m_accBuffer(200)
 , m_gyroBuffer(200)
 , m_tikCount(0)
 , m_lambda(0)
{
}

AngleEstimator::~AngleEstimator()
{
}

void AngleEstimator::predict(double gyro_dpitch, double gyro_droll, double gyro_dyaw)
{
	m_angle.x() += gyro_dpitch + m_gyroBias.x();
	m_angle.y() += gyro_droll + m_gyroBias.y();

	m_gyro.x() += gyro_dpitch;
	m_gyro.y() += gyro_droll;
 	estimateBias();
}

void AngleEstimator::update(double x, double y, double z)
{
	m_acc_x = (1.0 - m_acc_smooth_decay) * m_acc_x + m_acc_smooth_decay * x;
	m_acc_y = (1.0 - m_acc_smooth_decay) * m_acc_y + m_acc_smooth_decay * y;
	m_acc_z = (1.0 - m_acc_smooth_decay) * m_acc_z + m_acc_smooth_decay * z;

	m_acc <<
		-atan2(-m_acc_x, sqrt(m_acc_y*m_acc_y+m_acc_z*m_acc_z)),
		-atan2(-m_acc_y, -m_acc_z);

// 	m_angle = (1.0 - m_acc_gain) * m_angle + m_acc_gain * m_acc;
	m_angle = m_lambda * ((1.0 - m_acc_gain) * m_angle + m_acc_gain * m_acc) + (1.0 - m_lambda) * m_acc;
}

void AngleEstimator::estimateBias()
{
	m_lambda = min(1.0, max(0.0, m_tikCount/500.0 - 0.4));

	if (m_tikCount > 200)
	{
		Eigen::Vector2d oldAccAngle = m_accBuffer[0];
		Eigen::Vector2d oldGyroAngle = m_gyroBuffer[0];

		Eigen::Vector2d gyroDelta = m_gyro - oldGyroAngle;
		Eigen::Vector2d angleBiasDelta = (m_acc - oldAccAngle) - gyroDelta;
		angleBiasDelta *= 1.0/200.0;

		double gamma = (1.0 - m_lambda) * 0.02 + 0.001;
		m_gyroBias += gamma * (angleBiasDelta  - m_gyroBias);
	}


	m_accBuffer.push_back(m_acc);
	m_gyroBuffer.push_back(m_gyro);
	if (m_tikCount < 5000)
		m_tikCount++;
}

}
