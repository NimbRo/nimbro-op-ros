// Hard-iron compass correction
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <robotcontrol/hw/compassfilter.h>

#include <Eigen/SVD>
#include <std_srvs/Empty.h>

namespace robotcontrol
{

CompassFilter::CompassFilter()
 : m_hard_x("compass/hard/x", -1.0, 0.001, 1.0, 0.0)
 , m_hard_y("compass/hard/y", -1.0, 0.001, 1.0, 0.0)
 , m_hard_z("compass/hard/z", -1.0, 0.001, 1.0, 0.0)
 , m_value(Eigen::Vector3d::Zero())
 , m_calibrating(false)
{
	ros::NodeHandle nh("~");

	m_srv_startCalibration = nh.advertiseService(
		"compass/startCalibration",
		&CompassFilter::startCalibration, this
	);
	m_srv_stopCalibration = nh.advertiseService(
		"compass/stopCalibration",
		&CompassFilter::stopCalibration, this
	);
}

CompassFilter::~CompassFilter()
{
}

void CompassFilter::update(const Eigen::Vector3d& measurement)
{
	Eigen::Vector3d hard_iron(m_hard_x(), m_hard_y(), m_hard_z());

	m_value = measurement - hard_iron;

	if(m_calibrating)
		m_calibrationMeasurements.push_back(measurement);
}

bool CompassFilter::startCalibration(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
{
	ROS_INFO("Starting compass calibration");
	m_calibrating = true;

	return true;
}

bool CompassFilter::stopCalibration(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
{
	size_t cnt = m_calibrationMeasurements.size();
	Eigen::MatrixXd A(cnt, 4);
	Eigen::VectorXd b(cnt);

	for(size_t i = 0; i < cnt; ++i)
	{
		const Eigen::Vector3d m = m_calibrationMeasurements[i];

		A.row(i).head<3>() = m;
		A(i, 3) = 1;

		b(i) = m.squaredNorm();
	}

	Eigen::VectorXd coefficients = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	Eigen::Vector3d hard_iron = coefficients.head<3>() / 2.0;
	double B = sqrt(coefficients(3) - hard_iron.squaredNorm());

	ROS_INFO("Calibration ended with hard iron offset % 10.7lf, % 10.7lf, % 10.7lf and field strength % 10.7lf Gauss",
		hard_iron.x(), hard_iron.y(), hard_iron.z(),
		B
	);

	m_calibrating = false;
	m_hard_x.set((float)hard_iron(0));
	m_hard_y.set((float)hard_iron(1));
	m_hard_z.set((float)hard_iron(2));

	return true;
}

}

