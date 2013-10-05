// Hard-iron compass correction
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef COMPASSFILTER_H
#define COMPASSFILTER_H

#include <Eigen/Core>
#include <config_server/parameter.h>
#include <std_srvs/Empty.h>

namespace robotcontrol
{

class CompassFilter
{
public:
	CompassFilter();
	~CompassFilter();

	void update(const Eigen::Vector3d& measurement);
	inline Eigen::Vector3d value() const
	{ return m_value; }
private:
	bool startCalibration(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);
	bool stopCalibration(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);

	config_server::Parameter<float> m_hard_x;
	config_server::Parameter<float> m_hard_y;
	config_server::Parameter<float> m_hard_z;

	Eigen::Vector3d m_value;

	bool m_calibrating;

	std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_calibrationMeasurements;

	ros::ServiceServer m_srv_startCalibration;
	ros::ServiceServer m_srv_stopCalibration;
};

}

#endif
