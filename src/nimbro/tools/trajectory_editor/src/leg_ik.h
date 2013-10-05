// Analytical Inverse Kinematics for robot legs
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LEG_IK_H
#define LEG_IK_H

#include <rbdl/Model.h>

#include <rbdl/rbdl_parser.h>

#include <ros/publisher.h>

namespace Math = RigidBodyDynamics::Math;

class LegIK
{
public:
	LegIK(const boost::shared_ptr<rbdl_parser::URDF_RBDL_Model>& model, const std::string& tip);
	virtual ~LegIK();

	bool doIK(boost::function<void(int, double)> cb, const Eigen::Vector3d& footLocation, const Eigen::Matrix3d& footRot = Eigen::Matrix3d::Identity());
private:
	typedef Eigen::Matrix<double, 6, 1> JointVec;

	bool doIK(JointVec* q, const Eigen::Vector3d& footLocation, const Eigen::Matrix3d& footRot = Eigen::Matrix3d::Identity());

	boost::shared_ptr<rbdl_parser::URDF_RBDL_Model> m_model;
	std::string m_tip;
	int m_sign;

	std::vector<unsigned int> m_links;

	Math::SpatialTransform m_hipTransform;
	double m_shankLength;
	double m_shankLength2;
	double m_thighLength;
	double m_thighLength2;

	double m_lengthDiv;

	ros::Publisher m_pub_markers;

	Eigen::Vector3d m_hip_yaw_pitch_offset;
};

#endif
