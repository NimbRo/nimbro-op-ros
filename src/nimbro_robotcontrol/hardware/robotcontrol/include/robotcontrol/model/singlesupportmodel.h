// RBDL model for endeffector -> body tree
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef SINGLESUPPORTMODEL_H
#define SINGLESUPPORTMODEL_H

#include <rbdl/rbdl_parser.h>

#include <robotcontrol/model/joint.h>

#include <boost/shared_ptr.hpp>

namespace Math = RigidBodyDynamics::Math;

namespace robotcontrol
{
class RobotModel;

/**
 * @brief RBDL model for endeffector -> body tree
 **/
class SingleSupportModel : public rbdl_parser::URDF_RBDL_Model
{
public:
	enum DataSource
	{
		MeasurementData,
		CommandData
	};

	SingleSupportModel(RobotModel* model, const boost::shared_ptr<const urdf::Link>& link);

	inline boost::shared_ptr<const urdf::Link> link() const
	{ return m_link; }

    virtual bool initFrom(const urdf::Model& model, const std::string& root = "");

	void setCoefficient(double coeff);
	inline double coefficient() const
	{ return m_coeff; }

	void normalize(double total);

	void doInverseDynamics(bool normalize = true, DataSource source = CommandData);

	void computeCOM();
	void computeZMP();

	inline Math::Vector3d centerOfMass() const
	{ return m_com; }

	inline Math::Vector3d zeroMomentPoint() const
	{ return m_zmp; }

	inline Math::Vector3d torqueZeroMomentPoint() const
	{ return m_zmp_torque; }

	inline Math::Vector3d footForce() const
	{ return m_footForce; }

	void setJoint(int idx, double angle, const ros::Time& time);
	void setJoint_unsmoothed(int idx, double angle, const ros::Time&);

	inline Joint::Ptr joint(int idx)
	{ return m_joints[idx]; }

	void updateKinematics(DataSource source = CommandData);
protected:
	virtual void setupJoint(int index, const urdf::Joint& urdf, bool reverse);
private:
	RobotModel* m_model;
	boost::shared_ptr<const urdf::Link> m_link;
	std::vector<Joint::Ptr> m_joints;
	Math::VectorNd m_q;
	Math::VectorNd m_qdot;
	Math::VectorNd m_qdotdot;
	Math::VectorNd m_tau;
	double m_coeff;
	double m_normalizedCoeff;
	int m_trunkJoint;

	Math::Vector3d m_com;
	Math::Vector3d m_zmp;
	Math::Vector3d m_zmp_torque;
	Math::Vector3d m_footForce;

	void updateJointData(DataSource source = CommandData);
};

}

#endif
