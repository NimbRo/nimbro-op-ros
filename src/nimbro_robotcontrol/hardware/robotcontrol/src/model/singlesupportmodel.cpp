// RBDL model for endeffector -> body chain
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <robotcontrol/model/singlesupportmodel.h>
#include <robotcontrol/model/robotmodel.h>

#include <ros/console.h>

#include <tf/tf.h>

#include <rbdl/Dynamics.h>
#include <rbdl/Kinematics.h>

#include <nimbro_op_model/dimensions.h>

namespace robotcontrol
{

SingleSupportModel::SingleSupportModel(RobotModel* model, const boost::shared_ptr<const urdf::Link>& link)
 : m_model(model)
 , m_link(link)
 , m_coeff(0)
 , m_trunkJoint(-1)
 , m_com(Math::Vector3d::Zero())
 , m_zmp(Math::Vector3d::Zero())
{
}

bool SingleSupportModel::initFrom(const urdf::Model& model, const std::string& root)
{
	if(!rbdl_parser::URDF_RBDL_Model::initFrom(model, root))
		return false;

	int nj = dof_count;
	m_q.resize(nj);
	m_qdot.resize(nj);
	m_qdotdot.resize(nj);
	m_tau.resize(nj);

	gravity << 0, 0, -9.81;

	return true;
}

void SingleSupportModel::setCoefficient(double coeff)
{
	m_coeff = coeff;
}

void SingleSupportModel::normalize(double total)
{
	if(total == 0)
		m_normalizedCoeff = 0;
	else
		m_normalizedCoeff = m_coeff / total;
}

void SingleSupportModel::updateJointData(DataSource source)
{
	for(size_t i = 0; i < m_joints.size(); ++i)
	{
		if(!m_joints[i])
			continue;

		const Joint& joint = *m_joints[i];

		if(source == MeasurementData)
			m_q[i] = joint.feedback.pos;
		else
			m_q[i] = joint.cmd.pos;

		if(isnan(joint.cmd.vel))
			m_qdot[i] = 0;
		else
			m_qdot[i] = joint.cmd.vel;

		if(isnan(joint.cmd.acc))
			m_qdotdot[i] = 0;
		else
			m_qdotdot[i] = joint.cmd.acc;
	}
}

void SingleSupportModel::updateKinematics(DataSource source)
{
	updateJointData(source);
	RigidBodyDynamics::UpdateKinematicsCustom(*this, &m_q, 0, 0);
}

void SingleSupportModel::doInverseDynamics(bool normalize, DataSource source)
{
	if(m_coeff == 0 && normalize)
		return;

	double coeff = m_normalizedCoeff;
	if(!normalize)
		coeff = 1.0;

	updateKinematics(source);

	tf::Vector3 grav(0, 0, -9.81);
	grav = tf::Transform(m_model->robotOrientation()).inverse() * grav;

	// Transform grav to base coordinates
	if(m_trunkJoint != -1)
	{
		const Math::SpatialTransform X = X_base[m_trunkJoint];
		gravity = X.E.transpose() * Math::Vector3d(grav.x(), grav.y(), grav.z());
	}

	RigidBodyDynamics::InverseDynamics(*this, m_q, m_qdot, m_qdotdot, m_tau, 0);

	for(size_t i = 0; i < m_joints.size(); ++i)
	{
		if(!m_joints[i])
			continue;

		m_joints[i]->feedback.modelTorque += coeff * m_tau[i];
	}
}

void SingleSupportModel::setupJoint(int index, const urdf::Joint& urdf, bool reverse)
{
	if(index-1 >= (int)m_joints.size())
		m_joints.resize(index);

	if(!reverse && urdf.child_link_name == "trunk_link")
		m_trunkJoint = index;
	else if(reverse && urdf.parent_link_name == "trunk_link")
		m_trunkJoint = index;

	Joint::Ptr joint = m_model->getJoint(urdf.name);

	ROS_ASSERT(joint);

	m_joints[index-1] = joint;
}

void SingleSupportModel::computeCOM()
{
	Math::Vector3d com = Math::Vector3d::Zero();
	double mass = 0;

	updateKinematics(MeasurementData);

	for(size_t i = 0; i < mBodies.size(); ++i)
	{
		const Math::SpatialTransform& X = X_base[i];
		const RigidBodyDynamics::Body& body = mBodies[i];

		Math::Vector3d bodyCom = X.E.transpose() * body.mCenterOfMass + X.r;

		com += body.mMass * bodyCom;
		mass += body.mMass;
	}

	m_com = com / mass;
}

void SingleSupportModel::computeZMP()
{
	if(m_coeff == 0)
	{
		ROS_ERROR("SingleSupportModel: computeZMP() was called with zero support coefficient");
		return;
	}

	Math::SpatialVector spatialFootForce = -X_base[1].applyTranspose(f[1]);

	double mass = link()->inertial->mass;
	Eigen::Vector3d centerOfMass(
		link()->inertial->origin.position.x,
		link()->inertial->origin.position.y,
		link()->inertial->origin.position.z
	);

	double F = -mass * 9.81;

	double total_Fz = spatialFootForce[5] + F;
	double total_Mx = spatialFootForce[0] + F * centerOfMass.y() - spatialFootForce[4] * nimbro_op_model::ANKLE_Z_HEIGHT;
	double total_My = spatialFootForce[1] - F * centerOfMass.x() + spatialFootForce[3] * nimbro_op_model::ANKLE_Z_HEIGHT;

	m_footForce = Eigen::Vector3d(0.0, 0.0, total_Fz);

	m_zmp = Eigen::Vector3d(-total_My / total_Fz, total_Mx / total_Fz, 0);

	// ZMP estimation based on the measured ankle roll torque
	// TODO: Do this in a more general way?
	Joint::Ptr ankle_roll = m_joints[0];
	double total_Mx_torque = ankle_roll->feedback.torque + F * centerOfMass.y() - spatialFootForce[4] * nimbro_op_model::ANKLE_Z_HEIGHT;
	m_zmp_torque = Eigen::Vector3d(-total_My / total_Fz, total_Mx_torque / total_Fz, 0);
}

void SingleSupportModel::setJoint(int index, double angle, const ros::Time& time)
{
	m_joints[index]->cmd.setFromPos(time, angle);
}

void SingleSupportModel::setJoint_unsmoothed(int index, double angle, const ros::Time&)
{
	m_joints[index]->cmd.setFromPos_unsmoothed(angle);
}

}
