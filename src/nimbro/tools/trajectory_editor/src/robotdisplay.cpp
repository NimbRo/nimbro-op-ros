// Displays the robot in a pose defined by joint angles
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "robotdisplay.h"

#include <rviz/robot/robot.h>

#include <rviz/robot/link_updater.h>

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include <OGRE/OgreMatrix3.h>

class RBDLLinkUpdater : public rviz::LinkUpdater
{
public:
	RBDLLinkUpdater(RigidBodyDynamics::Model* model)
	 : m_model(model)
	{}

	virtual bool getLinkTransforms(const std::string& link_name,
		Ogre::Vector3& visual_position,
		Ogre::Quaternion& visual_orientation,
		Ogre::Vector3& collision_position,
		Ogre::Quaternion& collision_orientation) const
	{
		unsigned int id = m_model->GetBodyId(link_name.c_str());

		if(id >= m_model->X_base.size())
		{
			visual_position = Ogre::Vector3::ZERO;
			visual_orientation = Ogre::Quaternion::IDENTITY;
			return true;
		}

		RigidBodyDynamics::Math::SpatialTransform trans = m_model->X_base[id];

		visual_position = Ogre::Vector3(
			trans.r.x(), trans.r.y(), trans.r.z()
		);

		Ogre::Matrix3 mat(
			trans.E(0, 0), trans.E(1, 0), trans.E(2, 0),
			trans.E(0, 1), trans.E(1, 1), trans.E(2, 1),
			trans.E(0, 2), trans.E(1, 2), trans.E(2, 2)
		);
		visual_orientation.FromRotationMatrix(mat);

		return true;
	}
private:
	RigidBodyDynamics::Model* m_model;
};

RobotDisplay::RobotDisplay(const boost::shared_ptr< urdf::Model >& model)
 : m_model(model)
{
	m_rbdl = boost::make_shared<rbdl_parser::URDF_RBDL_Model>();
	m_rbdl->initFrom(*model);
}

RobotDisplay::~RobotDisplay()
{
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 6)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#endif


	// rviz::Robot class has a non-virtual destructor, but is polymorphic.
	// This is bad design, but not an issue here, because it is assured
	// to be exactly rviz::Robot (see below).
	delete m_robot;

#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 6)
#pragma GCC diagnostic pop
#endif
}

void RobotDisplay::onInitialize()
{
	rviz::Display::onInitialize();

	m_robot = new rviz::Robot(scene_node_, context_, "robot", this);

	m_robot->load(*m_model, true, false);

	RigidBodyDynamics::Math::VectorNd Q = RigidBodyDynamics::Math::VectorNd::Constant(m_rbdl->dof_count, 0);
	RigidBodyDynamics::UpdateKinematics(*m_rbdl, Q, Q, Q);

	m_robot->update(RBDLLinkUpdater(m_rbdl.get()));
}

void RobotDisplay::update(std::vector<double> positions)
{
	double* p = &positions[0];
	RigidBodyDynamics::Math::VectorNd Q = Eigen::Map<Eigen::VectorXd>(p, m_rbdl->dof_count, 1);

	RigidBodyDynamics::Math::VectorNd QZero = RigidBodyDynamics::Math::VectorNd::Constant(m_rbdl->dof_count, 0);
	RigidBodyDynamics::UpdateKinematics(*m_rbdl, Q, QZero, QZero);

	m_robot->update(RBDLLinkUpdater(m_rbdl.get()));
}


