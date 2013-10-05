// URDF to RBDL parser
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <rbdl/rbdl_parser.h>

#include <urdf/model.h>
#include <rbdl/Model.h>
#include <Eigen/Geometry>

#include <ros/console.h>
#include <ros/assert.h>

namespace Math = RigidBodyDynamics::Math;

namespace rbdl_parser
{

// Some basic conversions

static Math::Vector3d toRBDL(const urdf::Vector3& vec)
{
	return Math::Vector3d(vec.x, vec.y, vec.z);
}

static Math::SpatialTransform toRBDL(const urdf::Pose& pose)
{
	double x, y, z, w;
	pose.rotation.getQuaternion(x, y, z, w);

	Math::Matrix3d mat = Eigen::Quaternion<double>(w, x, y, z).toRotationMatrix();

	return Math::SpatialTransform(mat.transpose(), toRBDL(pose.position));
}

static RigidBodyDynamics::Joint toRBDL(const urdf::Joint& joint)
{
	switch(joint.type)
	{
		case urdf::Joint::FIXED:
			return RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFixed);
		case urdf::Joint::CONTINUOUS:
		case urdf::Joint::REVOLUTE:
			return RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, toRBDL(joint.axis));
		default:
			return RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFixed);
	}
}



URDF_RBDL_Model::URDF_RBDL_Model()
{
}

URDF_RBDL_Model::~URDF_RBDL_Model()
{
}

void URDF_RBDL_Model::process(const urdf::Link& link, int parent, const Math::SpatialTransform& parentJointFrame)
{
	RigidBodyDynamics::Body body;

	if(link.inertial)
	{
		const urdf::Inertial& in = *link.inertial;
		Math::Matrix3d I;
		I << in.ixx, in.ixy, in.ixz,
		     in.ixy, in.iyy, in.iyz,
		     in.ixz, in.iyz, in.izz;

		double mass = link.inertial->mass;
		if(mass == 0)
			mass = 0.000001;
		body = RigidBodyDynamics::Body(mass, toRBDL(link.inertial->origin.position), I);
	}
	else
	{
		body.mCenterOfMass = Math::Vector3d::Zero();
		body.mInertia = Math::Matrix3d::Zero();
		body.mMass = 0.000001;
	}

	Math::SpatialTransform jointFrame = Math::SpatialTransform();
	RigidBodyDynamics::Joint joint = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFixed);

	if(link.parent_joint)
	{
		jointFrame = toRBDL(link.parent_joint->parent_to_joint_origin_transform);
		jointFrame.r = jointFrame.r - parentJointFrame.r;
		joint = toRBDL(*link.parent_joint);
	}

	int id = AddBody(parent, jointFrame, joint, body, link.name);

	if(link.parent_joint && link.parent_joint->type != urdf::Joint::FIXED)
		setupJoint(id, *link.parent_joint, false);

	for(size_t i = 0; i < link.child_links.size(); ++i)
	{
		process(*link.child_links[i], id, Math::SpatialTransform());
	}
}

void URDF_RBDL_Model::processReverse(const urdf::Link& link, int parent, const urdf::Link* parentLink, const Math::SpatialTransform& parentJointFrame)
{
	RigidBodyDynamics::Body body;

	// Situation here: In the URDF world, we are the child of link.parent_link.
	//  We want to become the child of parentLink, which is a child of ours in
	//  URDF.

	// Our origin needs to be in the point of the joint from parentLink to us.

	// Determine transform from old origin to the new origin (joint of the parentLink)
	Math::SpatialTransform tf;
	boost::shared_ptr<urdf::Joint> child_joint;

	if(parentLink)
	{
		child_joint = parentLink->parent_joint;
		tf = toRBDL(child_joint->parent_to_joint_origin_transform);
	}
	else
	{
		// Just leave our origin at the child joint
		tf.E = Math::Matrix3d::Identity();
		tf.r = Math::Vector3d::Zero();
	}

	if(link.inertial)
	{
		const urdf::Inertial& in = *link.inertial;
		Math::Matrix3d I;
		I << in.ixx, in.ixy, in.ixz,
		     in.ixy, in.iyy, in.iyz,
		     in.ixz, in.iyz, in.izz;

		Math::Vector3d cog = toRBDL(link.inertial->origin.position);

		// As the coordinate system is not rotated, and the center of mass
		// stays where it is, the inertia tensor does not need to be
		// modified.

		// Transform cog relative to our new origin
		cog = cog - tf.r;

		double mass = link.inertial->mass;
		if(mass == 0)
			mass = 0.000001;
		body = RigidBodyDynamics::Body(mass, cog, I);
	}
	else
	{
		body.mCenterOfMass = Math::Vector3d::Zero();
		body.mInertia = Math::Matrix3d::Zero();
		body.mMass = 0.00001;
	}

	Math::SpatialTransform jointFrame = Math::SpatialTransform();
	RigidBodyDynamics::Joint joint = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFixed);

	if(parentLink && parentLink->parent_joint)
	{
		// Rotate the other way around since we are reversing the joint.
		Math::Matrix3d E = tf.E.transpose();

		// Origin of the joint in parentLink coordinates
		Math::Vector3d r = -parentJointFrame.r;
		jointFrame = Math::SpatialTransform(E, r);

		joint = toRBDL(*parentLink->parent_joint);

		// Flip joint axes
		for(size_t i = 0; i < joint.mDoFCount; ++i)
		{
			joint.mJointAxes[i] = -joint.mJointAxes[i];
		}
	}

	int id = AddBody(parent, jointFrame, joint, body, link.name);
	if(parentLink && parentLink->parent_joint && parentLink->parent_joint->type != urdf::Joint::FIXED)
		setupJoint(id, *parentLink->parent_joint, true);

	for(size_t i = 0; i < link.child_links.size(); ++i)
	{
		const urdf::Link* child = link.child_links[i].get();
		if(child == parentLink)
			continue;

		process(*link.child_links[i], id, tf);
	}

	boost::shared_ptr<urdf::Link> urdfParent = link.getParent();
	if(urdfParent)
	{
		processReverse(*urdfParent, id, &link, tf);
	}
}

bool URDF_RBDL_Model::initFrom(const urdf::Model& model, const std::string& root)
{
	// Reset root mass to zero and give it the proper name
	mBodies[0].mMass = 0;

	// Set gravity to ROS standards
	gravity << 0, 0, -9.81;

	std::vector<boost::shared_ptr<urdf::Link> > links;
	model.getLinks(links);

	boost::shared_ptr<const urdf::Link> oldRoot = model.getRoot();
	boost::shared_ptr<const urdf::Link> newRoot = model.getLink(root);

	if(oldRoot == newRoot || !newRoot)
		process(*oldRoot, 0, Math::SpatialTransform());
	else
		processReverse(*newRoot, 0, 0, Math::SpatialTransform());

	// Rename the root body to our URDF root
	mBodyNameMap.erase("ROOT");
	mBodyNameMap[root] = 0;

	return true;
}

void URDF_RBDL_Model::setupJoint(int index, const urdf::Joint& urdf, bool reverse)
{
	m_jointMap[urdf.name] = index;

	if(index >= (int)m_jointNames.size())
		m_jointNames.resize(index+1);

	m_jointNames[index] = urdf.name;
}

int URDF_RBDL_Model::jointIndex(const std::string& name)
{
	JointMap::const_iterator it = m_jointMap.find(name);
	ROS_ASSERT_MSG(it != m_jointMap.end(), "Unknown joint '%s'", name.c_str());

	return it->second;
}

std::string URDF_RBDL_Model::jointName(int index)
{
	ROS_ASSERT_MSG(index >= 0 && index < (int)m_jointNames.size(),
		"Invalid joint index %d", index
	);

	return m_jointNames[index];
}

}
