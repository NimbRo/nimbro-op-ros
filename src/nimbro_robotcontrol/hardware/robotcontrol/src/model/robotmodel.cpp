// Model of the robot
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/model/singlesupportmodel.h>
#include <robotcontrol/model/joint.h>

#include <rbdl/rbdl_parser.h>
#include <rbdl/treestream.h>
#include <rbdl/Dynamics.h>

#include <plot_msgs/Plot.h>

#include <boost/make_shared.hpp>

#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <robotcontrol/State.h>
#include <robotcontrol/CompassHeading.h>

#define DUMP_TREES 0

namespace Math = RigidBodyDynamics::Math;

namespace robotcontrol
{

inline void rbdlToTF(const Math::SpatialTransform& X, tf::Transform* t)
{
	t->setOrigin(tf::Vector3(X.r.x(), X.r.y(), X.r.z()));
	t->setBasis(tf::Matrix3x3(
		X.E(0, 0), X.E(1, 0), X.E(2, 0),
		X.E(0, 1), X.E(1, 1), X.E(2, 1),
		X.E(0, 2), X.E(1, 2), X.E(2, 2)
	));
}


RobotModel::RobotModel()
 : m_robotOrientation(tf::createIdentityQuaternion())
 , m_useSupportInformation("useSupportInfo", true)
 , m_useFeedbackPos("useFeedbackPos", false)
 , m_currentState(0)
 , m_relaxed(false)
{
	ros::NodeHandle nh("~");
	m_pub_plot = nh.advertise<plot_msgs::Plot>("/plot", 1);
	m_pub_state = nh.advertise<robotcontrol::State>("state", 1, true);

	m_pub_compass = nh.advertise<geometry_msgs::Vector3Stamped>("/compass/field", 1);
	m_pub_compassHeading = nh.advertise<robotcontrol::CompassHeading>("/compass/heading", 1);

	m_magneticFieldVector.setZero();

	setState(registerState("init"));
}

RobotModel::~RobotModel()
{
}

void RobotModel::setModel(const boost::shared_ptr< urdf::Model >& model)
{
	m_model = model;
}

void RobotModel::addJoint(const boost::shared_ptr<Joint>& joint)
{
	m_joints.push_back(joint);
}

/**
 * Initialize the kinodynamic trees (the SingleSupportModels).
 **/
void RobotModel::initTrees()
{
	boost::shared_ptr<const urdf::Link> root = m_model->getRoot();
	boost::shared_ptr<SingleSupportModel> model = boost::make_shared<SingleSupportModel>(this, root);
	model->initFrom(*m_model, root->name);
	model->setCoefficient(0.0);

#if DUMP_TREES
	TreeStream stream(&std::cout);
	stream << *model;
#endif

	m_models.push_back(model);

	doInit(root);

	// Calculate the total mass
	m_mass = 0.0;
	std::vector<boost::shared_ptr<urdf::Link> > links;
	m_model->getLinks(links);
	for(size_t i = 0; i < links.size(); ++i)
	{
		if(links[i]->inertial)
			m_mass += links[i]->inertial->mass;
	}

	// Setup tf transform for /ego_rot
	tf::StampedTransform t_imu;
	t_imu.frame_id_ = "/ego_rot";
	t_imu.child_frame_id_ = "/trunk_link";
	t_imu.setIdentity();
	m_tf_buf.push_back(t_imu);

	// Setup tf transforms for each link
	for(size_t i = 1; i < model->mBodies.size(); ++i)
	{
		tf::StampedTransform t;
		t.frame_id_ = model->GetBodyName(model->lambda[i]);
		t.child_frame_id_ = model->GetBodyName(i);
		t.setIdentity();
		m_tf_buf.push_back(t);
	}

	// Setup fixed transforms
	for(size_t i = 1; i < model->mFixedBodies.size(); ++i)
	{
		tf::StampedTransform t;
		const RigidBodyDynamics::FixedBody& body = model->mFixedBodies[i];
		t.frame_id_ = model->GetBodyName(body.mMovableParent);
		t.child_frame_id_ = model->GetBodyName(model->fixed_body_discriminator + i);

		rbdlToTF(body.mParentTransform, &t);

		m_tf_fixed_buf.push_back(t);
	}
}

/**
 * Helper method for initTrees(). Calls itself recursively on every
 * link in the kinematic tree.
 **/
void RobotModel::doInit(const boost::shared_ptr< const urdf::Link >& link)
{
	if(link->child_links.size() == 0)
	{
		// This is a tip, create a SingleSupportModel
		boost::shared_ptr<SingleSupportModel> model = boost::make_shared<SingleSupportModel>(this, link);
		model->initFrom(*m_model, link->name);

#if DUMP_TREES
		TreeStream stream(&std::cout);
		stream << *model;
#endif

		m_models.push_back(model);
	}
	else
	{
		for(size_t i = 0; i < link->child_links.size(); ++i)
			doInit(link->child_links[i]);
	}
}

/**
 * @param name URDF joint name
 * @return Pointer to Joint, Null pointer if the joint does not exist
 **/
Joint::Ptr RobotModel::getJoint(const std::string& name)
{
	for(size_t i = 0; i < m_joints.size(); ++i)
	{
		if(m_joints[i]->name == name)
			return m_joints[i];
	}

	return Joint::Ptr();
}

int RobotModel::jointIndex(const std::string& name)
{
	for(size_t i = 0; i < m_joints.size(); ++i)
	{
		if(m_joints[i]->name == name)
			return i;
	}

	std::stringstream ss;
	ss << "RobotModel::jointIndex(): Unknown joint '" << name << "'";
	throw std::logic_error(ss.str());
}

/**
 * Do the dynamics calculations.
 **/
void RobotModel::doInverseDynamics()
{
	for(size_t i = 0; i < m_joints.size(); ++i)
		m_joints[i]->feedback.modelTorque = 0;

	SingleSupportModel::DataSource source;
	if(m_useFeedbackPos())
		source = SingleSupportModel::MeasurementData;
	else
		source = SingleSupportModel::CommandData;

	if(m_useSupportInformation())
	{
		for(size_t i = 0; i < m_models.size(); ++i)
		{
			m_models[i]->doInverseDynamics(true, source);
		}
	}
	else
	{
		m_models[0]->doInverseDynamics(false, source);
	}

	m_models[0]->computeCOM();
}

/**
 * @param link_name Name of the tip link used as base of the support model
 *   (e.g. right_foot_link)
 **/
boost::shared_ptr<SingleSupportModel> RobotModel::supportModel(const std::string& link_name)
{
	for(size_t i = 0; i < m_models.size(); ++i)
	{
		if(m_models[i]->link()->name == link_name)
			return m_models[i];
	}

	return boost::shared_ptr<SingleSupportModel>();
}

/**
 * Set a support coefficient. The coefficients are used as weights in the mixing
 * calculations (see SingleSupportModel::setCoefficient()).
 *
 * @param link The tip link used as base of the support model
 *   (e.g. right_foot_link)
 * @param coeff Support coefficient (positive or zero)
 **/
void RobotModel::setSupportCoefficient(const boost::shared_ptr<const urdf::Link>& link, double coeff)
{
	boost::shared_ptr<SingleSupportModel> model;
	for(size_t i = 0; i < m_models.size(); ++i)
	{
		if(m_models[i]->link() == link)
		{
			model = m_models[i];
			break;
		}
	}

	if(!model)
	{
		ROS_ERROR("setSupportCoefficient called with non-tip link '%s'", link->name.c_str());
		assert(0);
	}

	model->setCoefficient(coeff);

	// We need to normalize the coefficients to a sum of 1.
	double total = 0;
	for(size_t i = 0; i < m_models.size(); ++i)
		total += m_models[i]->coefficient();

	for(size_t i = 0; i < m_models.size(); ++i)
		m_models[i]->normalize(total);

	// Publish plots of the support coefficients
	ros::Time now = ros::Time::now();

	if(total == 0)
		ROS_WARN_THROTTLE(1.0, "setSupportCoefficient: %s with %f => total 0", link->name.c_str(), coeff);
}

void RobotModel::setRobotOrientation(const tf::Quaternion& quaternion)
{
	m_robotOrientation = quaternion;
}

void RobotModel::setRobotAngularVelocity(const tf::Vector3& velocity)
{
	m_robotAngularVelocity = velocity;
}

void RobotModel::setMagneticFieldVector(const Eigen::Vector3d& magnetic)
{
	m_magneticFieldVector = magnetic;
}

tf::Vector3 RobotModel::centerOfMass() const
{
	RigidBodyDynamics::Math::Vector3d com = m_models[0]->centerOfMass();

	return tf::Vector3(com.x(), com.y(), com.z());
}

/**
 * @return ZMP calculated using the SingleSupportModel with the highest
 *   coefficient
 **/
tf::Stamped<tf::Vector3> RobotModel::zeroMomentPoint() const
{
	int idx_max = 0;
	for(size_t i = 1; i < m_models.size(); ++i)
	{
		if(m_models[i]->coefficient() > m_models[idx_max]->coefficient())
			idx_max = i;
	}

// 	ROS_WARN("Using model %d to calculate ZMP", idx_max);

	RigidBodyDynamics::Math::Vector3d zmp = m_models[idx_max]->zeroMomentPoint();

// 	ROS_WARN("Got ZMP (% 7.3f % 7.3f % 7.3f", zmp.x(), zmp.y(), zmp.z());

	return tf::Stamped<tf::Vector3>(
		tf::Vector3(zmp.x(), zmp.y(), zmp.z()),
		ros::Time::now(),
		"/" + m_models[idx_max]->link()->name
	);
}

void RobotModel::visualizeData(visualization_msgs::MarkerArray* markers)
{
	ros::Time now = ros::Time::now();

	tf::Vector3 com = centerOfMass();
	visualization_msgs::Marker com_marker;
	com_marker.header.stamp = now;
	com_marker.header.frame_id = "/trunk_link";
	com_marker.ns = "robotcontrol";
	com_marker.id = 0;
	com_marker.type = visualization_msgs::Marker::SPHERE;
	com_marker.action = visualization_msgs::Marker::ADD;
	com_marker.pose.position.x = com.x();
	com_marker.pose.position.y = com.y();
	com_marker.pose.position.z = com.z();
	com_marker.pose.orientation.w = 1;
	com_marker.pose.orientation.x = 0;
	com_marker.pose.orientation.y = 0;
	com_marker.pose.orientation.z = 0;
	com_marker.scale.x = 0.05;
	com_marker.scale.y = 0.05;
	com_marker.scale.z = 0.05;
	com_marker.color.a = 1.0;
	com_marker.color.r = 1.0;
	com_marker.color.g = 0.0;
	com_marker.color.b = 0.0;

	markers->markers.push_back(com_marker);

	visualization_msgs::Marker magnetic_marker;
	magnetic_marker.header.stamp = now;
	magnetic_marker.header.frame_id = "/trunk_link";
	magnetic_marker.ns = "robotcontrol";
	magnetic_marker.id = 1;
	magnetic_marker.type = visualization_msgs::Marker::ARROW;
	magnetic_marker.action = visualization_msgs::Marker::ADD;
	magnetic_marker.points.resize(2);
	tf::pointEigenToMsg(Eigen::Vector3d::Zero(), magnetic_marker.points[0]);
	tf::pointEigenToMsg(m_magneticFieldVector, magnetic_marker.points[1]);
	magnetic_marker.scale.x = 0.01;
	magnetic_marker.scale.y = 0.02;
	magnetic_marker.scale.z = 0.0;
	magnetic_marker.color.a = 1.0;
	magnetic_marker.color.r = 0.0;
	magnetic_marker.color.g = 1.0;
	magnetic_marker.color.b = 0.0;

	markers->markers.push_back(magnetic_marker);

	// Plot support coefficients
	plot_msgs::Plot plot;
	plot.header.stamp = now;
	plot.points.resize(m_models.size());
	for(size_t i = 0; i < m_models.size(); ++i)
	{
		plot.points[i].name = "Support/" + m_models[i]->link()->name;
		plot.points[i].value = m_models[i]->coefficient();
	}

	m_pub_plot.publish(plot);
	m_pub_plot_lastTime = now;
}

void RobotModel::publishTF(bool useMeasurement)
{
	boost::shared_ptr<SingleSupportModel> model = m_models[0];
	ros::Time now = ros::Time::now();

	if(useMeasurement)
		model->updateKinematics(SingleSupportModel::MeasurementData);
	else
		model->updateKinematics(SingleSupportModel::CommandData);

	m_tf_buf[0].stamp_ = model->joint(1)->feedback.stamp;
	m_tf_buf[0].setRotation(robotOrientation());

	for(size_t i = 1; i < model->mBodies.size(); ++i)
	{
		ROS_ASSERT(i < m_tf_buf.size());
		tf::StampedTransform* t = &m_tf_buf[i];

		rbdlToTF(model->X_lambda[i], t);
		t->stamp_ = model->joint(i-1)->feedback.stamp;
	}

	m_pub_tf.sendTransform(m_tf_buf);

	ros::Time fixedTime = now + ros::Duration(0.5);
	for(size_t i = 0; i < m_tf_fixed_buf.size(); ++i)
		m_tf_fixed_buf[i].stamp_ = fixedTime;

	m_pub_tf.sendTransform(m_tf_fixed_buf);

	// Publish compass vector
	geometry_msgs::Vector3Stamped msg;
	msg.header.stamp = now;
	msg.header.frame_id = "/trunk_link";
	tf::vectorEigenToMsg(m_magneticFieldVector, msg.vector);
	m_pub_compass.publish(msg);

	robotcontrol::CompassHeading hmsg;
	Eigen::Quaterniond qrot;
	tf::quaternionTFToEigen(m_robotOrientation, qrot);
	Eigen::Vector3d tilt_compensated = qrot * m_magneticFieldVector;
	double angle = atan2(tilt_compensated.y(), tilt_compensated.x());

	hmsg.stamp = now;
	hmsg.heading = angle;
	m_pub_compassHeading.publish(hmsg);
}

RobotModel::State RobotModel::registerState(const std::string& name)
{
	std::vector<std::string>::iterator it = std::find(m_states.begin(), m_states.end(), name);
	if(it == m_states.end())
	{
		State state(m_states.size());
		m_states.push_back(name);
		return state;
	}

	State state(it - m_states.begin());

	return state;
}

void RobotModel::setState(RobotModel::State state)
{
	ROS_ASSERT(state.m_idx >= 0 && state.m_idx < (int)m_states.size());

	if(state != m_currentState)
	{
		robotcontrol::State msg;
		msg.id = state.m_idx;
		msg.label = m_states[state.m_idx];

		m_pub_state.publish(msg);
	}

	m_currentState = state;
}

std::string RobotModel::stateLabel(const robotcontrol::RobotModel::State& state) const
{
	ROS_ASSERT(state.m_idx >= 0 && state.m_idx < (int)m_states.size());

	return m_states[state.m_idx];
}

std::string RobotModel::currentStateLabel() const
{
	return stateLabel(state());
}

void RobotModel::setRelaxed(bool relax)
{
	m_relaxed = relax;
}

void RobotModel::resetSupport()
{
	for(size_t i = 0; i < m_models.size(); ++i)
		setSupportCoefficient(m_models[i]->link(), 0.0);
}

}
