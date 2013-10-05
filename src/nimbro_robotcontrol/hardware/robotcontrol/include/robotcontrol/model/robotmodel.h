// Model of the robot
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include <vector>
#include <boost/shared_ptr.hpp>

#include <robotcontrol/model/joint.h>
#include <config_server/parameter.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>

namespace urdf {
class Model;
}

namespace robotcontrol
{

class SingleSupportModel;

/**
 * @brief Kinodynamic robot model
 *
 * This class represents the robot with all its links and joints. Since we need
 * to work with multiple dynamics bases, it consists of one SingleSupportModel
 * for each tip in the kinematic tree.
 *
 * Parameters on the config_server:
 *
 * Name             | Meaning
 * ---------------- | ---------------------------
 * useSupportInfo   | Off = Just use the trunk SingleSupportModel for dynamics
 **/
class RobotModel
{
public:
	RobotModel();
	virtual ~RobotModel();

	//! @name Joint level operations
	//@{
	//! Return number of joints
	size_t numJoints() const
	{ return m_joints.size(); }

	//! Access a joint
	inline const boost::shared_ptr<Joint>& operator[](int i)
	{ return m_joints[i]; }

	//! Access a joint
	inline const boost::shared_ptr<Joint>& joint(int i)
	{ return m_joints[i]; }

	//! Get joint index by name
	int jointIndex(const std::string& name);

	//! Find joint by name
	Joint::Ptr getJoint(const std::string& name);

	void setRelaxed(bool relax);
	inline bool isRelaxed() const
	{ return m_relaxed; }
	//@}

	//! @name Initialization
	//@{
	void setModel(const boost::shared_ptr<urdf::Model>& model);
	inline boost::shared_ptr<urdf::Model> urdf()
	{ return m_model; }

	void addJoint(const boost::shared_ptr<Joint>& joint);
	void initTrees();
	//@}

	//! @name Dynamic model
	//@{
	void doInverseDynamics();
	void setSupportCoefficient(const boost::shared_ptr<const urdf::Link>& link, double coeff);
	void resetSupport();

	boost::shared_ptr<SingleSupportModel> supportModel(const std::string& link_name);
	//@}

	//! @name Robot orientation
	//@{
	/**
	 * @brief Set the robot orientation (transform ego_map -> trunk_link)
	 **/
	void setRobotOrientation(const tf::Quaternion& quaternion);
	inline const tf::Quaternion& robotOrientation() const
	{ return m_robotOrientation; }

	/**
	 * @param velocity angular velocity (roll, pitch, yaw)
	 **/
	void setRobotAngularVelocity(const tf::Vector3& velocity);

	/**
	 * @return angular velocity (roll, pitch, yaw)
	 **/
	inline const tf::Vector3 robotAngularVelocity() const
	{ return m_robotAngularVelocity; }

	void setMagneticFieldVector(const Eigen::Vector3d& magnetic);
	inline const Eigen::Vector3d magneticFieldVector() const
	{ return m_magneticFieldVector; }
	//@}

	//! @name Dynamics information
	//@{
	//! Center of mass in trunk_link coordinates
	tf::Vector3 centerOfMass() const;
	
	tf::Stamped<tf::Vector3> zeroMomentPoint() const;

	inline double mass() const
	{ return m_mass; }
	//@}

	void visualizeData(visualization_msgs::MarkerArray* markers);

	void publishTF(bool useMeasurement);

	//! @name Robot state
	//@{
	class State
	{
	friend class RobotModel;
	public:
		State()
		 : m_idx(-1)
		{}

		bool operator==(const State& other) const
		{ return m_idx == other.m_idx; }

		bool operator!=(const State& other) const
		{ return m_idx != other.m_idx; }
	private:
		explicit State(int idx)
		 : m_idx(idx)
		{}

		int m_idx;
	};

	State registerState(const std::string& name);

	inline State state() const
	{ return m_currentState; }
	void setState(State state);

	std::string stateLabel(const State& state) const;
	std::string currentStateLabel() const;
	//@}
private:
	//! The URDF model
	boost::shared_ptr<urdf::Model> m_model;

	//! List of all Joints
	std::vector<boost::shared_ptr<Joint> > m_joints;

	//! List of all support models
	std::vector<boost::shared_ptr<SingleSupportModel> > m_models;

	//! Current robot orientation
	tf::Quaternion m_robotOrientation;
	tf::Vector3 m_robotAngularVelocity;
	Eigen::Vector3d m_magneticFieldVector;

	config_server::Parameter<bool> m_useSupportInformation;
	config_server::Parameter<bool> m_useFeedbackPos;

	ros::Publisher m_pub_plot;
	ros::Time m_pub_plot_lastTime;

	double m_mass;

	tf::TransformBroadcaster m_pub_tf;
	ros::Publisher m_pub_compass;
	ros::Publisher m_pub_compassHeading;

	std::vector<tf::StampedTransform> m_tf_buf;
	std::vector<tf::StampedTransform> m_tf_fixed_buf;
	int m_tf_fixed_counter;

	std::vector<std::string> m_states;
	State m_currentState;
	ros::Publisher m_pub_state;

	bool m_relaxed;

	void doInit(const boost::shared_ptr<const urdf::Link>& link);
};

}

#endif
