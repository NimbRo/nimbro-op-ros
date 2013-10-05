// Closed-form Inverse Kinematics for robot legs
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "leg_ik.h"

#include <exception>

#include <ros/console.h>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>

#include <rbdl/Kinematics.h>
#include <nimbro_op_model/dimensions.h>

#include <eigen_conversions/eigen_msg.h>

#define PUBLISH_MARKERS 0

LegIK::LegIK(const boost::shared_ptr<rbdl_parser::URDF_RBDL_Model>& model, const std::string& tip)
 : m_model(model)
 , m_tip(tip)
{
	unsigned int id = model->GetBodyId(tip.c_str());
	if(id == (unsigned int)-1)
		throw std::logic_error((std::string("LegIK: unknown link: ") + tip).c_str());

	Math::VectorNd Q = Math::VectorNd::Constant(model->dof_count, 0);
	RigidBodyDynamics::UpdateKinematicsCustom(*model, &Q, 0, 0);

	// Build a chain of joints to the trunk link
	for(int pid = id; pid != 0; pid = model->lambda[pid])
	{
// 		ROS_INFO("LegIK for %s: link %lu: %s", tip.c_str(), m_links.size(), model->GetBodyName(pid).c_str());
		m_links.push_back(pid);
	}

	/* Links:
	 * link 1: left_ankle_link
	 * link 2: left_shank_link
	 * link 3: left_thigh_link
	 * link 4: left_hip_roll_link
	 * link 5: left_hip_yaw_link
	 */

	// Sanity checks
	if(m_links.size() != 6)
		throw std::logic_error("LegIK: We need exactly 6 joints between the foot frame and trunk.");

	for(size_t i = 0; i < m_links.size(); ++i)
	{
		const Math::SpatialTransform& X_lambda = model->X_lambda[m_links[i]];

		if(!X_lambda.E.isIdentity())
		{
			ROS_ERROR("LegIK(%s): The rotation from %s to %s is not zero.",
				tip.c_str(), model->GetBodyName(model->lambda[m_links[i]]).c_str(),
				model->GetBodyName(m_links[i]).c_str()
			);
// 			ROS_ERROR_STREAM("E: " << X_lambda.E);
		}
	}

	const Math::SpatialTransform trans_foot_ankle = model->X_lambda[m_links[0]];
	if(!trans_foot_ankle.r.isZero())
	{
		ROS_ERROR("LegIK(%s): the foot and ankle frames are not equal. Please check the URDF model!", tip.c_str());
		ROS_ERROR_STREAM("r: " << trans_foot_ankle.r.transpose());
	}

	const Math::SpatialTransform& trans_ankle_knee = model->X_lambda[m_links[1]];
	m_shankLength = trans_ankle_knee.r.norm();
	ROS_INFO("LegIK for %s: shank length: %8.5lfm", tip.c_str(), m_shankLength);

// 	if(!trans_ankle_knee.E.isIdentity())
// 	{
// 		ROS_ERROR("LegIK(%s): The rotation from knee to hip is not zero.", tip.c_str());
// 		ROS_ERROR_STREAM("E: " << trans_ankle_knee.E);
// 	}

	const Math::SpatialTransform& trans_knee_hip = model->X_lambda[m_links[2]];
	m_thighLength = trans_knee_hip.r.norm();
	ROS_INFO("LegIK for %s: thigh length: %8.5lfm", tip.c_str(), m_thighLength);

// 	const Math::SpatialTransform trans_hip_roll_yaw = model->X_lambda[m_links[4]];
// 	if(!trans_hip_roll_yaw.r.isZero())
// 	{
// 		ROS_ERROR("LegIK(%s): the hip roll and yaw frames are not equal. Please check the URDF model!", tip.c_str());
// 		ROS_ERROR_STREAM("r: " << trans_hip_roll_yaw.r.transpose());
// 	}

	m_hipTransform = model->X_base[m_links[5]];
	ROS_INFO_STREAM("LegIK(" << tip << "): hip offset: " << m_hipTransform.r.transpose());

	m_hipTransform.r.z() += model->X_lambda[m_links[4]].r.z() + model->X_lambda[m_links[3]].r.z();

	ROS_INFO_STREAM("LegIK(" << tip << "): hip offset: " << m_hipTransform.r.transpose());

	m_hip_yaw_pitch_offset << m_model->X_lambda[m_links[4]].r.x() + m_model->X_lambda[m_links[3]].r.x(), 0.0, 0.0;

	// Some pre-calculated stuff for the IK
	m_shankLength2 = m_shankLength * m_shankLength;
	m_thighLength2 = m_thighLength * m_thighLength;
	m_lengthDiv = 2.0 * m_shankLength * m_thighLength;

	if(fabs(m_thighLength - m_shankLength) > 1e-5)
	{
		throw std::logic_error("LegIK: FIXME: Thigh and shank links need to have the same length at the moment");
	}

	if(tip == "left_foot_link")
		m_sign = 1;
	else
		m_sign = -1;

#if PUBLISH_MARKERS
	m_pub_markers = ros::NodeHandle("~").advertise<visualization_msgs::MarkerArray>("legik/markers", 1);
#endif
}

LegIK::~LegIK()
{

}

inline void drawArrow(visualization_msgs::MarkerArray* array, const std::string& ns, const std::string& frame, const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
	visualization_msgs::Marker marker;
	marker.ns = ns;
	marker.id = array->markers.size();
	marker.type = visualization_msgs::Marker::ARROW;
	marker.points.resize(2);
	tf::pointEigenToMsg(from, marker.points[0]);
	tf::pointEigenToMsg(to, marker.points[1]);
	marker.scale.x = 0.005;
	marker.scale.y = 0.025;
	marker.scale.z = 0;

	marker.action = visualization_msgs::Marker::ADD;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time::now();

	array->markers.push_back(marker);
}

bool LegIK::doIK(JointVec* q, const Eigen::Vector3d& footLocation, const Eigen::Matrix3d& footRot)
{
	Eigen::Vector3d goal = m_hipTransform.E * (footLocation - m_hipTransform.r);

	// Rotation semantics: All rotations convert from local coordinates to
	// global coordinates.

	// The general idea is to treat the shank-knee-thigh assembly as a prismatic
	// joint. The knee angle can then be calculated in a last step from the
	// distance between ankle and hip.

	// 1) Determine the roll angle at the foot
	Eigen::Vector3d goal_foot = footRot.transpose() * (-goal);

	if(goal_foot.z() == 0)
		return false;

	double roll_angle = atan(goal_foot.y() / goal_foot.z());
	Eigen::Matrix3d rot_ankle_roll;
	rot_ankle_roll = Eigen::AngleAxisd(roll_angle, Eigen::Vector3d::UnitX());

	// Eliminate the roll from the following equations
	goal_foot = rot_ankle_roll * goal_foot;

	// Okay, the foot is in a fixed pose and the ankle roll angle is already
	// determined. This means our hip can move on a plane defined by
	// the ankle pitch axis (as the normal).

	// In particular, the hip roll axis needs to lie completely inside the
	// plane. We can use that to calculate the hip yaw.

	// The plane normal
	Eigen::Vector3d normal = footRot * rot_ankle_roll.transpose() * Eigen::Vector3d::UnitY();

	// We are only interested in the direction of the intersection (and we know
	// one exists as we rotated the plane in step 1) to that effect)
	Eigen::Vector3d intersection = normal.cross(Eigen::Vector3d::UnitZ());

	if(intersection.x() == 0)
		return false;

	// We need to rotate the X axis onto the intersection.
	double yaw_angle = atan2(intersection.y(), intersection.x());

	Eigen::Matrix3d rot_yaw;
	rot_yaw = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

	// Determine the location of the intersection hip_roll/pitch in foot coordinates
	Eigen::Vector3d pitch_goal_foot = rot_ankle_roll * footRot.transpose() * (-goal + rot_yaw * m_hip_yaw_pitch_offset);

	if(pitch_goal_foot.z() == 0)
		return false;

	double pitch_angle = atan(-pitch_goal_foot.x() / pitch_goal_foot.z());

	Eigen::Matrix3d rot_ankle_pitch;
	rot_ankle_pitch = Eigen::AngleAxisd(pitch_angle, Eigen::Vector3d::UnitY());

	// Determine missing angles in the hip
	Eigen::Vector3d euler = (footRot * rot_ankle_roll.transpose() * rot_ankle_pitch.transpose()).eulerAngles(2, 0, 1);
	double hip_pitch_angle = euler(2);
	double hip_roll_angle = euler(1);


	// Now we can replace the prismatic joint with the real knee
	double length = pitch_goal_foot.norm();

	// Calculate the angle between the shank and thigh links (Law of cosines)
	double knee_len = (m_shankLength2 + m_thighLength2 - length*length) / m_lengthDiv;
	double knee_angle = acos(knee_len);
	if(isnan(knee_angle))
		return false; // Goal is too far away

#if PUBLISH_MARKERS
	visualization_msgs::MarkerArray markers;
	drawArrow(&markers, m_tip, "/trunk_link", Eigen::Vector3d::Zero(), footLocation);
	drawArrow(&markers, m_tip, "/trunk_link", footLocation, footLocation + footRot * Eigen::Vector3d(0.2, 0.0, 0.0));
	drawArrow(&markers, m_tip, "/trunk_link", m_hipTransform.r, m_hipTransform.r + m_hip_yaw_pitch_offset);

	Eigen::Vector3d boxLoc(nimbro_op_model::FOOT_CENTER_OFFSET_X, m_sign * nimbro_op_model::FOOT_CENTER_OFFSET_Y_LEFT, -nimbro_op_model::ANKLE_Z_HEIGHT/2.0);
	boxLoc = footLocation + footRot * boxLoc;

	visualization_msgs::Marker box;
	box.ns = m_tip;
	box.id = markers.markers.size();
	box.type = visualization_msgs::Marker::CUBE;
	box.scale.x = nimbro_op_model::FOOT_LENGTH;
	box.scale.y = nimbro_op_model::FOOT_WIDTH;
	box.scale.z = nimbro_op_model::ANKLE_Z_HEIGHT;

	tf::pointEigenToMsg(boxLoc, box.pose.position);
	tf::quaternionEigenToMsg(Eigen::Quaterniond(footRot), box.pose.orientation);

	box.action = visualization_msgs::Marker::ADD;
	box.color.r = 0.0;
	box.color.g = 0.0;
	box.color.b = 1.0;
	box.color.a = 0.5;

	box.header.frame_id = "/trunk_link";
	box.header.stamp = ros::Time::now();
	markers.markers.push_back(box);


	m_pub_markers.publish(markers);
#endif



	// FIXME: The correction for the knee angle in the pitch joints below
	//  is only correct if m_tighLength == m_shankLength.
	//  If you fix this, please remove the check in the constructor above.

	// Ankle roll
	(*q)[0] = roll_angle;

	// Ankle pitch
	(*q)[1] = pitch_angle - asin(sin(knee_angle) / length * m_shankLength);

	// Knee pitch
	(*q)[2] = M_PI - knee_angle;

	// Hip pitch
	(*q)[3] = hip_pitch_angle - asin(sin(knee_angle) / length * m_thighLength);

	// Hip roll
	(*q)[4] = hip_roll_angle;

	// Hip yaw
	(*q)[5] = yaw_angle;

	return true;
}

bool LegIK::doIK(boost::function<void(int, double)> cb, const Eigen::Vector3d& footLocation, const Eigen::Matrix3d& footRot)
{
	JointVec vec;
	if(!doIK(&vec, footLocation, footRot))
		return false;

	for(int i = 0; i < JointVec::RowsAtCompileTime; ++i)
		cb(m_links[i]-1, vec[i]);

	return true;
}

