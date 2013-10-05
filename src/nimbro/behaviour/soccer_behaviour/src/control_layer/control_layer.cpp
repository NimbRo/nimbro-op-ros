// Behaviour Control Framework - Control Layer
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <soccer_behaviour/control_layer/control_layer.h>
#include <soccer_behaviour/soccer_manager/soccer_manager.h>

#include <soccer_behaviour/control_layer/game_control.h>

#include <field_model/field_model.h>
#include <Eigen/Geometry>

#include <boost/foreach.hpp>

// Namespaces
using namespace std;
using namespace soccerbehaviour;

// Constants
const double ControlLayer::targetBallOffsetX = 0.35;
const double ControlLayer::targetBallOffsetY = 0.13;
const double ControlLayer::maxBallXWeak = 2.00;

inline double picut(double x)
{
	while(x > M_PI)
		x -= 2.0*M_PI;
	while(x < -M_PI)
		x += 2.0*M_PI;

	return x;
}

//
// ControlLayer class
//

// Contructor
ControlLayer::ControlLayer(SoccerManager* M) : BehaviourLayer(M, "ControlLayer"), M(M)
 , m_param_teamNumber("/gc/teamNumber", 0, 1, 80, 0)
 , m_param_robotNumber("/gc/robotNumber", 0, 1, 5, 0)
 , m_param_positiveIsYellow("/gc/positiveIsYellow", true)
 , m_param_positive("/gc/enemyIsRight", true)
{
	// Create sensor and actuator managers
	SM = new ControlLayerSM(this);
	AM = new ControlLayerAM(this);

	// Create child behaviours
	gameControl = new GameControl(this);
	kick = new Kick(this);
	dribble = new Dribble(this);
	goBehindBall = new GoBehindBall(this);
	searchForBall = new SearchForBall(this);
	controlHead = new ControlHead(this);

	// Add inhibitions
	addChainInhibition(gameControl, kick);
	addChainInhibition(kick, dribble);
	addChainInhibition(dribble, goBehindBall);
	addChainInhibition(goBehindBall, searchForBall);
	addInhibition(searchForBall, controlHead);
	addInhibition(gameControl, controlHead);
}

// Destructor
ControlLayer::~ControlLayer()
{
	halt();

	// Delete child behaviours
	delete gameControl;
	delete kick;
	delete dribble;
	delete goBehindBall;
	delete searchForBall;
	delete controlHead;

	// Delete sensor and actuator managers
	delete SM;
	delete AM;
}

// Initialisation function
ret_t ControlLayer::init()
{
	// Return that initialisation was successful
	return RET_OK;
}

void ControlLayer::updateGCInfo()
{
	rcup_game_controller::GCDataConstPtr gc = SM->gameControlData.read();

	if(!gc)
		return;

	BOOST_FOREACH(const rcup_game_controller::GCTeamInfo& team, gc->teams)
	{
		if(team.teamNumber != m_param_teamNumber())
			continue;

		bool own_positive =
			   (m_param_positiveIsYellow() && team.goalDirection == rcup_game_controller::GCTeamInfo::DIR_1)
			|| (!m_param_positiveIsYellow() && team.goalDirection == rcup_game_controller::GCTeamInfo::DIR_0)
		;

		bool enemy_positive = !own_positive;

		if(enemy_positive != m_param_positive())
			m_param_positive.set(enemy_positive);
	}
}

void ControlLayer::findGoalInVision()
{
	soccer_vision::DetectionsConstPtr detections = SM->visionDetections.read();
	robotcontrol::CompassHeadingConstPtr compass = SM->compassHeading.read();
	field_model::FieldModel* field = field_model::FieldModel::getInstance();

	Eigen::Vector2d goalPos(1.0, 0.0);
	double theta = 0.0;

	if(compass)
		theta = field->magneticHeading() - compass->heading;
	else
		ROS_WARN_THROTTLE(0.1, "No compass value, goal calculation will be wrong!");

	double expectedAngle = -theta;

	if(goalSign() < 0)
		expectedAngle = picut(expectedAngle + M_PI);

	// Try to find the goal in the detections
	if(detections)
	{
		std::vector<Eigen::Vector2d> goalPosts;
		Eigen::Vector2d mean = Eigen::Vector2d::Zero();
		double minGoalPostAngle = M_PI;
		double maxGoalPostAngle = -M_PI;

		BOOST_FOREACH(const soccer_vision::ObjectDetection& det, detections->objects)
		{
			if(det.type != field_model::WorldObject::Type_GoalPost)
				continue;

			Eigen::Vector2d pos(det.pose.x, det.pose.y);

			double postAngle = atan2(pos.y(), pos.x());
			if(fabs(picut(postAngle - expectedAngle)) > M_PI/2.0)
				continue;

			goalPosts.push_back(pos);
			mean += pos;

			minGoalPostAngle = std::min(postAngle, minGoalPostAngle);
			maxGoalPostAngle = std::max(postAngle, maxGoalPostAngle);
		}
		
		if(goalPosts.size() > 2)
			ROS_WARN_THROTTLE(0.2, "More than 2 goal posts, result is going to be bad");

		if(goalPosts.size() == 2 && (goalPosts[0] - goalPosts[1]).norm() > 2.0)
		{
			mean /= goalPosts.size();
			m_minGoalPostAngle = minGoalPostAngle;
			m_maxGoalPostAngle = maxGoalPostAngle;
			m_lastGoalPos = mean;
			m_lastGoalPosStamp = detections->header.stamp;
		}
	}
}

Eigen::Vector2d ControlLayer::dribbleTarget(bool& isGoal)
{
	// Constants
	const double compassDribbleTargetExtra = 0.5;

	if(goalIsValid())
	{
// 		ROS_INFO_STREAM_THROTTLE(0.2, "Dribble target: Using goal vision-based target!");
		isGoal = true;
		return m_lastGoalPos;
	}
	else
	{
		isGoal = false;
		bool haveCompass = false;
		double compassTarget = compassGoalTarget(haveCompass);
		if(haveCompass)
		{
// 			ROS_INFO_STREAM_THROTTLE(0.2, "Dribble target: Using compass-based target!");
			return (maxBallXWeak + compassDribbleTargetExtra) * Eigen::Vector2d(cos(compassTarget), -sin(compassTarget));
		}
		else
		{
// 			ROS_INFO_STREAM_THROTTLE(0.2, "Dribble target: Don't have either goal or compass!");
			return Eigen::Vector2d(-0.05, 0.0); // Note: This value should ALWAYS disable the dribble behaviour - if not, change this!
		}
	}
}

Eigen::Vector2d ControlLayer::kickTarget(bool& isGoal)
{
	// Same as dribbleTarget() for now...
	return dribbleTarget(isGoal);
}

double ControlLayer::maxGoalPostAngle()
{return m_maxGoalPostAngle;}

double ControlLayer::minGoalPostAngle()
{return m_minGoalPostAngle;}

bool ControlLayer::goalIsValid()
{
	ros::Duration timeout(3.0);
	if ((ros::Time::now() - m_lastGoalPosStamp) < timeout)
		return true;
	return false;
}

double ControlLayer::compassGoalTarget(bool& haveCompass)
{
	robotcontrol::CompassHeadingConstPtr compass = SM->compassHeading.read();
	field_model::FieldModel* field = field_model::FieldModel::getInstance();
	haveCompass = (bool) compass;

	if(compass)
	{
		double side_offset = m_param_positive() ? 0.0 : M_PI;
		return picut(field->magneticHeading() - compass->heading + side_offset);
	}
	else
		return 0.0;
}

Eigen::Vector2d ControlLayer::goBehindBallTarget()
{
// 	if((ros::Time::now() - m_lastGoalPosStamp) < ros::Duration(1.0))
// 		return m_lastGoalPos;
// 	else
//		return compassGoalTarget();
	return m_lastGoalPos; // TODO: Just temporary because this fn should not be in use as it stands
}

// Update function
void ControlLayer::update()
{
	updateGCInfo();
	findGoalInVision();
}

// Post-execute callback
void ControlLayer::postExecuteCallback()
{
}

void ControlLayer::halt()
{
}
// EOF