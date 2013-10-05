//Motion Module to play motions from Trajectories
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/package.h>

#include <motion_player/PlayMotion.h>

#include "motionplayer.h"


namespace fs = boost::filesystem;
namespace motionplayer
{

void MappedMotion::resetPlayer()
{
	for (unsigned i = 0; i < player.size(); i++)
		player[i].reset();
}

void MappedMotion::initPlayer()
{
	player.clear();

	for (unsigned i = 0; i < jointList.size(); i++)
		player.push_back(kf_player::KeyframePlayer());

	double t_old = 0;
	for (unsigned i = 0; i < frames.size(); i++)
	{
		double t = frames[i]->duration + t_old;
		t_old = t;

		for (unsigned j = 0; j < jointList.size(); j++)
		{
			double p = frames[i]->joints[j].position;
			double eff = frames[i]->joints[j].effort;
			double vel = frames[i]->joints[j].velocity;
			player[j].addKeyframe(t, p, vel, eff);
		}
	}

// 	for (unsigned i = 0; i < player.size(); i++)
// 		player[i].calculateCommands();
}




MotionPlayer::MotionPlayer()
 : m_sysItTime(0.008) // If someday implemented; read system iterarion time fron System
 , m_torqueAct("/robotcontrol/fade_torque")
 , m_isInitialized(false)
 , isPlaying(false)
{
	ros::NodeHandle nh("~");

	m_srv_play = nh.advertiseService("/motion_player/play", &MotionPlayer::handlePlay, this);
	m_srv_update = nh.advertiseService("/motion_player/update", &MotionPlayer::handleUpdateMotion, this);
// 	m_sub_joy = nh.subscribe("/joy", &MotionPlayer::handleJoy, this);  //TODO: implement joystick handler
}

MotionPlayer::~MotionPlayer()
{

}

bool MotionPlayer::init(robotcontrol::RobotModel* model)
{
	if (! robotcontrol::MotionModule::init(model))
		return false;


	fs::path p((ros::package::getPath("launch") + "/motions"));
	if (! loadMotionFiles(p))
		return false;

	m_model = model;

	m_state_prone   = m_model->registerState("prone");
	m_state_supine  = m_model->registerState("supine");
	m_state_init    = m_model->registerState("init");
	m_state_relaxed = m_model->registerState("relaxed");

	initMotions();
	m_cmdPositions.resize(model->numJoints());
	m_cmdEffort.resize(model->numJoints());

	/*
	 * 	Change the initial Pose of the Robot from standing (zero-positions) to
	 * 	a sitting pose
	 */

	motion_player::PlayMotion initPose;
	initPose.request.name = "INIT_POSE";
	handlePlay(initPose.request, initPose.response);

	return true;
}


void MotionPlayer::initMotions()
{
	std::map<std::string, MappedMotion>::iterator motionIt;
	for (motionIt = m_motionNames.begin(); motionIt != m_motionNames.end(); ++motionIt)
	{
		MappedMotion& motion = motionIt->second;
		std::vector<std::string> jointList = motion.jointList;
		motion.motionToModel.resize(jointList.size());
		for (unsigned i = 0; i < jointList.size(); i++)
		{
			motion.motionToModel[i] = findIndex(jointList[i]);
		}

		motion.reqState = model()->registerState(motion.preState);
		motion.transState = model()->registerState(motion.playState);
		motion.resState = model()->registerState(motion.postState);
		motion.initPlayer();
	}
}

int MotionPlayer::findIndex(std::string name)
{
	for (size_t i = 0; i < model()->numJoints(); i++)
	{
		const boost::shared_ptr<robotcontrol::Joint>& joint = (*m_model)[i];

		if (joint->name == name)
			return i;
	}
	return 0;
}


bool MotionPlayer::isTriggered()
{
	if (isPlaying)
		return true;

	if (model()->state() == m_state_relaxed && m_isInitialized)
	{
		m_isInitialized = false;

		motion_player::PlayMotion init_pose;
		init_pose.request.name = "INIT_POSE";

		handlePlay(init_pose.request, init_pose.response);
	}

	if(model()->state() == m_state_init && ! m_isInitialized)
	{
		m_isInitialized = true;
		std::string initMotionName = "INIT";
		motion_player::PlayMotion initMotion;
		initMotion.request.name = initMotionName;
		handlePlay(initMotion.request, initMotion.response);
	}

	if (model()->state() == m_state_prone || model()->state() == m_state_supine)
	{

		m_isInitialized = false;
		robotcontrol::FadeTorqueGoal goal;
		goal.torque = 1.0;
		m_torqueAct.cancelAllGoals();
		m_torqueAct.sendGoal(goal);

		motion_player::PlayMotion getUp;
		if (model()->state() == m_state_prone)
			getUp.request.name = "PRONE_GETUP";
		else
			getUp.request.name = "SUPINE_GETUP";

		handlePlay(getUp.request, getUp.response);
	}


	return false;
}

void MotionPlayer::handleJoy()
{

}

bool MotionPlayer::handlePlay(motion_player::PlayMotionRequest& req, motion_player::PlayMotionResponse& res)
{
	if(isPlaying)
	{
		ROS_ERROR("Someone requested motion '%s', but I'm still playing", req.name.c_str());
		return false;
	}

	if (m_motionNames.find(req.name) == m_motionNames.end())
	{
		ROS_ERROR("Couldn't find this motion: %s", req.name.c_str());
		return false;
	}

	if (model()->state() != m_motionNames[req.name].reqState)
	{
		ROS_ERROR("Motion %s expected State %s, but current State is %s"
		  , req.name.c_str()
		  , model()->stateLabel(m_motionNames[req.name].reqState).c_str()
		  , model()->stateLabel(model()->state()).c_str()
		);
		return false;
	}
	else
	{
		ROS_INFO("Playing Motion %s", req.name.c_str());
		play(req.name);
		return true;
	}

}

void MotionPlayer::play(std::string motion)
{
	if (isPlaying)
	{
		ROS_ERROR("Another Motion is still playing");
		return;
	}

	m_playingMotion = &m_motionNames[motion];


	for (unsigned i = 0; i < m_playingMotion->player.size(); i++)
	{
		kf_player::KeyframePlayer* currentPlayer = &m_playingMotion->player[i];
		int index = m_playingMotion->motionToModel[i];
		robotcontrol::Joint::Ptr joint = (*m_model)[index];
		currentPlayer->keyframes[0].x = joint->cmd.pos;

		m_playingMotion->player[i].calculateCommands();
	}

	startPlaying();
	m_playingMotion->resetPlayer();
	model()->setState(m_playingMotion->transState);
}

void MotionPlayer::finished()
{
	model()->setState(m_playingMotion->resState);
	m_playingMotion = 0;
	stopPlaying();
}


bool MotionPlayer::loadMotionFiles(const fs::path& dir)
{
	if (!fs::exists(dir) || !fs::is_directory(dir))
		return false;

	fs::directory_iterator end;
	fs::directory_iterator file(dir);

	std::string fName;
	boost::regex suffixEx("\\.yaml$");
	std::string suffix = (".yaml");

	MappedMotion newMotion;

	for (; file != end; ++file)
	{
		if (fs::is_directory(*file))
			continue;

		fName = file->path().leaf().string();

		if (!boost::regex_search(fName, suffixEx))
			continue;

		if (!newMotion.load(file->path().string()))
			return false;

		fName = fName.substr(0, fName.size() - suffix.size());
		std::pair<std::string, MappedMotion> entry(fName, newMotion);
		m_motionNames.insert(entry);
	}


	ROS_INFO("Following Motions were found and loaded:");
	std::map<std::string, MappedMotion>::iterator mIt;
	for (mIt = m_motionNames.begin(); mIt != m_motionNames.end(); ++mIt)
	{
		ROS_INFO("%s", mIt->first.c_str());
	}

	return true;
}

bool MotionPlayer::handleUpdateMotion(motion_player::StreamMotionRequest& req, motion_player::StreamMotionResponse& res)
{
	if (m_motionNames.find(req.name) == m_motionNames.end())
	{
		ROS_ERROR("Tried to Update motion %s, but it wasn't found.", req.name.c_str());
		return false;
	}

	MappedMotion nMotion;

	if (! nMotion.parse(req.motion))
	{
		ROS_ERROR("Failed to parse given motion: %s", req.name.c_str());
		return false;
	}

	std::vector<std::string> jointList = nMotion.jointList;
	nMotion.motionToModel.resize(jointList.size());
	for (unsigned i = 0; i < jointList.size(); i++)
	{
		nMotion.motionToModel[i] = findIndex(jointList[i]);
	}

	nMotion.reqState = model()->registerState(nMotion.preState);
	nMotion.transState = model()->registerState(nMotion.playState);
	nMotion.resState = model()->registerState(nMotion.postState);
	nMotion.initPlayer();

	m_motionNames[req.name] = nMotion;




	ROS_INFO("Updated motion: %s", req.name.c_str());
	return true;
}



void MotionPlayer::step()
{
	//Just for debug
	//ROS_WARN_THROTTLE(0.1, "Playing motion '%s', time is %lf", m_playingMotion->motionName.c_str(), m_playingMotion->player[0].getCurrentState().t);
	kf_player::Keyframe cmdFrame;
	for (unsigned i = 0; i < m_playingMotion->player.size(); i++)
	{

		if (m_playingMotion->player[i].atEnd())
		{
			finished();
			return;
		}
		cmdFrame = m_playingMotion->player[i].step(m_sysItTime);
		int index = m_playingMotion->motionToModel[i];
		m_cmdPositions[index] = cmdFrame.x;
		m_cmdEffort[index] = cmdFrame.eff;
	}

	m_model->resetSupport();
	writeCommands();
}


void MotionPlayer::writeCommands()
{
	for (unsigned i = 0; i < m_cmdPositions.size(); i++)
	{
		robotcontrol::Joint::Ptr joint = (*m_model)[i];
		joint->cmd.setFromPos(ros::Time(), m_cmdPositions[i]);
		joint->cmd.raw = false;
		joint->cmd.effort = m_cmdEffort[i];
	}
}


}

PLUGINLIB_EXPORT_CLASS(motionplayer::MotionPlayer, robotcontrol::MotionModule)