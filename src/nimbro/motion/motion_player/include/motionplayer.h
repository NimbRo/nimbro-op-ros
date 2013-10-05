//Motion Module to play motions from Trajectories
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#ifndef MOTION_PLAYER_H
#define MOTION_PLAYER_H


#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/motionmodule.h>
#include <robotcontrol/FadeTorqueAction.h>


#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"
#include "boost/regex.hpp"

#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>

#include <motion_player/PlayMotion.h>
#include <motion_player/StreamMotion.h>

#include "keyframe_player/KeyframePlayer.h"
#include "motion_file/motionfile.h"


namespace motionplayer
{

class MappedMotion: public motionfile::Motion
{
public:
	std::vector<int> motionToModel;
	robotcontrol::RobotModel::State reqState;
	robotcontrol::RobotModel::State transState;
	robotcontrol::RobotModel::State resState;

	std::vector<kf_player::KeyframePlayer> player;

	void resetPlayer();
	void initPlayer();
};


class MotionPlayer : public robotcontrol::MotionModule
{
public:
	MotionPlayer();
	virtual ~MotionPlayer();

	virtual bool init(robotcontrol::RobotModel* model);
	virtual void step();
	virtual bool isTriggered();



private:

	std::map<std::string, MappedMotion> m_motionNames;
	MappedMotion* m_playingMotion;

	robotcontrol::RobotModel* m_model;

	ros::ServiceServer m_srv_play;
	ros::ServiceServer m_srv_update;
	ros::Subscriber m_sub_joy;

	const double m_sysItTime;

	std::vector<double> m_cmdPositions;
	std::vector<double> m_cmdEffort;

	actionlib::SimpleActionClient<robotcontrol::FadeTorqueAction> m_torqueAct;

	//Predefined states
	robotcontrol::RobotModel::State m_state_prone;
	robotcontrol::RobotModel::State m_state_supine;
	robotcontrol::RobotModel::State m_state_init;
	robotcontrol::RobotModel::State m_state_relaxed;

	void handleJoy();
	bool handlePlay(motion_player::PlayMotionRequest& req, motion_player::PlayMotionResponse& res);
	bool handleUpdateMotion(motion_player::StreamMotionRequest& req, motion_player::StreamMotionResponse& res);

	void play(std::string motion);
	void finished();

	bool m_isInitialized;
	bool isPlaying;
	
	inline void startPlaying(){isPlaying=true;};
	inline void stopPlaying(){isPlaying=false;};

	void writeCommands();


	void initMotions();

	bool loadMotionFiles(const boost::filesystem3::path& dir);
	int findIndex(std::string name);
};





}

#endif