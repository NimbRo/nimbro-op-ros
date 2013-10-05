//Library to manage motion files
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include "motion_file/motionfile.h"

#include <fstream>

#include <yaml-cpp/emitter.h>
#include <yaml-cpp/parser.h>
#include <yaml-cpp/node.h>
#include <boost/concept_check.hpp>

using namespace motionfile;

Motion::Motion()
{
}

FrameJoint::FrameJoint()
 : position(0)
 , velocity(0)
 , effort(1)
{}


void operator >> (const YAML::Node& node, FrameJoint& joint)
{
	node["position"] >> joint.position;
	node["effort"] >> joint.effort;
	node["velocity"] >> joint.velocity;
}


void insertKeyframe(const YAML::Node& node, Motion::KeyframPtr frame, std::vector<std::string>& jointList)
{
	node["duration"] >> frame->duration;
	node["support"] >> frame->support;

	const YAML::Node& joints = node["joints"];
	frame->joints.resize(joints.size());
	YAML::Iterator joint;

	int i = 0;
	for (joint = joints.begin(); joint != joints.end(); ++joint)
	{
		joint.second() >> frame->joints[i++];
		std::string key;
		joint.first() >> key;
		if (std::find(jointList.begin(), jointList.end(), key) == jointList.end())
			jointList.push_back(key);
	}
}

bool Motion::load(std::string name)
{
	std::ifstream inFile;
	inFile.open(name.c_str());
	if (!inFile.is_open())
		return false;

	YAML::Parser fileParser(inFile);
	YAML::Node motionFile;
	fileParser.GetNextDocument(motionFile);
	try
	{
		motionFile["header"]["name"] >> motionName;
		motionFile["header"]["preState"] >> preState;
		motionFile["header"]["playState"] >> playState;
		motionFile["header"]["postState"] >> postState;

		const YAML::Node& keyframes = motionFile["motion"];
		frames.clear();
		frames.resize(keyframes.size());
		jointList.clear();
		for (unsigned i = 0; i < keyframes.size(); i++)
		{
			KeyframPtr newKeyframe(new Keyframe);
			insertKeyframe(keyframes[i], newKeyframe, jointList);
			frames[i] = newKeyframe;
		}

	}
	catch (YAML::ParserException& e)
	{
		return false;
	}

	return true;
}

bool Motion::parse(std::string motion)
{
	std::stringstream inStream(motion);
	YAML::Parser fileParser(inStream);
	YAML::Node motionFile;
	fileParser.GetNextDocument(motionFile);
	try
	{
		motionFile["header"]["name"] >> motionName;
		motionFile["header"]["preState"] >> preState;
		motionFile["header"]["playState"] >> playState;
		motionFile["header"]["postState"] >> postState;

		const YAML::Node& keyframes = motionFile["motion"];
		frames.clear();
		frames.resize(keyframes.size());
		jointList.clear();
		for (unsigned i = 0; i < keyframes.size(); i++)
		{
			KeyframPtr newKeyframe(new Keyframe);
			insertKeyframe(keyframes[i], newKeyframe, jointList);
			frames[i] = newKeyframe;
		}

	}
	catch (YAML::ParserException& e)
	{
		return false;
	}

	return true;
}


bool Motion::save(std::string name)
{

	YAML::Emitter em;
	em << YAML::BeginMap;

	//write header
	em << YAML::Key << "header";
	em << YAML::Value << YAML::BeginMap;

	em << YAML::Key << "name" << YAML::Value << motionName;
	em << YAML::Key << "preState" << YAML::Value << preState;
	em << YAML::Key << "playState" << YAML::Value << playState;
	em << YAML::Key << "postState" << YAML::Value << postState;

	em << YAML::EndMap;

	//write motion
	em << YAML::Key << "motion" << YAML::Value << YAML::BeginSeq;
	for (int i = 0; i < (int)frames.size(); i++)
	{
		em << YAML::BeginMap;
		em << YAML::Key << "duration"<< YAML::Value << frames[i]->duration;
		em << YAML::Key << "support" << YAML::Value << frames[i]->support;
		em << YAML::Key << "joints" << YAML::Value << YAML::BeginMap;
		for (int j = 0; j < (int)frames[i]->joints.size(); j++)
		{
			if (jointList[j].empty())
				continue;
			FrameJoint* joint = &(frames[i]->joints[j]);
			em << YAML::Key << jointList[j];
			em << YAML::Value << YAML::BeginMap;
			em << YAML::Key << "position" << YAML::Value << joint->position;
			em << YAML::Key << "effort" << YAML::Value << joint->effort;
			em << YAML::Key << "velocity" << YAML::Value << joint->velocity;
			em << YAML::EndMap;
		}
		em << YAML::EndMap;

		em << YAML::EndMap;
	}
	em << YAML::EndSeq;

	em << YAML::EndMap;

	std::ofstream out;
	std::string fileName = name + ".yaml";

	out.open(fileName.c_str());
	out << em.c_str() << "\n";
	out.close();
	return true;
}

std::string Motion::dump()
{
	YAML::Emitter em;
	em << YAML::BeginMap;

	//write header
	em << YAML::Key << "header";
	em << YAML::Value << YAML::BeginMap;

	em << YAML::Key << "name" << YAML::Value << motionName;
	em << YAML::Key << "preState" << YAML::Value << preState;
	em << YAML::Key << "playState" << YAML::Value << playState;
	em << YAML::Key << "postState" << YAML::Value << postState;

	em << YAML::EndMap;

	//write motion
	em << YAML::Key << "motion" << YAML::Value << YAML::BeginSeq;
	for (int i = 0; i < (int)frames.size(); i++)
	{
		em << YAML::BeginMap;
		em << YAML::Key << "duration"<< YAML::Value << frames[i]->duration;
		em << YAML::Key << "support" << YAML::Value << frames[i]->support;
		em << YAML::Key << "joints" << YAML::Value << YAML::BeginMap;
		for (int j = 0; j < (int)frames[i]->joints.size(); j++)
		{
			if (jointList[j].empty())
				continue;
			FrameJoint* joint = &(frames[i]->joints[j]);
			em << YAML::Key << jointList[j];
			em << YAML::Value << YAML::BeginMap;
			em << YAML::Key << "position" << YAML::Value << joint->position;
			em << YAML::Key << "effort" << YAML::Value << joint->effort;
			em << YAML::Key << "velocity" << YAML::Value << joint->velocity;
			em << YAML::EndMap;
		}
		em << YAML::EndMap;

		em << YAML::EndMap;
	}
	em << YAML::EndSeq;

	em << YAML::EndMap;

	std::stringstream oss;
	oss << em.c_str() << std::endl;


	return oss.str();
}
