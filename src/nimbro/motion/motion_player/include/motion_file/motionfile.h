//Library to manage motion files
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#ifndef MOTIONFILE_H
#define MOTIONFILE_H


#include <map>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

namespace motionfile
{

struct FrameJoint
{
	FrameJoint();

	double position;
	double velocity;
	double effort;
};

struct Keyframe
{
	std::vector<FrameJoint> joints;
	double duration;
	std::string support;
};


class Motion
{
public:
	Motion();
	~Motion(){};

	bool load(std::string name);
	bool parse(std::string motion);
	bool save(std::string name);
	std::string dump();

	typedef boost::shared_ptr<Keyframe> KeyframPtr;

//header
	std::string motionName;
	std::string preState;
	std::string playState;
	std::string postState;
	std::vector<std::string> jointList;

//keyframes
	std::vector<KeyframPtr> frames;
};



}

#endif