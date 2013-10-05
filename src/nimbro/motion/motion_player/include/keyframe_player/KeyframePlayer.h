#ifndef KEYFRAMEPLAYER_H_
#define KEYFRAMEPLAYER_H_
#include "keyframe_player/Keyframe.h"
#include <QList>

namespace kf_player
{

class KeyframePlayer
{
public:

	double V;
	double A;
	double VX;

	QList<Keyframe> keyframes;
	QList<Keyframe> commands;
	Keyframe currentState;

public:
	KeyframePlayer();
	~KeyframePlayer(){};

	void setA(double A);
	void setV(double V);
	void setVX(double VX);

	void reset();
	void clear();
	bool addKeyframe(double t, double x=0, double v=0);
	bool addKeyframe(double t, double x,  double v, double eff);
	bool addKeyframe(Keyframe kf);
	void calculateCommands();

	bool atEnd();
	Keyframe step(double t);
	Keyframe getCurrentState();
	Keyframe evaluateAt(double t);
	double totalTime();
	double minimumTime();

private:
	void transformState(double a, double t, Keyframe& kf);
};

}

#endif // KEYFRAMEPLAYER_H_
