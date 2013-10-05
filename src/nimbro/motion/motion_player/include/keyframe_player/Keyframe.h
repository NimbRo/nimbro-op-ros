#ifndef KEYFRAME_H_
#define KEYFRAME_H_
#include "keyframe_player/Vec2f.h"

namespace kf_player
{

class Keyframe
{
public:
	Keyframe();
	Keyframe(double t, double x, double v);
	Keyframe(double t, double x, double v, double eff);
	~Keyframe(){};

	int type;
	double t;
	double x;
	double v;
	double a;

	double eff;

	enum types
	{
		TYPE_DEFAULT,
		TYPE_AMAX,
		TYPE_VMAX,
		TYPE_UNREACHABLE
	};

	void set(double t, double x, double v);
	Vec2f location();
	void setLocation(Vec2f);
	void relocateBy(Vec2f);

	inline bool operator<(const Keyframe& k) const {return (t < k.t);}
	inline bool operator<=(const Keyframe& k) const {return (t <= k.t);}
	inline bool operator>(const Keyframe& k) const {return (t > k.t);}
	inline bool operator>=(const Keyframe& v) const {return (t >= v.t);}
	inline bool operator==(const Keyframe& k) const {return (t == k.t && x == k.x && v == k.v);}
	inline bool operator!=(const Keyframe& k) const {return (t != k.t || x != k.x || v != k.v);}
};

QDebug operator<<(QDebug dbg, const Keyframe &k);

#endif /* KEYFRAME_H_ */

}