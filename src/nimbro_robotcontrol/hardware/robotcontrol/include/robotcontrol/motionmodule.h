// Interface for motion plugins
// Author: Sebastian Schüller

#ifndef MOTIONMODULE_H
#define MOTIONMODULE_H

namespace robotcontrol
{
class RobotModel;

/**
 * @brief MotionModule abstract base class
 **/
class MotionModule
{
public:
	virtual ~MotionModule();

	/**
	 * @brief Initialize Plugin
	 *
	 * @param model The RobotModel to operate on
	 * @return true on success
	 **/
	virtual bool init(RobotModel* model);

	/**
	 * @brief Activation
	 *
	 * Determine whether this module should be activated right now.
	 * The default implementation returns true.
	 **/
	virtual bool isTriggered();

	/**
	 * @brief Calculate next positions
	 **/
	virtual void step() = 0;

	virtual void publishTransforms();

	inline RobotModel* model()
	{ return m_model; }
protected:
	void setJointCommand(int index, double pos, double effort);
private:
	RobotModel* m_model;
};

}

#endif