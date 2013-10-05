// Displays the robot in a pose defined by joint angles
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROBOTDISPLAY_H
#define ROBOTDISPLAY_H

#include <rviz/display.h>

#include <urdf/model.h>

#include <rbdl/rbdl_parser.h>

namespace rviz { class Robot; }

class RobotDisplay : public rviz::Display
{
public:
	RobotDisplay(const boost::shared_ptr<urdf::Model>& model);
	virtual ~RobotDisplay();

	virtual void onInitialize();
	void update(std::vector<double> positions);

	inline boost::shared_ptr<rbdl_parser::URDF_RBDL_Model> getRBDL()
	{return m_rbdl;}
private:
	boost::shared_ptr<urdf::Model> m_model;
	boost::shared_ptr<rbdl_parser::URDF_RBDL_Model> m_rbdl;
	rviz::Robot* m_robot;
};

#endif
