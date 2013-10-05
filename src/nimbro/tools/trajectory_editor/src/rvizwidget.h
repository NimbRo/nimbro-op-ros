// Incorporates a RViz display
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef RVIZWIDGET_H
#define RVIZWIDGET_H

#include <rviz/render_panel.h>

#include <ros/publisher.h>

#include <sensor_msgs/JointState.h>

#include <urdf/model.h>

namespace rviz
{
	class VisualizationManager;
}

class RobotDisplay;

class RVizWidget : public rviz::RenderPanel
{
Q_OBJECT
public:
	explicit RVizWidget(QWidget* parent = 0);
	virtual ~RVizWidget();

	void initialize(ros::NodeHandle* nh);

	inline boost::shared_ptr<urdf::Model> getModel()
	{return m_model;}

	inline RobotDisplay* getRobot()
	{return m_robot;}
private:
	rviz::VisualizationManager* m_manager;
	RobotDisplay* m_robot;

	boost::shared_ptr<urdf::Model> m_model;

};

#endif
