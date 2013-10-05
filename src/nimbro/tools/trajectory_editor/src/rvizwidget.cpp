// Incorporates a RViz display
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "rvizwidget.h"
#include "robotdisplay.h"

#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>

#include <sensor_msgs/JointState.h>

RVizWidget::RVizWidget(QWidget* parent)
 : RenderPanel(parent)
{
	m_manager = new rviz::VisualizationManager(this);

	RenderPanel::initialize(m_manager->getSceneManager(), m_manager);

	m_manager->initialize();
	m_manager->startUpdate();
}

RVizWidget::~RVizWidget()
{
	delete m_manager;
}

void RVizWidget::initialize(ros::NodeHandle* nh)
{
	m_model = boost::make_shared<urdf::Model>();

	m_model->initParam("/robot_description");
	m_robot = new RobotDisplay(m_model);

	m_manager->addDisplay(m_robot, true);
}
