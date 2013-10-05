// Provides plots of the joint angles & efforts
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "plotter/plots/jointstateplot.h"
#include <ros/node_handle.h>
#include <ros/console.h>
#include <qmetatype.h>

namespace plotter
{

JointStatePlot::JointStatePlot(ros::NodeHandle& nh, Plot* parent)
 : Plot("Joint states", parent)
{
	m_positionGroup = new Plot("Positions", this);
	m_velocityGroup = new Plot("Velocities", this);
	m_effortGroup = new Plot("Efforts", this);
	m_goalGroup = new Plot("Goals", this);
	m_commandGroup = new Plot("Commands", this);

	m_sub_js = nh.subscribe("/joint_states", 1, &JointStatePlot::gotData, this);
	m_sub_cmd_js = nh.subscribe("/robotcontrol/sent_joint_cmds", 1, &JointStatePlot::gotCMDData, this);

	qRegisterMetaType<sensor_msgs::JointStateConstPtr>("sensor_msgs::JointStateConstPtr");

	connect(this, SIGNAL(gotData(sensor_msgs::JointStateConstPtr)),
		SLOT(handleData(sensor_msgs::JointStateConstPtr)), Qt::QueuedConnection);

	connect(this, SIGNAL(gotCMDData(sensor_msgs::JointStateConstPtr)),
		SLOT(handleCMDData(sensor_msgs::JointStateConstPtr)), Qt::QueuedConnection);
}

JointStatePlot::~JointStatePlot()
{
}

void JointStatePlot::handleData(const sensor_msgs::JointStateConstPtr& data)
{
	if(paused())
		return;

	for(size_t i = 0; i < data->position.size(); ++i)
	{
		QString name = QString::fromStdString(data->name[i]);
		PlotHash::iterator it = m_plotters.find(name);

		if(it == m_plotters.end())
		{
			JointPlotters plotters;
			plotters.positionPlot = new Plot(name, m_positionGroup);
			plotters.effortPlot = new Plot(name, m_effortGroup);
			plotters.goalPlot = new Plot(name, m_goalGroup);
			plotters.cmdPlot = new Plot(name, m_commandGroup);
			plotters.velocityPlot = new Plot(name, m_velocityGroup);

			it = m_plotters.insert(name, plotters);
		}

		JointPlotters* plotters = &(*it);
		plotters->positionPlot->put(data->header.stamp, data->position[i]);

		if(i < data->effort.size())
			plotters->effortPlot->put(data->header.stamp, data->effort[i]);
	}
}

void JointStatePlot::handleCMDData(const sensor_msgs::JointStateConstPtr& data)
{
	if(paused())
		return;

	for(size_t i = 0; i < data->position.size(); ++i)
	{
		QString name = QString::fromStdString(data->name[i]);
		PlotHash::iterator it = m_plotters.find(name);

		if(it == m_plotters.end())
			continue;

		JointPlotters* plotters = &(*it);
		plotters->goalPlot->put(data->header.stamp, data->position[i]);

		if(i < data->effort.size())
			plotters->cmdPlot->put(data->header.stamp, data->effort[i]);

		if(i < data->velocity.size())
			plotters->velocityPlot->put(data->header.stamp, data->velocity[i]);
	}
}

}
