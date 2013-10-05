// Provides plots of the joint angles & efforts
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "plotter/plot.h"

#include <QtCore/QHash>
#include <sensor_msgs/JointState.h>
#include <ros/subscriber.h>

namespace plotter
{

class JointStatePlot : public Plot
{
Q_OBJECT
public:
	JointStatePlot(ros::NodeHandle& nh, Plot* parent = 0);
	virtual ~JointStatePlot();
Q_SIGNALS:
	void gotData(const sensor_msgs::JointStateConstPtr& data);
	void gotCMDData(const sensor_msgs::JointStateConstPtr& data);
private Q_SLOTS:
	void handleData(const sensor_msgs::JointStateConstPtr& data);
	void handleCMDData(const sensor_msgs::JointStateConstPtr& data);
private:
	struct JointPlotters
	{
		Plot* positionPlot;
		Plot* velocityPlot;
		Plot* effortPlot;
		Plot* goalPlot;
		Plot* cmdPlot;
	};

	Plot* m_positionGroup;
	Plot* m_velocityGroup;
	Plot* m_effortGroup;
	Plot* m_goalGroup;
	Plot* m_commandGroup;

	typedef QHash<QString, JointPlotters> PlotHash;
	PlotHash m_plotters;

	ros::Subscriber m_sub_js;
	ros::Subscriber m_sub_cmd_js;
};

}