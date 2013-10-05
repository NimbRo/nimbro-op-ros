// Plotter plugin
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PLOTTER_H
#define PLOTTER_H

#include <rqt_gui_cpp/plugin.h>
#include <ros/subscriber.h>
#include <plot_msgs/Plot.h>

#include <QtCore/QSettings>
#include <QtCore/QMutex>

class QProgressDialog;

namespace Ui { class Plotter; }

namespace plotter
{
	class PlotModel;
	class PlotFilterModel;
	class PlotIO;

	class Plotter : public rqt_gui_cpp::Plugin
	{
	Q_OBJECT
	public:
		Plotter();
		virtual ~Plotter();

		virtual void initPlugin(qt_gui_cpp::PluginContext& context);
		virtual void shutdownPlugin();

		virtual bool eventFilter(QObject* , QEvent* event);
	public slots:
		void handlePaused(bool checked);
		void updateTreeGeometry();

	signals:
		void plotDataReceived(const plot_msgs::PlotConstPtr& data);
	private slots:
		void handlePlotData(const plot_msgs::PlotConstPtr& data);
		void updateTimeWarp();
		void save();
		void load();
		void ioProgress(double progress);
	private:
		QWidget* m_w;
		Ui::Plotter* m_ui;
		QSettings m_settings;
		PlotModel* m_plotModel;
		PlotFilterModel* m_plotFilter;

		ros::Subscriber m_sub_plot;

		PlotIO* m_io;
		QProgressDialog* m_progressDialog;

		bool m_blocked;
	};
}

#endif
