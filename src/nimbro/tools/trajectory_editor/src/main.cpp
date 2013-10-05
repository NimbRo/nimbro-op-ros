// Trajectory editor
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <QtGui/QApplication>
#include <ros/init.h>

#include "mainwindow.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "trajectory_editor");
	QApplication app(argc, argv);

	MainWindow win;
	win.show();

	return app.exec();
}
