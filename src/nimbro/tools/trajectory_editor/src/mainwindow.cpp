// Trajectory editor main window
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "mainwindow.h"
#include "rvizwidget.h"

#include "ui_mainwindow.h"

#include <ros/node_handle.h>

MainWindow::MainWindow()
 : QMainWindow()
{
	m_ui = new Ui::MainWindow;
	QWidget* w = new QWidget(this);

	m_ui->setupUi(w);

	ros::NodeHandle nh("~");
	m_ui->rviz->initialize(&nh);

	setCentralWidget(w);

	m_kModel = new KeyframeModel(this);
	m_ui->trajectoryView->setModel(m_kModel);

	m_kModel->initRobot(m_ui->rviz->getRobot());
	m_kModel->initJointDisplay(m_ui->kfBox->widget());
	m_ui->trajectoryView->show();




	connect(m_ui->createFrameButton, SIGNAL(clicked(bool)), m_kModel, SLOT(createMotion()));
	connect(m_ui->addFrameButton, SIGNAL(clicked(bool)), m_kModel, SLOT(addFrame()));
	connect(m_ui->remFrameButton, SIGNAL(clicked(bool)), this, SLOT(handleRemoveButton()));
	connect(m_ui->saveButton, SIGNAL(clicked(bool)), m_kModel, SLOT(save()));
	connect(m_ui->loadButton, SIGNAL(clicked(bool)), m_kModel, SLOT(load()));
	connect(m_ui->trajectoryView, SIGNAL(clicked(QModelIndex)), m_kModel, SLOT(handleIndexChange(QModelIndex)));
	connect(m_ui->updateButton, SIGNAL(clicked(bool)), m_kModel, SLOT(requestUpdate()));
	connect(m_ui->up_n_PlayButton, SIGNAL(clicked(bool)), m_kModel, SLOT(requestPlay()));
	connect(m_ui->playFrameButton, SIGNAL(clicked(bool)), m_kModel, SLOT(handlePlayCurrentFrame()));
}

MainWindow::~MainWindow()
{
	delete m_ui;
}

void MainWindow::handleRemoveButton()
{
	QModelIndex index =  m_ui->trajectoryView->currentIndex();
	m_kModel->removeFrame(index.row());
}

