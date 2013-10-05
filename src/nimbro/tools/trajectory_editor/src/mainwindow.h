// Trajectory editor main window
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>

#include "keyframemodel.h"

namespace Ui { class MainWindow; }

class MainWindow : public QMainWindow
{
Q_OBJECT
public:
	MainWindow();
	virtual ~MainWindow();

public Q_SLOTS:
	void handleRemoveButton();

private:
	Ui::MainWindow* m_ui;
	KeyframeModel* m_kModel;
};

#endif
