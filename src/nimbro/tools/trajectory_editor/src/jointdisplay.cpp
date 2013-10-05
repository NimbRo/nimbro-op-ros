//Displays joints of loaded frame
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include "jointdisplay.h"
#include <QGridLayout>

#include <QtCore/QDebug>
#include <math.h>

std::vector<std::string> stripList(std::vector<std::string> list)
{
	std::vector<std::string> strippedList;
	for (unsigned i = 0; i < list.size(); i++)
	{
		if (not list[i].empty())
			strippedList.push_back(list[i]);
	}
	return strippedList;
}





const double angToTic = 1000.0 / M_PI;
const double ticToAng = M_PI / 1000.0;

JointManager::JointManager(QWidget* controlWidget, const std::vector< std::string >& modelJointList)
 : QObject(controlWidget)
 , m_currentFrame()
{
	m_controlWidget = controlWidget;
	QGridLayout* layout = new QGridLayout(m_controlWidget);
	m_controlWidget->setLayout(layout);


	m_jointList = modelJointList;
	initFrames(stripList(modelJointList));

	connect(this, SIGNAL(sliderChanged()), this, SLOT(handleSliderChange()));
	connect(this, SIGNAL(positionSpinChanged()), this, SLOT(handlePosSpinChange()));
	connect(this, SIGNAL(effortSpinChanged()), this, SLOT(handleEffSpinChange()));
	connect(this, SIGNAL(velocityChanged()), this, SLOT(handleVelChange()));
}

void JointManager::updateJointList(const std::vector< std::string >& jointList)
{
	m_jointList = jointList;
}

void JointManager::setFrame(JointManager::KeyframPtr frame)
{
	m_currentFrame = frame;
	updateFrame();
}

void JointManager::unsetFrame()
{
	m_currentFrame = KeyframPtr();
}

void JointManager::initFrames(std::vector<std::string> jointList)
{
	QGridLayout* layout = qobject_cast< QGridLayout* >(m_controlWidget->layout());
	if (!layout)
		return;

	for (unsigned i = 0; i < m_controlList.size(); i++)
	{
		delete m_controlList[i].positionLabel;
		delete m_controlList[i].positionSlider;
		delete m_controlList[i].positionSpin;
		delete m_controlList[i].effortSpin;
		delete m_controlList[i].velSpin;
	}
	m_controlList.clear();


	m_durationSlider = new QSlider(Qt::Horizontal, m_controlWidget);
	m_durationSlider->setMinimum(0);
	m_durationSlider->setMaximum(10000);
	m_durationSlider->setValue(0);

	m_durationSpin = new QDoubleSpinBox(m_controlWidget);
	m_durationSpin->setMinimum(0);
	m_durationSpin->setMaximum(100);
	m_durationSpin->setSingleStep(0.01);
	m_durationSpin->setValue(0);

	layout->addWidget(new QLabel("duration"), DURATION, 0);
	layout->addWidget(m_durationSlider, DURATION, 1);
	layout->addWidget(m_durationSpin, DURATION, 2);

	connect(m_durationSlider, SIGNAL(valueChanged(int)), this, SLOT(handleDurSliderChange()));
	connect(m_durationSpin, SIGNAL(valueChanged(double)), this, SLOT(handleDurSpinChange()));


	layout->addWidget(new QLabel("Joints"), CONTROL_TITLE, 0);
	layout->addWidget(new QLabel("Position"), CONTROL_TITLE, 1);
	layout->addWidget(new QLabel("Effort"), CONTROL_TITLE, 3);
	layout->addWidget(new QLabel("Velocity"),CONTROL_TITLE, 4);

	m_globalEffortSpin = new QDoubleSpinBox(m_controlWidget);
	m_globalEffortSpin->setMinimum(0);
	m_globalEffortSpin->setMaximum(1);
	m_globalEffortSpin->setSingleStep(0.1);
	m_globalEffortSpin->setValue(0.5);

	layout->addWidget(new QLabel("Global Effort"), GLOBAL_EFFORT, 0);
	layout->addWidget(m_globalEffortSpin, GLOBAL_EFFORT, 1);

	connect(m_globalEffortSpin, SIGNAL(valueChanged(double)), this, SLOT(handleGlobalEffort()));

	m_controlList.resize(jointList.size());
	for (unsigned i = 0; i < jointList.size(); i++)
	{
		Controls c;
		c.positionLabel = new QLabel(QString::fromStdString(jointList[i]), m_controlWidget);
		c.positionSlider = new QSlider(Qt::Horizontal, m_controlWidget);
		c.positionSpin = new QDoubleSpinBox(m_controlWidget);
		c.effortSpin = new QDoubleSpinBox(m_controlWidget);
		c.velSpin = new QDoubleSpinBox(m_controlWidget);

		c.positionSlider->setMinimum(-1000);
		c.positionSlider->setMaximum(1000);
		c.positionSlider->setValue(0);

		c.positionSpin->setMinimum(-1000 * ticToAng);
		c.positionSpin->setMaximum(1000 * ticToAng);
		c.positionSpin->setValue(0);
		c.positionSpin->setSingleStep(0.01);

		c.effortSpin->setMaximum(1);
		c.effortSpin->setMinimum(0);
		c.effortSpin->setValue(0);
		c.effortSpin->setSingleStep(0.01);

		c.velSpin->setMaximum(6);
		c.velSpin->setMinimum(-6);
		c.velSpin->setValue(0);
		c.velSpin->setSingleStep(0.1);


		connect(c.positionSlider, SIGNAL(valueChanged(int)), this, SIGNAL(sliderChanged()));
		connect(c.positionSpin, SIGNAL(valueChanged(double)), this, SIGNAL(positionSpinChanged()));
		connect(c.effortSpin, SIGNAL(valueChanged(double)), this, SIGNAL(effortSpinChanged()));
		connect(c.velSpin, SIGNAL(valueChanged(double)), this, SIGNAL(velocityChanged()));

		layout->addWidget(c.positionLabel,CONTROLS + i, 0);
		layout->addWidget(c.positionSlider,CONTROLS + i, 1);
		layout->addWidget(c.positionSpin,CONTROLS + i, 2);
		layout->addWidget(c.effortSpin,CONTROLS + i, 3);
		layout->addWidget(c.velSpin, CONTROLS + i, 4);

		m_controlList[i] = c;
	}
}




void JointManager::updateFrame()
{
	m_durationSlider->blockSignals(true);
	m_durationSpin->blockSignals(true);
	m_durationSpin->setValue(m_currentFrame->duration);
	m_durationSlider->setValue(m_currentFrame->duration * 100);
	m_durationSlider->blockSignals(false);
	m_durationSpin->blockSignals(false);

	for (unsigned i = 0; i < m_controlList.size(); i++)
	{
		Controls c = m_controlList[i];

		std::string name = c.positionLabel->text().toStdString();
		int index = indexToName(name);
		if (index < 0)
			continue;

		double currentPos = m_currentFrame->joints[index].position;
		double currentEff = m_currentFrame->joints[index].effort;
		double currentVel = m_currentFrame->joints[index].velocity;


		disconnect(this, SIGNAL(effortSpinChanged()), this, SLOT(handleEffSpinChange()));
		disconnect(this, SIGNAL(positionSpinChanged()), this, SLOT(handlePosSpinChange()));
		disconnect(this, SIGNAL(sliderChanged()), this, SLOT(handleSliderChange()));
		disconnect(this, SIGNAL(velocityChanged()), this, SLOT(handleVelChange()));
		c.positionSlider->setValue(currentPos * angToTic);
		c.positionSpin->setValue(currentPos);
		c.effortSpin->setValue(currentEff);
		c.velSpin->setValue(currentVel);
		connect(this, SIGNAL(sliderChanged()), this, SLOT(handleSliderChange()));
		connect(this, SIGNAL(positionSpinChanged()), this, SLOT(handlePosSpinChange()));
		connect(this, SIGNAL(effortSpinChanged()), this, SLOT(handleEffSpinChange()));
		connect(this, SIGNAL(velocityChanged()), this, SLOT(handleVelChange()));

	}
}

void JointManager::handleEffSpinChange()
{
	if (!m_currentFrame)
		return;

	for (unsigned i = 0; i < m_controlList.size(); i++)
	{
		std::string name = m_controlList[i].positionLabel->text().toStdString();
		int index = indexToName(name);
		if (index < 0)
			continue;
		m_currentFrame->joints[index].effort = m_controlList[i].effortSpin->value();
	}
}

void JointManager::handlePosSpinChange()
{
	if (!m_currentFrame)
		return;

	for (unsigned i = 0; i < m_controlList.size(); i++)
	{
		Controls c = m_controlList[i];
		c.positionSlider->blockSignals(true);
		c.positionSlider->setValue(c.positionSpin->value() * angToTic);
		c.positionSlider->blockSignals(false);

		std::string name = c.positionLabel->text().toStdString();
		int index = indexToName(name);
		if (index < 0)
			continue;

		m_currentFrame->joints[index].position = c.positionSpin->value();
	}
}

void JointManager::handleSliderChange()
{
	if (!m_currentFrame)
		return;

	for (unsigned i = 0; i < m_controlList.size(); i++)
	{
		Controls c = m_controlList[i];
		c.positionSpin->blockSignals(true);
		c.positionSpin->setValue(c.positionSlider->value() * ticToAng);
		c.positionSpin->blockSignals(false);

		std::string name = c.positionLabel->text().toStdString();
		int index = indexToName(name);
		if (index < 0)
			continue;
		m_currentFrame->joints[index].position = c.positionSpin->value();
	}
}

void JointManager::handleDurSliderChange()
{
	if (!m_currentFrame)
		return;

	m_durationSpin->blockSignals(true);
	m_durationSpin->setValue(m_durationSlider->value() / 100.0);
	m_durationSpin->blockSignals(false);

	m_currentFrame->duration = m_durationSpin->value();
}

void JointManager::handleDurSpinChange()
{
	if (!m_currentFrame)
		return;

	m_durationSlider->blockSignals(true);
	m_durationSlider->setValue(m_durationSpin->value() * 100);
	m_durationSlider->blockSignals(false);

	m_currentFrame->duration = m_durationSpin->value();
}

void JointManager::handleVelChange()
{
	if (!m_currentFrame)
		return;

	for (unsigned i = 0; i < m_controlList.size(); i++)
	{
		std::string name = m_controlList[i].positionLabel->text().toStdString();
		int index = indexToName(name);
		if (index < 0)
			continue;
		m_currentFrame->joints[index].velocity = m_controlList[i].velSpin->value();
	}

}

void JointManager::handleGlobalEffort()
{
	if (!m_currentFrame)
		return;

	for (unsigned i = 0; i < m_controlList.size(); i++)
	{
		Controls c = m_controlList[i];
		c.effortSpin->blockSignals(true);
		if (m_globalEffortSpin->value() > c.effortSpin->value())
			c.effortSpin->setValue(m_globalEffortSpin->value());
		c.effortSpin->blockSignals(false);

		std::string name = m_controlList[i].positionLabel->text().toStdString();
		int index = indexToName(name);
		if (index < 0)
			continue;
		m_currentFrame->joints[index].effort = m_globalEffortSpin->value();
	}
}

int JointManager::indexToName(std::string name)
{
	for (unsigned i = 0; i < m_jointList.size(); i++)
	{
		if (m_jointList[i] == name)
			return i;
	}
	return -1;
}
