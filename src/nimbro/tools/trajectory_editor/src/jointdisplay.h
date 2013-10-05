//Displays joints of loaded frame
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#ifndef JOINTDISPLAY_H
#define JOINTDISPLAY_H

#include <QAbstractItemModel>
#include <qitemdelegate.h>
#include <qstyleditemdelegate.h>
#include <QLabel>
#include <QSpinBox>
#include <boost/shared_ptr.hpp>
#include <boost/concept_check.hpp>

#include <motion_file/motionfile.h>

class JointManager : public QObject
{
Q_OBJECT
public:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframPtr;

	JointManager(QWidget* controlWidget, const std::vector<std::string>& modelJointList);
	~JointManager(){};

	void setFrame(KeyframPtr frame);
	void unsetFrame();

	void updateJointList(const std::vector<std::string>& jointList);

public Q_SLOTS:
	void updateFrame();
	void handleSliderChange();
	void handleEffSpinChange();
	void handlePosSpinChange();
	void handleDurSliderChange();
	void handleDurSpinChange();
	void handleGlobalEffort();
	void handleVelChange();


Q_SIGNALS:
	void sliderChanged();
	void positionSpinChanged();
	void effortSpinChanged();
	void velocityChanged();

private:
	struct Controls
	{
		QLabel* positionLabel;
		QSlider* positionSlider;
		QDoubleSpinBox* positionSpin;
		QDoubleSpinBox* effortSpin;
		QDoubleSpinBox* velSpin;
	};

	enum controlPos
	{
		DURATION,
		GLOBAL_EFFORT,
		CONTROL_TITLE,
		CONTROLS
	};


	QWidget* m_controlWidget;

	KeyframPtr m_currentFrame;
	std::vector<std::string> m_jointList;
	std::vector<Controls> m_controlList;

	QSlider* m_durationSlider;
	QDoubleSpinBox* m_durationSpin;

	QDoubleSpinBox* m_globalEffortSpin;

	void initFrames(std::vector<std::string> jointList);
	int indexToName(std::string name);
};


#endif