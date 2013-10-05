//Displays trajectories
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#ifndef KEYFRAME_MODEL_H
#define KEYFRAME_MODEL_H

#include <QtGui/QWidget>
#include <QtCore/QAbstractListModel>


#include <urdf/model.h>
#include <rbdl/rbdl_parser.h>


#include <motion_file/motionfile.h>

#include "robotdisplay.h"
#include "jointdisplay.h"

class InvKinBox;


class KeyframeModel : public QAbstractListModel
{
Q_OBJECT
public:
	explicit KeyframeModel(QObject* parent = 0);
	virtual ~KeyframeModel();

	virtual QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;
	virtual int rowCount(const QModelIndex& parent = QModelIndex()) const;

	void initRobot(RobotDisplay* robot);
	void initJointDisplay(QWidget* widget);

	void removeFrame(unsigned in);


public Q_SLOTS:

	void save();
	void load();
	void addFrame();
	void createMotion();
	void handleIndexChange(const QModelIndex& index);
	void updateRobotDisplay();

	void handlePlayCurrentFrame();

	void requestUpdate();
	void requestPlay();



private:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;

	QWidget* m_parent;
	JointManager* m_jointManager;
	InvKinBox* m_ikl;
	InvKinBox* m_ikr;


	bool m_fileActive;

	motionfile::Motion m_motion;
	KeyframePtr m_currentFrame;

	ros::ServiceClient m_cl_update;
	ros::ServiceClient m_cl_play;


    RobotDisplay* m_robot;
	boost::shared_ptr<rbdl_parser::URDF_RBDL_Model> m_rbdl;

	KeyframePtr createFrame();
	KeyframePtr createFrame(KeyframePtr oldFrame);

};


#endif