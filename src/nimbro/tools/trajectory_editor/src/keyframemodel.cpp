//Displays trajectories
//Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include "keyframemodel.h"
#include "invkinbox.h"


#include <QInputDialog>
#include <QPushButton>
#include <QFileDialog>
#include <QVBoxLayout>
#include <ros/package.h>

#include <motion_player/StreamMotion.h>
#include <motion_player/PlayMotion.h>



KeyframeModel::KeyframeModel(QObject* parent)
 : QAbstractListModel(parent)
 , m_fileActive(false)
{
	ros::NodeHandle nh("~");

	m_parent = (QWidget* )parent;

	m_cl_update = nh.serviceClient<motion_player::StreamMotion>("/motion_player/update");
	m_cl_play = nh.serviceClient<motion_player::PlayMotion>("/motion_player/play");
}

KeyframeModel::~KeyframeModel()
{}


void KeyframeModel::initRobot(RobotDisplay* robot)
{
	m_robot = robot;
	m_rbdl = robot->getRBDL();

	m_motion.jointList.resize(m_rbdl->mJoints.size());
	for (unsigned i = 0; i < m_rbdl->mJoints.size(); i++)
	{
		if (m_rbdl->jointName(i).empty())
			m_motion.jointList[i] = "";
		else
			m_motion.jointList[i] = m_rbdl->jointName(i);
	}
}

void KeyframeModel::initJointDisplay(QWidget* widget)
{
	Q_FOREACH(QObject* obj, widget->children())
		obj->deleteLater();

	if(widget->layout())
		delete widget->layout();

	QVBoxLayout* layout = new QVBoxLayout(widget);

	m_ikl = new InvKinBox(m_rbdl, "left_foot_link", m_motion.jointList, widget);
	connect(m_ikl, SIGNAL(changed()), this, SLOT(updateRobotDisplay()));
	layout->addWidget(m_ikl);

	m_ikr = new InvKinBox(m_rbdl, "right_foot_link", m_motion.jointList, widget);
	connect(m_ikr, SIGNAL(changed()), this, SLOT(updateRobotDisplay()));
	layout->addWidget(m_ikr);

	QWidget* jointWidget = new QWidget(widget);
	m_jointManager = new JointManager(jointWidget, m_motion.jointList);
	connect(m_jointManager, SIGNAL(positionSpinChanged()), this, SLOT(updateRobotDisplay()));
	connect(m_jointManager, SIGNAL(sliderChanged()), this, SLOT(updateRobotDisplay()));
	layout->addWidget(jointWidget);

	connect(m_jointManager, SIGNAL(positionSpinChanged()), m_ikl, SLOT(updateFromMotionFile()));
	connect(m_jointManager, SIGNAL(positionSpinChanged()), m_ikr, SLOT(updateFromMotionFile()));

	connect(m_jointManager, SIGNAL(sliderChanged()), m_ikl, SLOT(updateFromMotionFile()));
	connect(m_jointManager, SIGNAL(sliderChanged()), m_ikr, SLOT(updateFromMotionFile()));
}


QVariant KeyframeModel::data(const QModelIndex& index, int role) const
{
	if(role != Qt::DisplayRole)
		return QVariant();

	return QVariant(index.row());
}

int KeyframeModel::rowCount(const QModelIndex& parent) const
{
	return m_motion.frames.size();
}

void KeyframeModel::load()
{
	const QString dir = QString::fromStdString(ros::package::getPath("launch") + "/motions/");
	QString filenName = QFileDialog::getOpenFileName(m_parent, tr("open file"), dir);
	m_motion.jointList.clear();
	m_motion.frames.clear();

	m_motion.load(filenName.toStdString());
	reset();

	m_jointManager->updateJointList(m_motion.jointList);
	m_ikl->updateJointList(m_motion.jointList);
	m_ikr->updateJointList(m_motion.jointList);
}

void KeyframeModel::save()
{
	if (m_motion.motionName.empty() || m_motion.jointList.empty())
		return;

	std::string prefix;
	prefix = ros::package::getPath("launch") + "/motions/";

	m_motion.save(prefix + m_motion.motionName);

}

void KeyframeModel::createMotion()
{
	if (m_fileActive)
		return;

	m_motion.motionName = QInputDialog::getText(m_parent, tr("Please name your motion file"), tr("Filename: ")).toStdString();
	m_motion.frames.clear();
	m_motion.jointList.clear();
	m_motion.jointList.resize(m_rbdl->mJoints.size());
	for (unsigned i = 0; i < m_rbdl->mJoints.size(); i++)
	{
		if (m_rbdl->jointName(i).empty())
			m_motion.jointList[i] = "";
		else
			m_motion.jointList[i] = m_rbdl->jointName(i);
	}

	m_jointManager->updateJointList(m_motion.jointList);

	m_motion.frames.push_back(createFrame());
	reset();
}

KeyframeModel::KeyframePtr KeyframeModel::createFrame()
{
	KeyframePtr newKeyframe(new motionfile::Keyframe);
	newKeyframe->duration = 0;
	for (unsigned i = 0; i < m_motion.jointList.size(); i++)
	{
		motionfile::FrameJoint newJoint;
		newJoint.effort = 1;
		newJoint.position = 0;
		newKeyframe->joints.push_back(newJoint);
	}

	m_currentFrame = newKeyframe;
	m_jointManager->setFrame(newKeyframe);
	m_ikl->setFrame(newKeyframe);
	m_ikr->setFrame(newKeyframe);
	return newKeyframe;
}

KeyframeModel::KeyframePtr KeyframeModel::createFrame(KeyframeModel::KeyframePtr oldFrame)
{
	if(!oldFrame)
		return createFrame();

	KeyframePtr newFrame(new motionfile::Keyframe);
	newFrame->duration = oldFrame->duration;
	for (unsigned i = 0; i < oldFrame->joints.size(); i++)
	{
		motionfile::FrameJoint newJoint(oldFrame->joints[i]);
		newFrame->joints.push_back(newJoint);
	}

	m_currentFrame = newFrame;
	m_jointManager->setFrame(newFrame);
	m_ikl->setFrame(newFrame);
	m_ikr->setFrame(newFrame);
	return newFrame;
}


void KeyframeModel::addFrame()
{
	if (m_motion.motionName.empty())
		return;

	beginInsertRows(QModelIndex(), 0, m_motion.frames.size());
	m_motion.frames.push_back(createFrame(m_currentFrame));
	endInsertRows();
}

void KeyframeModel::removeFrame(unsigned in)
{
	if (in > m_motion.frames.size() || in < 0)
		return;

	if (m_motion.frames.size() == 1)
	{
		m_currentFrame = KeyframePtr();
		m_jointManager->unsetFrame();
	}
	else
	{
		m_currentFrame = m_motion.frames[in -1];
		m_jointManager->setFrame(m_currentFrame);
		m_ikr->setFrame(m_currentFrame);
		m_ikl->setFrame(m_currentFrame);
	}

	beginRemoveRows(QModelIndex(), in, in);
	std::vector<KeyframePtr>::iterator c = m_motion.frames.begin() + in;
	if (m_motion.frames.size() > 0)
		m_motion.frames.erase(c);
	endRemoveRows();
	dataChanged(index(in, 0), index(m_motion.frames.size(), 0));
}

void KeyframeModel::handleIndexChange(const QModelIndex& index)
{
	int frameIndex = index.row();
	m_currentFrame = m_motion.frames[frameIndex];
	if (m_currentFrame)
	{
		m_jointManager->setFrame(m_currentFrame);
		m_ikr->setFrame(m_currentFrame);
		m_ikl->setFrame(m_currentFrame);
	}
}

void KeyframeModel::updateRobotDisplay()
{
	if (!m_currentFrame)
		return;

	std::vector<double> positions(m_rbdl->dof_count, 0);
	for (unsigned i = 0; i < m_motion.jointList.size(); i++)
	{
		if (m_motion.jointList[i].empty())
			continue;

		int ind = m_rbdl->jointIndex(m_motion.jointList[i]) - 1;
		positions[ind] = m_currentFrame->joints[i].position;
	}
	m_robot->update(positions);
}


void KeyframeModel::requestPlay()
{
	requestUpdate();
	motion_player::PlayMotion play;
	play.request.name = m_motion.motionName;

	m_cl_play.call(play);



}

void KeyframeModel::requestUpdate()
{
	motion_player::StreamMotion update;
	std::string sendM = m_motion.dump();

	update.request.name = m_motion.motionName;
	update.request.motion = sendM;

	m_cl_update.call(update);
}

void KeyframeModel::handlePlayCurrentFrame()
{
	if (!m_currentFrame)
		return;

	std::string fName = "FramePlay";

	motionfile::Motion tempMotion;
	tempMotion.motionName = fName;
	tempMotion.jointList = m_motion.jointList;
	tempMotion.playState = "init";
	tempMotion.preState = "init";
	tempMotion.postState = "init";
	tempMotion.frames.push_back(m_currentFrame);
	tempMotion.frames.push_back(m_currentFrame);

	motion_player::StreamMotion frame;
	std::string sendM = tempMotion.dump();

	frame.request.name = fName;
	frame.request.motion = sendM;

	m_cl_update.call(frame);

	motion_player::PlayMotion play;
	play.request.name = fName;
	m_cl_play.call(play);

}
