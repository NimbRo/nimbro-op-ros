// Inverse kinematics box

#ifndef INVKINBOX_H
#define INVKINBOX_H

#include <QtGui/QWidget>
#include <QSlider>
#include <rbdl/rbdl_parser.h>
#include "leg_ik.h"

#include <motion_file/motionfile.h>

namespace Ui
{
	class InvKinBox;
}

class InvKinBox : public QWidget
{
Q_OBJECT
public:
	typedef boost::shared_ptr<motionfile::Keyframe> KeyframePtr;

	InvKinBox(const boost::shared_ptr<rbdl_parser::URDF_RBDL_Model>& model, const std::string& tip, const std::vector< std::string >& modelJointList, QWidget* parent);
	virtual ~InvKinBox();

	void setFrame(const KeyframePtr& ptr);
	void updateJointList(const std::vector<std::string>& list);
public Q_SLOTS:
	void updateIK();
	void updateFromMotionFile();
	void updateSlider();
	void updateSpinboxes();
Q_SIGNALS:
	void changed();
private:
	void ikCB(int idx, double angle);

	boost::shared_ptr<rbdl_parser::URDF_RBDL_Model> m_model;
	LegIK* m_leg_ik;

	Ui::InvKinBox* m_ui;

	KeyframePtr m_kf;

	std::vector<std::string> m_jointList;

	int m_bodyIdx;
};

#endif
