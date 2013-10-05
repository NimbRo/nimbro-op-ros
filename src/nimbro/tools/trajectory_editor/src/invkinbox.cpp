// Inverse kinematics box

#include "invkinbox.h"
#include <rbdl/Kinematics.h>

#include "ui_invkinbox.h"
#include <boost/iterator/iterator_concepts.hpp>

const double FACTOR = 0.0001;

InvKinBox::InvKinBox(
	const boost::shared_ptr< rbdl_parser::URDF_RBDL_Model >& model,
	const std::string& tip,
	const std::vector< std::string >& modelJointList, QWidget* parent)
 : QWidget(parent)
 , m_model(model)
 , m_jointList(modelJointList)
{
	m_bodyIdx = model->GetBodyId(tip.c_str());

	m_leg_ik = new LegIK(model, tip);

	m_ui = new Ui::InvKinBox;

	m_ui->setupUi(this);

	connect(m_ui->xSlider, SIGNAL(valueChanged(int)), SLOT(updateIK()));
	connect(m_ui->ySlider, SIGNAL(valueChanged(int)), SLOT(updateIK()));
	connect(m_ui->zSlider, SIGNAL(valueChanged(int)), SLOT(updateIK()));
	connect(m_ui->rollSlider, SIGNAL(valueChanged(int)), SLOT(updateIK()));
	connect(m_ui->pitchSlider, SIGNAL(valueChanged(int)), SLOT(updateIK()));
	connect(m_ui->yawSlider, SIGNAL(valueChanged(int)), SLOT(updateIK()));

	connect(m_ui->xSlider, SIGNAL(valueChanged(int)), SLOT(updateSpinboxes()));
	connect(m_ui->ySlider, SIGNAL(valueChanged(int)), SLOT(updateSpinboxes()));
	connect(m_ui->zSlider, SIGNAL(valueChanged(int)), SLOT(updateSpinboxes()));
	connect(m_ui->rollSlider, SIGNAL(valueChanged(int)), SLOT(updateSpinboxes()));
	connect(m_ui->pitchSlider, SIGNAL(valueChanged(int)), SLOT(updateSpinboxes()));
	connect(m_ui->yawSlider, SIGNAL(valueChanged(int)), SLOT(updateSpinboxes()));

	connect(m_ui->xBox, SIGNAL(valueChanged(double)), SLOT(updateSlider()));
	connect(m_ui->yBox, SIGNAL(valueChanged(double)), SLOT(updateSlider()));
	connect(m_ui->zBox, SIGNAL(valueChanged(double)), SLOT(updateSlider()));
	connect(m_ui->rollBox, SIGNAL(valueChanged(double)), SLOT(updateSlider()));
	connect(m_ui->pitchBox, SIGNAL(valueChanged(double)), SLOT(updateSlider()));
	connect(m_ui->yawBox, SIGNAL(valueChanged(double)), SLOT(updateSlider()));

	connect(m_ui->xBox, SIGNAL(valueChanged(double)), SLOT(updateIK()));
	connect(m_ui->yBox, SIGNAL(valueChanged(double)), SLOT(updateIK()));
	connect(m_ui->zBox, SIGNAL(valueChanged(double)), SLOT(updateIK()));
	connect(m_ui->rollBox, SIGNAL(valueChanged(double)), SLOT(updateIK()));
	connect(m_ui->pitchBox, SIGNAL(valueChanged(double)), SLOT(updateIK()));
	connect(m_ui->yawBox, SIGNAL(valueChanged(double)), SLOT(updateIK()));

}

void InvKinBox::updateSlider()
{
	m_ui->xSlider->blockSignals(true);
	m_ui->ySlider->blockSignals(true);
	m_ui->zSlider->blockSignals(true);
	m_ui->rollSlider->blockSignals(true);
	m_ui->pitchSlider->blockSignals(true);
	m_ui->yawSlider->blockSignals(true);

	m_ui->xSlider->setValue((m_ui->xBox->value() / FACTOR));
	m_ui->ySlider->setValue((m_ui->yBox->value() / FACTOR));
	m_ui->zSlider->setValue((m_ui->zBox->value() / FACTOR));
	m_ui->rollSlider->setValue((m_ui->rollBox->value() / FACTOR));
	m_ui->pitchSlider->setValue((m_ui->pitchBox->value() / FACTOR));
	m_ui->yawSlider->setValue((m_ui->yawBox->value() / FACTOR));


	m_ui->xSlider->blockSignals(false);
	m_ui->ySlider->blockSignals(false);
	m_ui->zSlider->blockSignals(false);
	m_ui->rollSlider->blockSignals(false);
	m_ui->pitchSlider->blockSignals(false);
	m_ui->yawSlider->blockSignals(false);
}

void InvKinBox::updateSpinboxes()
{
	m_ui->xBox->blockSignals(true);
	m_ui->yBox->blockSignals(true);
	m_ui->zBox->blockSignals(true);
	m_ui->rollBox->blockSignals(true);
	m_ui->pitchBox->blockSignals(true);
	m_ui->yawBox->blockSignals(true);

	m_ui->xBox->setValue((m_ui->xSlider->value() * FACTOR));
	m_ui->yBox->setValue((m_ui->ySlider->value() * FACTOR));
	m_ui->zBox->setValue((m_ui->zSlider->value() * FACTOR));
	m_ui->rollBox->setValue((m_ui->rollSlider->value() * FACTOR));
	m_ui->pitchBox->setValue((m_ui->pitchSlider->value() * FACTOR));
	m_ui->yawBox->setValue((m_ui->yawSlider->value() * FACTOR));


	m_ui->xBox->blockSignals(false);
	m_ui->yBox->blockSignals(false);
	m_ui->zBox->blockSignals(false);
	m_ui->rollBox->blockSignals(false);
	m_ui->pitchBox->blockSignals(false);
	m_ui->yawBox->blockSignals(false);
}


InvKinBox::~InvKinBox()
{
}

void InvKinBox::updateJointList(const std::vector< std::string >& list)
{
	m_jointList = list;
}

void InvKinBox::updateIK()
{
	Eigen::Vector3d pos(
		FACTOR * m_ui->xSlider->value(),
		FACTOR * m_ui->ySlider->value(),
		FACTOR * m_ui->zSlider->value()
	);

	Eigen::Matrix3d rot;
	rot =
		Eigen::AngleAxisd(FACTOR * m_ui->yawSlider->value(), Eigen::Vector3d(0, 0, 1)) *
		Eigen::AngleAxisd(FACTOR * m_ui->pitchSlider->value(), Eigen::Vector3d(0, 1, 0)) *
		Eigen::AngleAxisd(FACTOR * m_ui->rollSlider->value(), Eigen::Vector3d(1, 0, 0))
	;

	if(!m_leg_ik->doIK(boost::bind(&InvKinBox::ikCB, this, _1, _2), pos, rot))
	{
		ROS_WARN("no IK solution");
	}

	ROS_WARN("changed");
	changed();
}

void InvKinBox::setFrame(const InvKinBox::KeyframePtr& ptr)
{
	m_kf = ptr;
	updateFromMotionFile();
}

void InvKinBox::ikCB(int idx, double angle)
{
	if(m_kf)
	{
		std::vector<std::string>::iterator it = std::find(m_jointList.begin(), m_jointList.end(), m_model->jointName(idx+1));
		if(it == m_jointList.end())
		{
			ROS_WARN("Can't find joint '%s'", m_model->jointName(idx+1).c_str());
			return;
		}

		int kf_idx = it - m_jointList.begin();

		ROS_WARN("Joint %s maps to %d", m_model->jointName(idx+1).c_str(), kf_idx);

		motionfile::FrameJoint* j = &m_kf->joints[kf_idx];

		j->position = angle;
	}
	else
	{
		ROS_WARN("No kf loaded");
	}
}

void InvKinBox::updateFromMotionFile()
{
	if(!m_kf)
		return;

	Math::VectorNd zero = Math::VectorNd::Constant(m_model->dof_count, 0);
	Math::VectorNd q = zero;

	for(size_t i = 0; i < m_jointList.size(); ++i)
	{
		double angle = m_kf->joints[i].position;
		const std::string& name = m_jointList[i];

		if(m_jointList[i].empty())
			continue;

		int idx = m_model->jointIndex(name)-1;
		if(idx >= (int)m_model->dof_count)
		{
			ROS_ERROR("idx");
			abort();
		}

		q[idx] = angle;
	}

	RigidBodyDynamics::UpdateKinematics(*m_model, q, zero, zero);

	if(m_bodyIdx >= (int)m_model->X_base.size())
	{
		ROS_ERROR("bodyIDx");
		abort();
	}
	Math::SpatialTransform X = m_model->X_base[m_bodyIdx];

	m_ui->xSlider->blockSignals(true);
	m_ui->ySlider->blockSignals(true);
	m_ui->zSlider->blockSignals(true);
	m_ui->pitchSlider->blockSignals(true);
	m_ui->rollSlider->blockSignals(true);
	m_ui->yawSlider->blockSignals(true);

	m_ui->xBox->blockSignals(true);
	m_ui->yBox->blockSignals(true);
	m_ui->zBox->blockSignals(true);
	m_ui->rollBox->blockSignals(true);
	m_ui->pitchBox->blockSignals(true);
	m_ui->yawBox->blockSignals(true);

	m_ui->xSlider->setValue(X.r.x() / FACTOR);
	m_ui->ySlider->setValue(X.r.y() / FACTOR);
	m_ui->zSlider->setValue(X.r.z() / FACTOR);

	m_ui->xBox->setValue(X.r.x());
	m_ui->yBox->setValue(X.r.y());
	m_ui->zBox->setValue(X.r.z());

	Eigen::Vector3d euler = X.E.transpose().eulerAngles(2, 1, 0);

	m_ui->pitchBox->setValue(euler.y());
	m_ui->rollBox->setValue(euler.z());
	m_ui->yawBox->setValue(euler.x());

	m_ui->pitchSlider->setValue(euler.y() / FACTOR);
	m_ui->rollSlider->setValue(euler.z() / FACTOR);
	m_ui->yawSlider->setValue(euler.x() / FACTOR);

	m_ui->xSlider->blockSignals(false);
	m_ui->ySlider->blockSignals(false);
	m_ui->zSlider->blockSignals(false);
	m_ui->pitchSlider->blockSignals(false);
	m_ui->rollSlider->blockSignals(false);
	m_ui->yawSlider->blockSignals(false);

	m_ui->xBox->blockSignals(false);
	m_ui->yBox->blockSignals(false);
	m_ui->zBox->blockSignals(false);
	m_ui->rollBox->blockSignals(false);
	m_ui->pitchBox->blockSignals(false);
	m_ui->yawBox->blockSignals(false);
}

