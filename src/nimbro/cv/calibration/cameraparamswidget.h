// Provides access to camera settings (e.g. white balance)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CAMERAPARAMSWIDGET_H
#define CAMERAPARAMSWIDGET_H

#include <QtGui/QScrollArea>
#include <calibration/EnumerateCameraParams.h>
#include <calibration/SetCameraParam.h>

class CameraParamsWidget : public QScrollArea
{
Q_OBJECT
public:
	explicit CameraParamsWidget(QWidget* parent = 0);
	virtual ~CameraParamsWidget();

public slots:
	void setParamInfo(const calibration::EnumerateCameraParamsResponse::Ptr& resp);
signals:
	void setParam(const calibration::SetCameraParamRequest::Ptr& req);
private slots:
	void updateSlider(int value);
	void updateCheckBox(bool checked);
	void updateComboBox();
private:
	calibration::EnumerateCameraParamsResponse::Ptr m_paramInfo;
	QWidget *m_w;

	QWidget* createEditorFor(const calibration::CameraParam& param);
};

#endif
