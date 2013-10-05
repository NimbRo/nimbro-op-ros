// Color Calibration Node
// Author: Max Schwarz <Max@x-quadraht.de>

#include "calibrationnodelet.h"

#include "calibscene.h"

#include <qtguithread/qtguithread.h>
#include <pluginlib/class_list_macros.h>

#include <calibration/EnumerateCameraParams.h>
#include <calibration/ClassifyImage.h>

PLUGINLIB_DECLARE_CLASS(openplatform, calibration, CalibrationNodelet, nodelet::Nodelet)



void CalibrationNodelet::onInit()
{
	ros::NodeHandle nh = getNodeHandle();
	m_sub_raw = nh.subscribe("image", 4, &CalibrationNodelet::receiveRawImage, this);
	m_sub_processed = nh.subscribe("classes", 4, &CalibrationNodelet::receiveProcessedImage, this);
	m_client_enumerate = nh.serviceClient<
		calibration::EnumerateCameraParamsRequest,
		calibration::EnumerateCameraParamsResponse>("/cam/enumerateParams");

	GUIThread::getInstance()->addWidgetProvider(this);
}

QWidget* CalibrationNodelet::createWidget()
{
	qRegisterMetaType<sensor_msgs::Image::Ptr>("sensor_msgs::Image::Ptr");
	qRegisterMetaType<calibration::UpdateLUTRequest::Ptr>("calibration::UpdateLUTRequest::Ptr");

	m_calib = new Calibration();
	m_calib->show();

	connect(
		this, SIGNAL(rawImageReceived(sensor_msgs::Image::Ptr)),
		m_calib, SLOT(takeLiveRawImage(sensor_msgs::Image::Ptr)),
		Qt::QueuedConnection
	);
	connect(
		this, SIGNAL(processedImageReceived(sensor_msgs::Image::Ptr)),
		m_calib, SLOT(takeLiveProcessedImage(sensor_msgs::Image::Ptr)),
		Qt::QueuedConnection
	);

	connect(
		m_calib->scene(), SIGNAL(LUT_updated(calibration::UpdateLUTRequest::Ptr)),
		SLOT(sendLUT(calibration::UpdateLUTRequest::Ptr)),
		Qt::DirectConnection
	);
	connect(
		this, SIGNAL(LUTUpdateComplete()),
		m_calib, SLOT(notifyLUTUpdateComplete()),
		Qt::QueuedConnection
	);

	connect(
		m_calib, SIGNAL(setParam(calibration::SetCameraParamRequest::Ptr)),
		SLOT(setCameraParam(calibration::SetCameraParamRequest::Ptr)),
		Qt::DirectConnection
	);
	connect(
		this, SIGNAL(gotCameraParams(calibration::EnumerateCameraParamsResponse::Ptr)),
		m_calib, SIGNAL(setParamInfo(calibration::EnumerateCameraParamsResponse::Ptr)),
		Qt::QueuedConnection
	);

	connect(
		m_calib, SIGNAL(classify(sensor_msgs::Image::Ptr)),
		SLOT(classify(sensor_msgs::Image::Ptr)),
		Qt::DirectConnection
	);
	connect(
		this, SIGNAL(gotClassificationResult(sensor_msgs::Image::Ptr)),
		m_calib, SLOT(takeClassificationResult(sensor_msgs::Image::Ptr)),
		Qt::QueuedConnection
	);

	requestCameraParamEnumeration();

	return m_calib;
}

void CalibrationNodelet::receiveRawImage(const sensor_msgs::Image::Ptr& img)
{
	if(img->header.stamp > m_lastStamp && (img->header.stamp - m_lastStamp) < ros::Duration(0.1))
		return;

	emit rawImageReceived(img);
	m_lastStamp = img->header.stamp;
}

void CalibrationNodelet::receiveProcessedImage(const sensor_msgs::Image::Ptr& img)
{
	emit processedImageReceived(img);
}

void CalibrationNodelet::sendLUT(const calibration::UpdateLUTRequest::Ptr& lut)
{
	calibration::UpdateLUTResponse resp;

	if(!ros::service::call("/classificator/updateLUT", *lut, resp))
	{
		NODELET_FATAL("Could not call /classificator/updateLUT service");
		return;
	}

	emit LUTUpdateComplete();
}

void CalibrationNodelet::requestCameraParamEnumeration()
{
	calibration::EnumerateCameraParamsRequest req;
	calibration::EnumerateCameraParamsResponse::Ptr response =
		boost::make_shared<calibration::EnumerateCameraParamsResponse>();

	if(!m_client_enumerate.waitForExistence(ros::Duration(10.0f)))
	{
		NODELET_FATAL("Parameter enumeration service did not appear.");
		return;
	}

	if(!m_client_enumerate.call(req, *response))
	{
		NODELET_FATAL("Could not call /cam/enumerateParams service");
		return;
	}

	emit gotCameraParams(response);
}

void CalibrationNodelet::setCameraParam(const calibration::SetCameraParamRequest::Ptr& param)
{
	calibration::SetCameraParamResponse resp;

	if(!ros::service::call("/cam/setParam", *param, resp))
	{
		NODELET_FATAL("Could not call /cam/setParam service");
		return;
	}
}

void CalibrationNodelet::classify(const sensor_msgs::Image::Ptr& img)
{
	calibration::ClassifyImage req;
	req.request.input = *img;

	if(!ros::service::call("/classificator/processImage", req.request, req.response))
	{
		NODELET_FATAL("Could not call /classificator/processImage service");
		return;
	}

	emit gotClassificationResult(boost::make_shared<sensor_msgs::Image>(req.response.output));
}



