// Configuration (e.g. camera parameters) manager
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <string>

#include <calibration/EnumerateCameraParams.h>



class ConfigManager
{
public:
	ConfigManager();
	virtual ~ConfigManager();

	// all functions return 0 on success
	int init(int fd, const std::vector<std::string>& controls);
	int readFromCamera(int fd);
	int setParamValue(int fd, const std::string& name, int value);
	int setParamValue(int fd, int id, int value);
	int getParamValue(const std::string& name, int* value);
	int getParamValue(int id, int* value);

	int readConfigFile(int fd, std::istream* stream);
	void writeConfigFile(int fd, std::ostream* ostream);

	calibration::CameraParam* getParamInfo(const std::string& name);
	calibration::CameraParam* getParamInfo(int id);

	const calibration::EnumerateCameraParamsResponse& enumerateParams() const
	{ return m_controls; }

	//! @name Write tracking
	//@{
	inline bool writePending() const
	{ return m_writePending; }
	//@}
private:
	calibration::EnumerateCameraParamsResponse m_controls;
	bool m_writePending;
};

#endif
