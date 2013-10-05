// Config server client singleton
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <config_server/parameterclient.h>
#include <config_server/parameter.h>

#include <ros/service.h>

namespace config_server
{

ParameterClient* ParameterClient::g_instance = 0;

ParameterClient* ParameterClient::instance()
{
	// FIXME: This is not thread-safe!
	if(!g_instance)
		g_instance = new ParameterClient();

	return g_instance;
}


ParameterClient::ParameterClient()
 : m_corked(0)
{
	ros::NodeHandle nh("~");
	m_srv = nh.advertiseService("configure", &ParameterClient::handleSet, this);
	m_srv_name = nh.getNamespace() + "/configure";

	m_subscribe.request.callback = m_srv_name;
}

ParameterClient::~ParameterClient()
{
}


void ParameterClient::registerParameter(ParameterBase* param, const ParameterDescription& desc)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	m_parameters.insert(
		std::pair<std::string, ParameterBase*>(param->name(), param)
	);

	if(m_corked)
	{
		for(size_t i = 0; i < m_subscribe.request.parameters.size(); ++i)
		{
			if(m_subscribe.request.parameters[i].name == desc.name)
				return;
		}

		m_subscribe.request.parameters.push_back(desc);
	}
	else
	{
		config_server::Subscribe srv;
		srv.request.callback = m_srv_name;
		srv.request.prop = param->name();
		srv.request.desc = desc;
		srv.request.desc.name = param->name();

		if(!ros::service::call("/config_server/subscribe", srv))
		{
			ROS_ERROR("Could not call /config_server/subscribe for parameter '%s'", param->name().c_str());
			param->handleSet(srv.request.desc.default_value);
		}
		else
			param->handleSet(srv.response.value);
	}
}

void ParameterClient::cork()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_corked++;
}

void ParameterClient::uncork()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	if(m_corked == 0)
	{
		ROS_ERROR("ParameterClient::uncork() called while not corked!");
		return;
	}

	if(--m_corked != 0)
		return;

	sync();
}

void ParameterClient::sync()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	if(!ros::service::call("/config_server/subscribe_list", m_subscribe))
	{
		ROS_ERROR("Could not call /config_server/subscribe_list");
		abort();
	}

	if(m_subscribe.response.values.size() != m_subscribe.request.parameters.size())
	{
		ROS_ERROR("Invalid response size from service call /config_server/subscribe_list");
		ROS_ERROR("Initial parameter values will be wrong.");
		return;
	}

	for(size_t i = 0; i < m_subscribe.request.parameters.size(); ++i)
	{
		std::pair<ParameterMap::iterator, ParameterMap::iterator> eq;
		eq = m_parameters.equal_range(m_subscribe.request.parameters[i].name);

		for(ParameterMap::iterator it = eq.first; it != eq.second; ++it)
		{
			it->second->handleSet(m_subscribe.response.values[i]);
		}
	}

	m_subscribe.request.parameters.clear();
}


void ParameterClient::unregisterParameter(ParameterBase* param)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	for(ParameterMap::iterator it = m_parameters.begin(); it != m_parameters.end(); ++it)
	{
		if(it->second == param)
		{
			m_parameters.erase(it);
			return;
		}
	}

	ROS_FATAL("config_server::ParameterClient: tried to unregister unknown parameter '%s'", param->name().c_str());
	abort();
}

bool ParameterClient::handleSet(SetParameterRequest& req, SetParameterResponse& resp)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	std::pair<ParameterMap::iterator, ParameterMap::iterator> eq;
	eq = m_parameters.equal_range(req.name);

	for(ParameterMap::iterator it = eq.first; it != eq.second; ++it)
	{
		if(!it->second->handleSet(req.value))
			return false;
	}

	return true;
}

void ParameterClient::notify(ParameterBase* param, const std::string& value)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	// Handle local notification
	std::pair<ParameterMap::iterator, ParameterMap::iterator> eq;
	eq = m_parameters.equal_range(param->name());

	for(ParameterMap::iterator it = eq.first; it != eq.second; ++it)
	{
		if(it->second == param)
			continue;

		it->second->handleSet(value);
	}

	// Notify the server
	config_server::SetParameter srv;
	srv.request.name = param->name();
	srv.request.value = value;
	srv.request.no_notify = m_srv_name;

	if(!ros::service::call("/config_server/set_parameter", srv))
	{
		ROS_ERROR("Could not set parameter '%s' on the parameter server", param->name().c_str());
	}
}

}
