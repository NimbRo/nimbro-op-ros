// Client object
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <config_server/parameter.h>
#include <config_server/parameterclient.h>
#include <config_server/Subscribe.h>
#include <ros/service.h>
#include <ros/this_node.h>

namespace config_server
{

ParameterBase::~ParameterBase()
{
	if(!m_name.empty())
		ParameterClient::instance()->unregisterParameter(this);
}

void ParameterBase::init(const ParameterDescription& desc, ros::NodeHandle* nh, bool create)
{
	m_name = desc.name;
	if(m_name[0] != '/')
		m_name = ros::this_node::getName() + "/" + desc.name;

	ParameterDescription createDesc;
	if(create)
		createDesc = desc;

	createDesc.name = m_name;

	ParameterClient::instance()->registerParameter(this, createDesc);
}

bool ParameterBase::handleSet(const std::string& value)
{
	if(!deserialize(value))
		return false;
	notifyClient();
	return true;
}

void ParameterBase::notifyServer()
{
	ParameterClient::instance()->notify(this, serialize());
}

void ParameterBase::notifyClient()
{
}

}
