// Configuration server
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <config_server/configserver.h>
#include <config_server/ParameterList.h>

#include <ros/node_handle.h>
#include <ros/service.h>
#include <ros/package.h>

#include <yaml-cpp/node.h>
#include <yaml-cpp/emitter.h>
#include <yaml-cpp/parser.h>

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/concept_check.hpp>

#include <std_msgs/Empty.h>

#include <fstream>
#include <deque>

namespace config_server
{

class NotifyThread
{
public:
	void handleRequest()
	{
		pthread_mutex_lock(&m_mutex);
		while(m_jobs.size() == 0)
			pthread_cond_wait(&m_cond, &m_mutex);

		Job* job = &m_jobs[0];
		std::string subscriber = job->subscribers[job->subscribers.size()-1];
		job->subscribers.pop_back();
		std::string value = job->value;

		Parameter* param = job->param;

		if(job->subscribers.size() == 0)
			m_jobs.pop_front();
		pthread_mutex_unlock(&m_mutex);

		config_server::SetParameter srv;
		srv.request.name = param->desc.name;
		srv.request.value = value;

		if(!ros::service::call(subscriber, srv))
		{
			ROS_WARN("Could not call subscriber: '%s'", subscriber.c_str());
			pthread_mutex_lock(&param->mutex);
			std::remove(param->subscribers.begin(), param->subscribers.end(), subscriber);
			pthread_mutex_unlock(&param->mutex);
		}
	}

	void enqueue(Parameter* param, const std::string& value, const std::vector<std::string>& subscribers)
	{
		pthread_mutex_lock(&m_mutex);

		for(size_t i = 0; i < m_jobs.size(); ++i)
		{
			if(m_jobs[i].param == param)
			{
				m_jobs[i].subscribers = subscribers;
				m_jobs[i].value = value;
				pthread_cond_signal(&m_cond);
				pthread_mutex_unlock(&m_mutex);
				return;
			}
		}

		Job job;
		job.subscribers = subscribers;
		job.value = value;
		job.param = param;
		m_jobs.push_back(job);

		pthread_cond_signal(&m_cond);
		pthread_mutex_unlock(&m_mutex);
	}

	static NotifyThread* instance()
	{
		if(!m_instance)
			m_instance = new NotifyThread();

		return m_instance;
	}
private:
	struct Job
	{
		std::vector<std::string> subscribers;
		std::string value;
		Parameter* param;
	};

	std::deque<Job> m_jobs;
	pthread_cond_t m_cond;
	pthread_mutex_t m_mutex;

	static NotifyThread* m_instance;
	NotifyThread()
	{
	}
};

NotifyThread* NotifyThread::m_instance;

void Parameter::notify(const std::string& exclude)
{
	config_server::SetParameter srv;
	srv.request.name = desc.name;
	srv.request.value = value;

	if(subscribers.size() == 0)
		return;

	std::vector<std::string> filtered;
	filtered.reserve(subscribers.size());
	for(size_t i = 0; i < subscribers.size(); ++i)
	{
		if(subscribers[i] != exclude)
			filtered.push_back(subscribers[i]);
	}

	if(filtered.size() != 0)
		NotifyThread::instance()->enqueue(this, value, filtered);
}

ConfigServer::ConfigServer()
 : m_nh("~")
 , m_publishParamsCounter(0)
{
	m_srv_setParameter = m_nh.advertiseService("set_parameter", &ConfigServer::handleSetParameter, this);
	m_srv_getParameter = m_nh.advertiseService("get_parameter", &ConfigServer::handleGetParameter, this);
	m_srv_subscribe = m_nh.advertiseService("subscribe", &ConfigServer::handleSubscribe, this);
	m_srv_subscribeList = m_nh.advertiseService("subscribe_list", &ConfigServer::handleSubscribeList, this);

	m_pub_paramList = m_nh.advertise<ParameterList>("parameter_list", 1, true);

	m_srv_save = m_nh.advertiseService("save", &ConfigServer::handleSave, this);
	m_srv_load = m_nh.advertiseService("load", &ConfigServer::handleLoad, this);

	m_publishParamsTimer = m_nh.createTimer(ros::Duration(1.0), boost::bind(&ConfigServer::updateParameterList, this), true);

	if(!load(""))
	{
		ROS_FATAL("Could not load initial configuration");
		ros::shutdown();
	}
}

ConfigServer::~ConfigServer()
{
}

bool ConfigServer::handleSetParameter(SetParameterRequest& req, SetParameterResponse& resp)
{
	ParameterMap::iterator it = m_params.find(req.name);
	if(it == m_params.end())
	{
		ROS_ERROR("Request to set unknown parameter '%s'", req.name.c_str());
		return false;
	}

	it->second.value = req.value;
	it->second.notify(req.no_notify);
	return true;
}

bool ConfigServer::handleGetParameter(GetParameterRequest& req, GetParameterResponse& resp)
{
	ParameterMap::iterator it = m_params.find(req.name);
	if(it == m_params.end())
	{
		ROS_ERROR("Request to get unknown parameter '%s'", req.name.c_str());
		return false;
	}

	resp.value = it->second.value;
	return true;
}

bool ConfigServer::doSubscribe(const std::string& callback, const ParameterDescription& desc, std::string* value, bool* changed)
{
	ParameterMap::iterator it = m_params.find(desc.name);
	if(it == m_params.end())
	{
		ROS_INFO("New parameter '%s', default: '%s'", desc.name.c_str(), desc.default_value.c_str());
		std::pair<std::string, Parameter> value;
		value.first = desc.name;
		value.second.desc = desc;
		value.second.value = desc.default_value;

		it = m_params.insert(m_params.end(), value);

		*changed = true;
	}

	it->second.subscribers.push_back(callback);

	if(desc.type.length() != 0 /*&& (
		desc.type != it->second.desc.type
		|| desc.min != it->second.desc.min
		|| desc.max != it->second.desc.max
		|| desc.step != it->second.desc.step
	)*/)
	{
		it->second.desc = desc;
		if(changed)
			*changed = true;
	}

	*value = it->second.value;

	return true;
}

void ConfigServer::planUpdate()
{
	if(m_publishParamsCounter == 0)
	{
		m_publishParamsTimer.start();
		m_publishParamsCounter++;
		return;
	}
	else if(m_publishParamsCounter < 5)
	{
		m_publishParamsCounter++;
		return;
	}

	m_publishParamsTimer.stop();
	updateParameterList();
}

bool ConfigServer::handleSubscribe(SubscribeRequest& req, SubscribeResponse& resp)
{
	bool changed = false;
	if(!doSubscribe(req.callback, req.desc, &resp.value, &changed))
		return false;

	if(changed)
		planUpdate();

	return true;
}

bool ConfigServer::handleSubscribeList(SubscribeListRequest& req, SubscribeListResponse& resp)
{
	bool changed = false;

	resp.values.resize(req.parameters.size());
	for(size_t i = 0; i < req.parameters.size(); ++i)
	{
		if(!doSubscribe(req.callback, req.parameters[i], &resp.values[i], &changed))
			return false;
	}

	if(changed)
		planUpdate();

	return true;
}

void ConfigServer::updateParameterList()
{
	ParameterListPtr list = boost::make_shared<ParameterList>();

	list->parameters.reserve(m_params.size());

	for(ParameterMap::iterator it = m_params.begin(); it != m_params.end(); ++it)
	{
		list->parameters.push_back(it->second.desc);
	}

	m_pub_paramList.publish(list);

	m_publishParamsCounter = 0;
}

std::string ConfigServer::defaultConfigName()
{
	std::string robot_name;
	m_nh.param("robot_name", robot_name, std::string());

	if(robot_name.length() != 0)
		return "config_" + robot_name + ".yaml";
	else
		return "config.yaml";
}

typedef boost::tokenizer<boost::char_separator<char> > Tokenizer;

bool ConfigServer::handleSave(SaveRequest& req, SaveResponse& resp)
{
	std::vector<std::string> current_path;
	YAML::Emitter em;

	boost::char_separator<char> sep("/");

	em << YAML::BeginMap;
	for(ParameterMap::iterator it = m_params.begin(); it != m_params.end(); ++it)
	{
		Tokenizer tokenizer(it->first, sep);
		std::vector<std::string> tokens;
		BOOST_FOREACH(const std::string& t, tokenizer)
			tokens.push_back(t);

		std::string prop_name = tokens[tokens.size()-1];
		tokens.pop_back();

		int idx = 0;
		size_t tok_idx;

		for(tok_idx = 0; tok_idx < tokens.size(); ++tok_idx)
		{
			if(tok_idx >= current_path.size() || tokens[tok_idx] != current_path[idx])
				break;

			++idx;
		}

		// Pop everything till idx from stack
		for(int i = current_path.size()-1; i >= idx; --i)
		{
			em << YAML::EndMap;
			current_path.pop_back();
		}

		// Push new paths
		for(; tok_idx < tokens.size(); ++tok_idx)
		{
			em << YAML::Key << tokens[tok_idx];
			em << YAML::Value << YAML::BeginMap;

			current_path.push_back(tokens[tok_idx]);
		}

		em << YAML::Key << prop_name;
		em << YAML::Value << it->second.value;
	}
	em << YAML::EndMap;

	std::string prefix;
	m_nh.param("config_path", prefix, ros::package::getPath("config_server"));



	std::string fname;
	if(req.filename.length() != 0)
		fname = req.filename + ".yaml";
	else
		fname = defaultConfigName();

	std::ofstream out;
	out.open((prefix + "/" + fname).c_str());
	out << em.c_str() << "\n";
	return true;
}

bool ConfigServer::load(const std::string& filename)
{
	std::string prefix;
	m_nh.param("config_path", prefix, ros::package::getPath("config_server"));

	std::string fname;
	if(filename.length() != 0)
		fname = filename + ".yaml";
	else
		fname = defaultConfigName();

	fname = prefix + "/" + fname;

	std::ifstream stream(fname.c_str());
	YAML::Parser p(stream);
	YAML::Node n;

	if(!p.GetNextDocument(n))
	{
		ROS_ERROR_STREAM("Could not read config file '" << fname << "'");
		if(stream.fail())
			ROS_ERROR("The I/O error might have been: %s", strerror(errno));
		return false;
	}

	insertFromYAML(n, "");

	ROS_INFO("Loaded configuration file '%s'", fname.c_str());
	return true;
}

void ConfigServer::insertFromYAML(const YAML::Node& n, const std::string& path)
{
	if(n.Type() == YAML::NodeType::Map)
	{
		for(YAML::Iterator it = n.begin(); it != n.end(); ++it)
		{
			insertFromYAML(it.second(), path + "/" + it.first().to<std::string>());
		}
	}

	if(n.Type() == YAML::NodeType::Scalar)
	{
		if(!m_params.count(path))
			m_params[path] = Parameter();

		Parameter& p = m_params[path];
		p.desc.name = path;
		p.value = n.to<std::string>();
		p.notify();
	}
}

bool ConfigServer::handleLoad(LoadRequest& req, LoadResponse& resp)
{
	return load(req.filename);
}

static void* notify_thread(void*)
{
	NotifyThread* thread = NotifyThread::instance();

	while(1)
		thread->handleRequest();

	return 0;
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "config_server");

	config_server::NotifyThread::instance();

	pthread_t notifyThread;
	pthread_create(&notifyThread, NULL, config_server::notify_thread, NULL);

	config_server::ConfigServer server;

	ros::spin();

	return 0;
}

