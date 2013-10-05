// Preload library to enable UDP per default
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <string>
#include <dlfcn.h>

#include <boost/enable_shared_from_this.hpp>
#include <ros/transport_hints.h>
#include <ros/subscription.h>
#include <ros/network.h>
#include <ros/console.h>

#include <sys/types.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <udp_hinter_config.h>

/**
 * This hijacks the ros::Subscription::negotiateConnection() method from
 * roscpp. The original method is called via dlsym.
 *
 * We try to determine the route to the other host and whether the connection
 * is wireless or wired. Wireless connections are then defaulted to UDP.
 **/


std::string getIfaceForHost(const std::string& host)
{
	struct addrinfo hints;
	memset(&hints, 0, sizeof(hints));

	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;

	struct addrinfo* result;
	int s = getaddrinfo(host.c_str(), "11311", &hints, &result);
	if(s != 0)
	{
		fprintf(stderr, "[udp_hinter] Cannot resolve host '%s'. Resuming normal operation.\n", host.c_str());
		return std::string();
	}

	char addrbuf[256];
	if(inet_ntop(result->ai_family, &((sockaddr_in*)result->ai_addr)->sin_addr, addrbuf, sizeof(addrbuf)) == 0)
	{
		fprintf(stderr, "[udp_hinter] inet_ntop failed.\n");
		return std::string();
	}

	std::string cmd = "ip route get ";
	cmd += addrbuf;
	cmd += " | egrep -o 'dev \\w+' | awk '{ print $2 }'";

	FILE* f = popen(cmd.c_str(), "r");
	int count = fread(addrbuf, 1, sizeof(addrbuf)-1, f);
	if(count < 1)
	{
		fprintf(stderr, "[udp_hinter] Could not read from ip route\n");
		return std::string();
	}

	if(pclose(f) != 0)
	{
		fprintf(stderr, "[udp_hinter] Could not find route to publisher\n");
		return std::string();
	}

	addrbuf[count-1] = 0;

	return addrbuf;
}



namespace ros
{

typedef bool (*FuncType)(void* that, const std::string&);
FuncType g_realFunc = 0;

bool Subscription::negotiateConnection(const std::string& uri)
{
	std::string host;
	uint32_t port = 0;

	std::stringstream portstr;
	portstr << port;

	network::splitURI(uri, host, port);

	std::string iface = getIfaceForHost(host);

	// TODO: Better way of determining whether this is a WiFi connection
	if(iface.substr(0, 4) == "wlan")
	{
		if(transport_hints_.getTransports().empty())
		{
			fprintf(stderr,
				"[udp_hinter] It seems '%s' is published over WiFi from host '%s', enabling UDP per default\n",
				name_.c_str(), host.c_str()
			);
			transport_hints_.udp();
		}
	}

	if(!g_realFunc)
	{
		g_realFunc = (FuncType)dlsym(RTLD_NEXT, MANGLED_NEGOTIATE_CONNECTION);
		if(!g_realFunc)
		{
			fprintf(stderr, "[udp_hinter] Could not find real Subscription::negotiateConnection() function");
			abort();
		}
	}

	return g_realFunc(this, uri);
}

}
