// URDF to RBDL parser
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef RBDL_PARSER_H
#define RBDL_PARSER_H

#include <string>

#include <rbdl/Model.h>

namespace urdf { class Model; class Joint; class Link; }

namespace rbdl_parser
{

class URDF_RBDL_Model : public RigidBodyDynamics::Model
{
public:
	URDF_RBDL_Model();
	virtual ~URDF_RBDL_Model();

	/**
	 * Parse an URDF model into a RBDL tree.
	 *
	 * This method also calls Init(), so you don't need to do that.
	 *
	 * @param root Decide which URDF link should be the root of the returned
	 *    RBDL tree. This can also be an URDF leaf if you want to reorder
	 *    your tree.
	 **/
	virtual bool initFrom(const urdf::Model& model, const std::string& root = "");

	int jointIndex(const std::string& name);
	std::string jointName(int index);
protected:

	/**
	 * Hook to get joint information from URDF.
	 *
	 * The default implementation does nothing. It might be interesting to
	 * override if you want to create a mapping between joint indexes and
	 * URDF joint names.
	 **/
	virtual void setupJoint(int index, const urdf::Joint& urdf, bool reverse);
private:
	void process(const urdf::Link& link, int parent, const RigidBodyDynamics::Math::SpatialTransform& parentJointFrame);
	void processReverse(const urdf::Link& link, int parent,
		const urdf::Link* parentLink,
		const RigidBodyDynamics::Math::SpatialTransform& parentJointFrame
	);

	typedef std::map<std::string, int> JointMap;
	JointMap m_jointMap;

	std::vector<std::string> m_jointNames;
};

}

#endif
