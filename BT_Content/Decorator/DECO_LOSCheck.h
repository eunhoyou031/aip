#pragma once
#include "../../behaviortree_cpp_v3/action_node.h"
#include "../../behaviortree_cpp_v3/bt_factory.h"
#include "../../../Geometry/Vector3.h"
#include <iostream>
#include "../Functions.h"
#include "../BlackBoard/CPPBlackBoard.h"

using namespace BT;

namespace Action
{
	class DECO_LOSCheck : public SyncActionNode
	{
	private:


	public:


		DECO_LOSCheck(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
		{
		}

		~DECO_LOSCheck()
		{

		}

		static PortsList providedPorts();

		NodeStatus tick() override;
	};
}
