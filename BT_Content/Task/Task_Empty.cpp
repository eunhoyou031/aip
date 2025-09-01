#include "Task_Empty.h"

PortsList Action::Task_Empty::providedPorts()
{
	return {
			InputPort<CPPBlackBoard*>("BB")
	};
}

NodeStatus Action::Task_Empty::tick()
{
	Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

	(*BB)->VP_Cartesian = (*BB)->MyForwardVector * 1000.0f + (*BB)->MyLocation_Cartesian;

	return NodeStatus::SUCCESS;
}
