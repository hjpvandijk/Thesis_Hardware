#ifndef AGENT_EXECUTOR_H
#define AGENT_EXECUTOR_H

#include "TaskAgent.h"
#include <cstdio>
#include <string>
#include "agent_implementation/agent.h"
#include "uart/UARTHandler.h"

class AgentExecutor : public TaskAgent{
public:
	AgentExecutor(std::string id);
	virtual ~AgentExecutor();

    std::string id{};
    Agent agent;

protected:

	/***
	 * Run loop for the agent.
	 */
	virtual void run();


	/***
	 * Get the static depth required in words
	 * @return - words
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();

};

#endif // AGENT_EXECUTOR_H