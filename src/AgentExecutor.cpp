#include "AgentExecutor.h"

AgentExecutor::AgentExecutor(std::string id) {
    agent = Agent(std::move(id), 9.1, "config_yaml_data.h");
    agent.setPosition(0, 0);
}

AgentExecutor::~AgentExecutor() {}

/***
 * Get the static depth required in words
 * @return - words
 */
void AgentExecutor::run() {
    // printf("AgentExecutor task started\n");
    // agent.startMission();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        // printf("Agent %s running\n", agent.getId().c_str());
        agent.doStep();
        // vTaskDelay(1000 / agent.ticks_per_second);
        vTaskDelayUntil(&xLastWakeTime, (1000/agent.ticks_per_second) / portTICK_PERIOD_MS);
        // vTaskDelay(1000/10);
    }

}


configSTACK_DEPTH_TYPE AgentExecutor::getMaxStackSize() { return 5000; }
