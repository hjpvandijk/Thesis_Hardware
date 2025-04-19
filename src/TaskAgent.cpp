/*
 * Agent.cpp
 * Abstract agent interface to an active agent object that runs as
 * FreeRTOS task
 *  Created on: 15 Aug 2022
 *      Author: jondurrant
 */

#include "TaskAgent.h"
#include <string.h>
#include <stdio.h>

/***
 * Constructor
 */
TaskAgent::TaskAgent() {
	// NOP

}

/***
 * Destructor
 */
TaskAgent::~TaskAgent() {
	stop();
}

/***
 * Stop task
 * @return
 */
void TaskAgent::stop(){
	if (xHandle != NULL){
		vTaskDelete(  xHandle );
		xHandle = NULL;
	}
}


/***
* Get high water for stack
* @return close to zero means overflow risk
*/
unsigned int TaskAgent::getStakHighWater(){
	if (xHandle != NULL)
		return uxTaskGetStackHighWaterMark(xHandle);
	else
		return 0;
}


/***
* Get the FreeRTOS task being used
* @return
*/
TaskHandle_t TaskAgent::getTask(){
	return xHandle;
}


/***
 * Start the task
 * @param priority - Priority to apply to process
 * @return
 */
bool TaskAgent::start(const char *name, UBaseType_t priority){
	BaseType_t res;

	if (strlen(name) >= MAX_NAME_LEN){
		memcpy(pName, name, MAX_NAME_LEN);
		pName[MAX_NAME_LEN-1]=0;
	} else {
		strcpy(pName, name);
	}
	res = xTaskCreate(
			TaskAgent::vTask,       /* Function that implements the task. */
		pName,   /* Text name for the task. */
		getMaxStackSize(),             /* Stack size in words, not bytes. */
		( void * ) this,    /* Parameter passed into the task. */
		priority,/* Priority at which the task is created. */
		&xHandle
	);

	if (res != pdPASS){
		printf("Failed to create task %s, error %d\n", pName, res);
	} else {
		printf("Created task %s\n", pName);
	}
	return (res == pdPASS);
}



/***
 * Internal function used by FreeRTOS to run the task
 * @param pvParameters
 */
 void TaskAgent::vTask( void * pvParameters ){
	 TaskAgent *task = (TaskAgent *) pvParameters;
	 if (task != NULL){
		 task->run();
	 }
 }