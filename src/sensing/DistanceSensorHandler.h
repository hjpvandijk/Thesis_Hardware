#ifndef DISTANCESENSORHANDLER_H
#define DISTANCESENSORHANDLER_H

#include "../TaskAgent.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"

#include <vector>
#include <string>
#include "queue.h"
#include "HC_SR04.h"


class DistanceSensorHandler : public TaskAgent{ 
public:
	DistanceSensorHandler();
	virtual ~DistanceSensorHandler();

   float getDistance(int sensorIndex);
   HC_SR04 *sensors[4];

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

    

private:
    int ticks_per_second = 100;
    bool sensorsInitialized;

};

#endif // DISTANCESENSORHANDLER_H