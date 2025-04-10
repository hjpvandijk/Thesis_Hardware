#ifndef DISTANCESENSORHANDLER_H
#define DISTANCESENSORHANDLER_H

#include "HC_SR04.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "../Agent.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"

#include <vector>
#include <string>
#include "queue.h"

class DistanceSensorHandler : public Agent{ 
public:
	DistanceSensorHandler();
	virtual ~DistanceSensorHandler();

   float getDistance(int sensorIndex);

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
    HC_SR04 *sensors[4];
    int ticks_per_second = 100;
    bool sensorsInitialized;

};

#endif // DISTANCESENSORHANDLER_H