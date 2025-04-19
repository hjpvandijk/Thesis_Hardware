
#ifndef UARTHANDLER_H_
#define UARTHANDLER_H_

#include <stdio.h>
#include "../TaskAgent.h"

#include <pico/stdlib.h>
#include "hardware/uart.h"
#include <pico/time.h>
#include "../agent_implementation/utils/coordinate.h"
#include "angles.h"

class UARTHandler : public TaskAgent{
public:
	UARTHandler() = default;
	virtual ~UARTHandler();

	Coordinate getPosition() const {
		return this->position;
	}

	argos::CRadians getHeading() const {
		return this->heading;
	}
   

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
	Coordinate position = {0, 0};
	argos::CRadians heading = argos::CRadians::ZERO;
   

};

#endif /* UARTHANDLER_H_ */