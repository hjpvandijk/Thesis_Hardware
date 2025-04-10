
#ifndef UARTHANDLER_H_
#define UARTHANDLER_H_

#include <stdio.h>
#include "../Agent.h"

#include <pico/stdlib.h>
#include "hardware/uart.h"
#include <pico/time.h>

class UARTHandler : public Agent{
public:
	UARTHandler() = default;
	virtual ~UARTHandler();
   

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
   

};

#endif /* UARTHANDLER_H_ */