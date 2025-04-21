#ifndef UARTHANDLER_H_
#define UARTHANDLER_H_

#include <stdio.h>
#include "../TaskAgent.h"

#include <pico/stdlib.h>
#include "hardware/uart.h"
#include <pico/time.h>
#include "../agent_implementation/utils/coordinate.h"
#include "angles.h"
#include "FreeRTOS.h"
#include "queue.h"

class UARTHandler : public TaskAgent {
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
    virtual void run();
    virtual configSTACK_DEPTH_TYPE getMaxStackSize();

private:
    static void uart_irq_handler();
    static QueueHandle_t rxQueue;
    Coordinate position = {0, 0};
    argos::CRadians heading = argos::CRadians::ZERO;
    uint8_t dataBuffer[18]; // Buffer to hold the complete packet
    int dataIndex = 0;
    uint8_t startByte = 0xAA;
    uint8_t endByte = 0x55;
};

#endif /* UARTHANDLER_H_ */