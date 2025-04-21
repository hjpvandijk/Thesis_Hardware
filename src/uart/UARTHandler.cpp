#include "UARTHandler.h"
#include <cstdio>
#include <cstring>
#include "../pins.h"
#include "hardware/irq.h"

#define UART_ID uart0
#define BAUD_RATE 9600

QueueHandle_t UARTHandler::rxQueue;

UARTHandler::~UARTHandler() {
    if (rxQueue != nullptr) {
        vQueueDelete(rxQueue);
    }
}

void UARTHandler::uart_irq_handler() {
    while (uart_is_readable(UART_ID)) {
        uint8_t byte = uart_getc(UART_ID);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(UARTHandler::rxQueue, &byte, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void UARTHandler::run() {
    printf("UARTHandler started (Interrupt-driven)\n");

    // Create FreeRTOS queue for received bytes
    rxQueue = xQueueCreate(32, sizeof(uint8_t)); // Adjust queue size as needed
    if (rxQueue == nullptr) {
        printf("Failed to create RX queue\n");
        return;
    }

    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Enable UART RX interrupt
    irq_set_exclusive_handler(UART0_IRQ, uart_irq_handler);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false); // Enable RX interrupt only

    uint8_t receivedByte;

    while (true) {
        if (xQueueReceive(rxQueue, &receivedByte, portMAX_DELAY) == pdTRUE) {
            // printf("Received from queue: %d, %02X\n", dataIndex, receivedByte);

            if (receivedByte == startByte) {
                dataIndex = 0; // Reset index on start byte
                dataBuffer[dataIndex++] = receivedByte;
            } else if (dataIndex > 0 && dataIndex < sizeof(dataBuffer)) {
                dataBuffer[dataIndex++] = receivedByte;
                if (receivedByte == endByte && dataIndex == sizeof(dataBuffer)) {
                    // Process the complete packet
                    float array[4];
                    // memcpy(array, &dataBuffer[1], sizeof(array)); // Skip start byte
                    for (int i = 0; i < 4; i++) {
                        memcpy(&array[i], &dataBuffer[i * sizeof(float) + 1], sizeof(float)); // Offset by 1 for start byte
                    }

                    position.x = array[0];
                    position.y = array[1];
                    heading = argos::CRadians(array[3]);
                    // printf("Position: (%f, %f), Heading: %f\n", position.x, position.y, heading);
                    dataIndex = 0; // Reset for the next packet
                } else if (dataIndex >= sizeof(dataBuffer)) {
                    printf("Packet too long or missing end byte. Resetting.\n");
                    dataIndex = 0; // Reset if packet exceeds expected size
                }
            } else {
                // Waiting for start byte
            }
            
        }
        // No need for vTaskDelay here as xQueueReceive will block until data arrives
    }
}

configSTACK_DEPTH_TYPE UARTHandler::getMaxStackSize() {
    return 1000; // 1000 words
}