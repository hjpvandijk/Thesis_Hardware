#include "UARTHandler.h"
#include <cstdio>
#include <cstring>
#include "../pins.h"

#define UART_ID uart0
#define BAUD_RATE 9600



UARTHandler::~UARTHandler() {
	// TODO Auto-generated destructor stub
}

void UARTHandler::run() {

    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    uint8_t start_byte = 0xAA;
    uint8_t end_byte = 0x55;
    uint8_t data[4 * sizeof(float) + 2]; // Extra space for start and end bytes
    float array[4] = {0.0, 0.0, 0.0, 0.0};

    while (1){

        while (uart_getc(UART_ID) != start_byte);
        for (int i = 1; i < sizeof(data) - 1; i++) { // Start at 1 to skip start byte
            data[i] = uart_getc(UART_ID);
        }
        // Check for end byte
        if (data[sizeof(data) - 1] != end_byte) {
            // Handle error
        }
        // Copy data into array
        for (int i = 0; i < 5; i++) {
            memcpy(&array[i], &data[i * sizeof(float) + 1], sizeof(float)); // Offset by 1 for start byte
        }

        // Process the array as needed
        printf("Received data: x: %f, y: %f, z: %f, yaw: %f\n", array[0], array[1], array[2], array[3]);
    }
}

configSTACK_DEPTH_TYPE UARTHandler::getMaxStackSize() {
  return 1000; // 1000 words
}
