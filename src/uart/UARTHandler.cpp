#include "UARTHandler.h"
#include <cstdio>
#include <cstring>
#include "../pins.h"

#define UART_ID uart0
#define BAUD_RATE 57600



UARTHandler::~UARTHandler() {
	// TODO Auto-generated destructor stub
}

void UARTHandler::run() {
    printf("UARTHandler started\n");

    // Set up our UART with the r equired speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    uint8_t start_byte = 0xAA;
    uint8_t end_byte = 0x55;
    uint8_t data[4 * sizeof(float) + 2]; // Extra space for start and end bytes
    float array[4] = {0.0, 0.0, 0.0, 0.0};
    while (true) {
        if (uart_is_readable(UART_ID)) {
            if (uart_getc(UART_ID) == start_byte) {
                data[0] = start_byte; // Store start byte
                for (int i = 1; i < sizeof(data) ; i++) { // Start at 1 to skip start byte
                    while (!uart_is_readable(UART_ID)) vTaskDelay(10); // Wait for data
                    auto new_char = uart_getc(UART_ID);
                    data[i] = new_char;
                }
                // printf("Received data: ");
                // for (int i = 0; i < sizeof(data); i++) {
                //     printf("%02X ", data[i]);
                // }
                // printf("\n");
                // Check for end byte
                if (data[sizeof(data) - 1] == end_byte) {
                    // Copy data into array
                    for (int i = 0; i < 5; i++) {
                        memcpy(&array[i], &data[i * sizeof(float) + 1], sizeof(float)); // Offset by 1 for start byte
                    }

                    // Process the array as needed
                    this->position.x = array[0];
                    this->position.y = array[1];
                    this->heading = argos::CRadians(array[3]);

                    // printf("Position: (%f, %f)\n", this->position.x, this->position.y);
                    // printf("Heading: %f\n", this->heading);
                } else {
                    // Handle error
                    printf("End byte mismatch: expected %02X, got %02X\n", end_byte, data[sizeof(data) - 1]);
                }
            }
        }
        // Yield to other threads
        vTaskDelay((1000/60)/portTICK_PERIOD_MS); // 16Hz
    }
}

configSTACK_DEPTH_TYPE UARTHandler::getMaxStackSize() {
  return 1000; // 1000 words
}
