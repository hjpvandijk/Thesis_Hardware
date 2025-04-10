#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "HC_SR04.h"

// Global array to store pointers to HC_SR04 instances, indexed by GPIO pin
static HC_SR04* sensor_instances[30] = {nullptr}; // Assuming max GPIO is 29

// Global interrupt handler
static void gpio_callback(uint gpio, uint32_t events) {
    // Find the HC_SR04 instance for this GPIO pin
    HC_SR04* instance = sensor_instances[gpio];
    if (instance) {
        HC_SR04::echo_irq_handler(gpio, events, instance);
    }
}

// Interrupt handler for the ECHO pin
void HC_SR04::echo_irq_handler(uint gpio, uint32_t events, HC_SR04* instance) {
    uint64_t current_time = time_us_64();

    if (events & GPIO_IRQ_EDGE_RISE) {
        instance->echo_start = current_time;
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        instance->echo_end = current_time;
        instance->valid_value = true;
    }
}

// Initialize the HC-SR04 sensor
HC_SR04::HC_SR04(uint8_t triggerPin, uint8_t echoPin) {
    this->triggerPin = triggerPin;
    this->echoPin = echoPin;
    this->echo_start = 0;
    this->echo_end = 0;
    this->valid_value = false;

    // Initialize the TRIGGER pin as output
    gpio_init(this->triggerPin);
    gpio_set_dir(this->triggerPin, GPIO_OUT);
    gpio_put(this->triggerPin, 0);

    // Initialize the ECHO pin as input
    gpio_init(this->echoPin);
    gpio_set_dir(this->echoPin, GPIO_IN);

    // Store this instance in the global array
    sensor_instances[echoPin] = this;

    // Set up interrupts for the ECHO pin
    // Only set the callback once
    static bool callback_set = false;
    if (!callback_set) {
        gpio_set_irq_callback(gpio_callback);
        callback_set = true;
    }
    
    gpio_set_irq_enabled(this->echoPin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

// Measure distance using the HC-SR04 sensor, in meters
float HC_SR04::measureDistance() {
    // Send a 10us pulse to the TRIGGER pin
    gpio_put(this->triggerPin, 1);
    sleep_us(10);
    gpio_put(this->triggerPin, 0);

    this->valid_value = false;

    // Wait for a valid measurement
    uint32_t timeout = 11600; // Timeout in microseconds (max range ~2m)
    absolute_time_t start_time = get_absolute_time();

    while (!this->valid_value) {
        // Check if we've exceeded the timeout
        if (absolute_time_diff_us(start_time, get_absolute_time()) > timeout) {
            this->latestDistance = 2.0f; // Out of range or timeout
            return 2.0f;
        }
        // Small yield to allow other processing
        tight_loop_contents();
    }

   // Calculate the distance
    uint64_t pulse_duration = this->echo_end - this->echo_start;
    
    // Validate the measurement
    if (pulse_duration > timeout || pulse_duration < 100) {
        // Invalid measurement (too long or too short)
        this->latestDistance = 2.0f;
        return 2.0f;
    }
    // Calculate the distance in cm
    // uint64_t pulse_duration = this->echo_end - this->echo_start;
    float distance = (pulse_duration / 2.0f) / 29.1f; // Speed of sound: 343m/s

    float inM = distance / 100.0f; // Convert to meters
    this->latestDistance = inM;
    return inM;
}