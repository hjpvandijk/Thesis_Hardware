#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "HC_SR04.h"
#include "pico/stdlib.h"
#include <algorithm> // For std::max
#include <pico/sync.h>


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

    // printf("GPIO %d IRQ: %d\n", gpio, events);

    if (xSemaphoreTakeFromISR(instance->dataMutex, NULL) == pdTRUE) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            instance->echo_start = current_time;
            // if (gpio == 12) printf("ECHO_LEFT RISE\n");
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            // if (gpio == 12) printf("ECHO_LEFT FALL\n");
            instance->echo_end = current_time;
            instance->valid_value = true;
        }
    xSemaphoreGiveFromISR(instance->dataMutex, NULL);
    } else {
        printf("Failed to acquire mutex in ISR\n");
    }
}

// Initialize the HC-SR04 sensor
HC_SR04::HC_SR04(uint8_t triggerPin, uint8_t echoPin) {
    this->triggerPin = triggerPin;
    this->echoPin = echoPin;
    this->echo_start = 0;
    this->echo_end = 0;
    this->valid_value = false;

    this->dataMutex = xSemaphoreCreateMutex();

    // Initialize the TRIGGER pin as output
    gpio_init(this->triggerPin);
    gpio_set_dir(this->triggerPin, GPIO_OUT);
    gpio_put(this->triggerPin, 0);

    // Initialize the ECHO pin as input
    gpio_init(this->echoPin);
    gpio_set_dir(this->echoPin, GPIO_IN);

    // Store this instance in the global array
    sensor_instances[echoPin] = this;

    // // Set up interrupts for the ECHO pin
    // // Only set the callback once
    static bool callback_set = false;
    if (!callback_set) {
        gpio_set_irq_callback(gpio_callback);
        callback_set = true;
    }
    
    gpio_set_irq_enabled(this->echoPin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}
// Measure distance using the HC-SR04 sensor, in meters
float HC_SR04::measureDistance(float distanceComp) {
    // Send a 10us pulse to the TRIGGER pin
    taskENTER_CRITICAL();
    gpio_put(this->triggerPin, 1);
    sleep_us(10);
    gpio_put(this->triggerPin, 0);
    this->valid_value = false;
    absolute_time_t start_time = get_absolute_time();
    taskEXIT_CRITICAL();
    // Wait for a valid measurement
    uint32_t timeout = 12000;//11600; // Timeout in microseconds (max range ~2m)

    while (!this->valid_value) {
        // Check if we've exceeded the timeout
        if (absolute_time_diff_us(start_time, get_absolute_time()) > timeout) {
            setDistance(2.0f); // Out of range or timeout
            return 2.0f;
        }
        // Yield to allow other threads to execute
        // sleep_us(100); // Small delay to reduce CPU usage and allow multitasking
        vTaskDelay(1);
    }

    // Calculate the distance
    uint64_t pulse_duration;
    
    if (xSemaphoreTake(this->dataMutex, portMAX_DELAY) == pdTRUE) {
        pulse_duration = this->echo_end - this->echo_start;
        xSemaphoreGive(this->dataMutex);
    } else {
        return 2.0f; // Failed to acquire mutex, return an error value
    }

    // Validate the measurement
    if (pulse_duration > timeout || pulse_duration < 100) {
        // Invalid measurement (too long or too short)
        setDistance(2.0f);
        return 2.0f;
    }

    // Calculate the distance in cm
    float distance = (pulse_duration / 2.0f) / 29.1f; // Speed of sound: 343m/s
    // printf("pulse_duration: %llu, distance: %f\n", pulse_duration, distance);

    float inM = distance / 100.0f + distanceComp; // Convert to meters, add compensation for sensor pos
    // printf("measured dist: %f, actual dist: %f\n", distance/100.0f, inM);
    setDistance(std::min(inM, 2.0f));;
    return inM;
}

// Measure distance using busy waiting, in meters
// float HC_SR04::measureDistance() {
//     uint64_t pulse_duration = 0;
//     bool echo_received = false;

//     // Send a 10us pulse to the TRIGGER pin
//     taskENTER_CRITICAL();
//     gpio_put(this->triggerPin, 1);
//     sleep_us(10);
//     gpio_put(this->triggerPin, 0);
//     absolute_time_t start_time = get_absolute_time();
//     taskEXIT_CRITICAL();

//     // Wait for the ECHO pin to go high with a timeout
//     uint32_t timeout = 12000; // Timeout in microseconds (max range ~2m)
//     while (!gpio_get(this->echoPin)) {
//         if (absolute_time_diff_us(start_time, get_absolute_time()) > timeout) {
//             setDistance(2.0f);
//             return 2.0f; // Timeout
//         }
//         vTaskDelay(1); // Yield to other tasks
//     }

//     // Measure the duration of the ECHO pulse
//     uint64_t echo_start_time = time_us_64();
//     while (gpio_get(this->echoPin)) {
//         if (absolute_time_diff_us(start_time, get_absolute_time()) > timeout) {
//             setDistance(2.0f);
//             return 2.0f; // Timeout during high pulse
//         }
//         vTaskDelay(1); // Yield to other tasks
//         echo_received = true;
//     }
//     uint64_t echo_end_time = time_us_64();

//     if (echo_received) {
//         pulse_duration = echo_end_time - echo_start_time;

//         // Validate the measurement
//         if (pulse_duration > timeout || pulse_duration < 100) {
//             setDistance(2.0f);
//             return 2.0f; // Invalid measurement
//         }

//         // Calculate the distance in cm
//         float distance_cm = (pulse_duration / 2.0f) / 29.1f; // Speed of sound: 343m/s
//         float distance_m = distance_cm / 100.0f;
//         setDistance(std::min(distance_m, 2.0f));
//         return distance_m;
//     } else {
//         setDistance(2.0f);
//         return 2.0f; // No echo received (should have timed out)
//     }
// }

double HC_SR04::getDistance() const { 
    // return this->latestDistance; 
    double result = 0.0;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      result = this->latestDistance;
      xSemaphoreGive(dataMutex);
    }
    return result; 
    }

void HC_SR04::setDistance(double distance) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        this->latestDistance = distance;
        xSemaphoreGive(dataMutex);
    }
}
