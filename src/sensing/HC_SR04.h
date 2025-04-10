#ifndef HC_SR04_H
#define HC_SR04_H

#include "pico/stdlib.h"
#include <stdint.h>
#include <cstdio>

class HC_SR04 {
public:
    HC_SR04(uint8_t triggerPin, uint8_t echoPin);
    float measureDistance();
    double getLatestDistance() const { return latestDistance; }
    static void echo_irq_handler(uint gpio, uint32_t events, HC_SR04* instance);

private:
    uint8_t triggerPin;
    uint8_t echoPin;

    volatile uint64_t echo_start = 0;
    volatile uint64_t echo_end = 0;
    volatile bool valid_value = false;

    double latestDistance = 0.0; // Store the latest distance measurement

};

#endif // HC_SR04_H