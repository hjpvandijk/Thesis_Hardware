#ifndef HC_SR04_H
#define HC_SR04_H

#include <stdint.h>
#include <cstdio>
#include "FreeRTOS.h"
#include <semphr.h>

class HC_SR04 {
public:
    HC_SR04() = default; // Default constructor
    HC_SR04(uint8_t triggerPin, uint8_t echoPin);
    float measureDistance();
    double getDistance() const;
    void setDistance(double distance);
    static void echo_irq_handler(uint gpio, uint32_t events, HC_SR04* instance);

private:
    uint8_t triggerPin;
    uint8_t echoPin;

    volatile uint64_t echo_start = 0;
    volatile uint64_t echo_end = 0;
    volatile bool valid_value = false;

    double latestDistance = 0.0; // Store the latest distance measurement
    mutable SemaphoreHandle_t dataMutex;

};

#endif // HC_SR04_H