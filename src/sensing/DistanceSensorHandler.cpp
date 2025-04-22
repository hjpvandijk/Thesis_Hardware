#include "DistanceSensorHandler.h"
#include "../pins.h"
#include <cstdio>
#include "task.h"
#include <memory>
#include <semphr.h>



xSemaphoreHandle sensorsMutex = NULL;

DistanceSensorHandler::DistanceSensorHandler() : sensorsInitialized(false) {
    sensorsMutex = xSemaphoreCreateMutex();
    // Initialize the sensors
    sensors[0] = new HC_SR04(TRIG_FORWARD, ECHO_FORWARD);
    sensors[1] = new HC_SR04(TRIG_LEFT, ECHO_LEFT);
    sensors[2] = new HC_SR04(TRIG_BACK, ECHO_BACK);
    sensors[3] = new HC_SR04(TRIG_RIGHT, ECHO_RIGHT);
}

DistanceSensorHandler::~DistanceSensorHandler() {
    if (sensorsMutex != nullptr) {
        vSemaphoreDelete(sensorsMutex);
    }
    for (int i = 0; i < 4; ++i) {
        delete sensors[i];
    }
}

float DistanceSensorHandler::getDistance(int sensorIndex) {
    if (xSemaphoreTake(sensorsMutex, portMAX_DELAY) == pdTRUE) {
        if (!sensorsInitialized || sensorIndex < 0 || sensorIndex >= 4 || sensors[sensorIndex] == nullptr) {
            printf("Invalid sensor index or sensors not initialized\n");
            xSemaphoreGive(sensorsMutex);
            return -1.0f; // Return an invalid distance
        }
        float distance = sensors[sensorIndex]->getDistance();
        xSemaphoreGive(sensorsMutex);
        return distance;
    } else {
        printf("Failed to acquire mutex in getDistance\n");
        return -1.0f;
    }
}

void DistanceSensorHandler::run() {
    if (xSemaphoreTake(sensorsMutex, portMAX_DELAY) == pdTRUE) {
        sensorsInitialized = true;
        xSemaphoreGive(sensorsMutex);
    } else {
        printf("Failed to acquire mutex during initialization\n");
        // Handle initialization failure appropriately
        return;
    }

    int currentSensor = 0;

    while (true) {
        // printf("running on core %d\n", get_core_num());
        if (xSemaphoreTake(sensorsMutex, portMAX_DELAY) == pdTRUE) {
                float distance = sensors[currentSensor]->measureDistance();
                // if (currentSensor==1){
                    // printf("Distance from sensor %d: %f\n", currentSensor, distance);
                // }
                currentSensor = (currentSensor + 1) % 4; // Cycle through sensors
            xSemaphoreGive(sensorsMutex);
        } else {
            printf("Failed to acquire mutex for measurement\n");
            // Handle the failure to acquire the mutex (e.g., log an error)
        }
        vTaskDelay((1000 / ticks_per_second)/portTICK_PERIOD_MS); // Delay for the specified ticks per second
    }
}

configSTACK_DEPTH_TYPE DistanceSensorHandler::getMaxStackSize() {
    return 200;
}
