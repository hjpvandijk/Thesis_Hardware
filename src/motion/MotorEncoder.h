// #ifndef MOTOR_ENCODER_H
// #define MOTOR_ENCODER_H

// #include <cstdint>
// #include "pico/stdlib.h"
// #include <cstdio>

// class MotorEncoder {
// public:
//     /**
//      * @brief Construct a new Motor Encoder object
//      * 
//      * @param pinA The GPIO pin for encoder channel A
//      * @param pinB The GPIO pin for encoder channel B
//      */
//     MotorEncoder(uint8_t pinA, uint8_t pinB);
    
//     /**
//      * @brief Initialize the encoder pins and interrupts
//      */
//     void init();
    
//     /**
//      * @brief Get the current count value
//      * 
//      * @return int32_t The current encoder count
//      */
//     int32_t getCount() const;
    
//     /**
//      * @brief Reset the encoder count to zero
//      */
//     void resetCount();
    
//     /**
//      * @brief Update the encoder count when an interrupt occurs
//      * 
//      * @param gpio The GPIO pin that triggered the interrupt
//      * @param events The event type that occurred
//      */
//     void handleInterrupt(uint gpio, uint32_t events);

// private:
//     uint8_t _pinA;
//     uint8_t _pinB;
//     volatile int32_t _count;
//     uint8_t _lastState;
    
//     static void gpioCallback(uint gpio, uint32_t events, void* userdata);
// };

// #endif // MOTOR_ENCODER_H