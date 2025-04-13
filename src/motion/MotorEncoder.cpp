// #include "MotorEncoder.h"
// #include "pico/stdlib.h"
// #include "hardware/gpio.h"
// #include "hardware/irq.h"

// // Constructor
// MotorEncoder::MotorEncoder(uint8_t pinA, uint8_t pinB)
//     : _pinA(pinA), _pinB(pinB), _count(0), _lastState(0) {
// }

// // Initialize encoder pins and interrupts
// void MotorEncoder::init() {
//     // Initialize GPIO pins
//     gpio_init(_pinA);
//     gpio_init(_pinB);
    
//     // Set as inputs with pull-ups
//     gpio_set_dir(_pinA, GPIO_IN);
//     gpio_set_dir(_pinB, GPIO_IN);
//     gpio_pull_up(_pinA);
//     gpio_pull_up(_pinB);
    
//     // Enable interrupts on both pins for both rising and falling edges
//     gpio_set_irq_enabled_with_callback(_pinA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &MotorEncoder::gpioCallback);
//     gpio_set_irq_enabled(_pinB, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    
//     // Get initial state
//     _lastState = (gpio_get(_pinA) << 1) | gpio_get(_pinB);
// }

// // Get current encoder count
// int32_t MotorEncoder::getCount() const {
//     return _count;
// }

// // Reset encoder count to zero
// void MotorEncoder::resetCount() {
//     _count = 0;
// }

// // Handle GPIO interrupt
// void MotorEncoder::handleInterrupt(uint gpio, uint32_t events) {
//     // Read current state of both pins
//     uint8_t currentState = (gpio_get(_pinA) << 1) | gpio_get(_pinB);
    
//     // Determine direction based on state change
//     // This uses the fact that encoder transitions follow a specific pattern
//     if (_lastState == 0) {
//         if (currentState == 1) _count++;
//         else if (currentState == 2) _count--;
//     } else if (_lastState == 1) {
//         if (currentState == 3) _count++;
//         else if (currentState == 0) _count--;
//     } else if (_lastState == 2) {
//         if (currentState == 0) _count++;
//         else if (currentState == 3) _count--;
//     } else if (_lastState == 3) {
//         if (currentState == 2) _count++;
//         else if (currentState == 1) _count--;
//     }
    
//     // Update last state
//     _lastState = currentState;
// }

// // Static callback function for GPIO interrupts
// void MotorEncoder::gpioCallback(uint gpio, uint32_t events, void* userdata) {
//     // Find the encoder instance based on which GPIO triggered the interrupt
//     MotorEncoder* encoder = static_cast<MotorEncoder*>(userdata);
//     if (gpio == encoder->_pinA || gpio == encoder->_pinB) {
//         encoder->handleInterrupt(gpio, events);
//     }
// }