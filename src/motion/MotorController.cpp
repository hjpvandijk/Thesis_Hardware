#include "MotorController.h"
#include "../pins.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"


MotorController::MotorController(){
    //Init for PWM
    gpio_init(LEFT_1A);
    gpio_set_function(LEFT_1A, GPIO_FUNC_PWM);
    gpio_init(LEFT_1B);
    gpio_set_function(LEFT_1B, GPIO_FUNC_PWM);
    gpio_init(RIGHT_1A);
    gpio_set_function(RIGHT_1A, GPIO_FUNC_PWM);
    gpio_init(RIGHT_1B);
    gpio_set_function(RIGHT_1B, GPIO_FUNC_PWM);

    // Get the PWM slice numbers for each pin
    slice_left_1A = pwm_gpio_to_slice_num(LEFT_1A);
    slice_left_1B = pwm_gpio_to_slice_num(LEFT_1B);
    slice_right_1A = pwm_gpio_to_slice_num(RIGHT_1A);
    slice_right_1B = pwm_gpio_to_slice_num(RIGHT_1B);
    
    // Set the PWM frequency (e.g., 20 kHz)
    pwm_set_clkdiv(slice_left_1A, 10.0f);  // Adjust this based on your desired frequency
    pwm_set_clkdiv(slice_left_1B, 10.0f);
    pwm_set_clkdiv(slice_right_1A, 10.0f);
    pwm_set_clkdiv(slice_right_1B, 10.0f);

    // Set the PWM wrap value (8-bit resolution: 255)
    pwm_set_wrap(slice_left_1A, 255);
    pwm_set_wrap(slice_left_1B, 255);
    pwm_set_wrap(slice_right_1A, 255);
    pwm_set_wrap(slice_right_1B, 255);

    // Enable PWM on each slice
    pwm_set_enabled(slice_left_1A, true);
    pwm_set_enabled(slice_left_1B, true);
    pwm_set_enabled(slice_right_1A, true);
    pwm_set_enabled(slice_right_1B, true);

    // Set initial speed to 0
    currentSpeed = 0;
    pwmSpeed = 0;
    setLeftMotorSpeed(0);
    setRightMotorSpeed(0);
    
}

MotorController::~MotorController() {}

void MotorController::setLeftMotorSpeed(int speed) {
    if (speed >= 0) {
        pwm_set_gpio_level(LEFT_1A, speed);
        pwm_set_gpio_level(LEFT_1B, 0);
    } else {
        speed = -speed;
        pwm_set_gpio_level(LEFT_1A, 0);
        pwm_set_gpio_level(LEFT_1B, speed);
    }
    
}

void MotorController::setRightMotorSpeed(int speed) {
    if (speed >= 0) {
        pwm_set_gpio_level(RIGHT_1A, speed);
        pwm_set_gpio_level(RIGHT_1B, 0);
    } else {
        speed = -speed;
        pwm_set_gpio_level(RIGHT_1A, 0);
        pwm_set_gpio_level(RIGHT_1B, speed);
    }
    
}
