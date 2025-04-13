#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H


class MotorController {
public:
    MotorController(); // Constructor
    ~MotorController(); // Destructor

    void setLeftMotorSpeed(int speed); // Method to set motor speed
    void setRightMotorSpeed(int speed); // Method to set motor speed

private:
    unsigned int slice_left_1A, slice_left_1B, slice_right_1A, slice_right_1B;  // PWM slice numbers
    int currentSpeed; // Variable to store the current speed
    int pwmSpeed = 200; // Variable to store the PWM speed
};

#endif // MOTORCONTROLLER_H