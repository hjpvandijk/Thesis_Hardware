#include "DifferentialDrive.h"

DifferentialDrive::DifferentialDrive(){
    this->acceleration = 0.5;
    this->deceleration = 0.5;
    this->max_speed_turn = 0.5;
    this->turn_acceleration = 0.5;
    this->turn_deceleration = 0.5;
    this->current_speed_right = 0;
    this->current_speed_left = 0;

    //TODO: Init RPMs

    this->motorController = MotorController();

    //TODO: Init PID
}

DifferentialDrive::DifferentialDrive(float max_speed_straight,
                                     float max_speed_turn){
    this->max_speed_straight = max_speed_straight;
    this->max_speed_turn = max_speed_turn;
    
    this->acceleration = 0.5;
    this->deceleration = 0.5;
    this->max_speed_turn = 0.5;
    this->turn_acceleration = 0.5;
    this->turn_deceleration = 0.5;
    this->current_speed_right = 0;
    this->current_speed_left = 0;

    //TODO: Init RPMs

    this->motorController = MotorController();

    //TODO: Init PID
}

void DifferentialDrive::setSpeed(float speed_right, float speed_left) {
    // motorController.setRightMotorSpeed(speed_right);
    // motorController.setLeftMotorSpeed(speed_left);
}

void DifferentialDrive::forward() {
    setSpeed(forwardRPM, forwardRPM);
}

void DifferentialDrive::stop() {
    setSpeed(0, 0);
}

void DifferentialDrive::turnLeft() {
    setSpeed(-turnRPM, turnRPM);
}

void DifferentialDrive::turnRight() {
    setSpeed(turnRPM, -turnRPM);
}

//TODO: PID loop

