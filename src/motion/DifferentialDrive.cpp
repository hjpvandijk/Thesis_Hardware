#include "DifferentialDrive.h"
#include <cstdio>

int current_dir = 0; //0=fwd, 1=left, 2=right
int prev_dir = 0;
int i_on = 0;
bool on = true;


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

DifferentialDrive::DifferentialDrive(std::string id, float max_speed_straight,
                                     float max_speed_turn){
    this->max_speed_straight = max_speed_straight;
    this->max_speed_turn = max_speed_turn;

    if (id == "0D351F00F5DF8253"){
        this->forwardRPM_L = 230;
        this->forwardRPM_R = 230;
        this->turnRPM_L = 230;
        this->turnRPM_R = 230;
    } 
    // else if (id == "12F959FD3BDFD31D"){
    //     this->forwardRPM_L = 210;
    //     this->forwardRPM_R = 210;
    //     this->turnRPM_L = 210;
    //     this->turnRPM_R = 210;
    // } else {
    //     this->forwardRPM_L = 220;
    //     this->forwardRPM_R = 220;
    //     this->turnRPM_L = 220;
    //     this->turnRPM_R = 220;
    // }
    
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
    if (current_dir != prev_dir)
        i_on = 0;
    

    auto soft_start_factor = i_on / 3.0f;
    // if (on) {
    // printf("Direction: %d, i_on: %d, pwm_r: %f\n", current_dir, i_on, speed_right * soft_start_factor);
    motorController.setRightMotorSpeed(speed_right * (soft_start_factor));
    motorController.setLeftMotorSpeed(speed_left * soft_start_factor);
    // motorController.setRightMotorSpeed(speed_right);
    // motorController.setLeftMotorSpeed(speed_left);

    // } else {
    //     motorController.setRightMotorSpeed(0);
    //     motorController.setLeftMotorSpeed(0);
    // }
    if (i_on < 3){
        i_on++;
    }
    prev_dir = current_dir;

    // if (on){
    //     motorController.setRightMotorSpeed(speed_right);
    //     motorController.setLeftMotorSpeed(speed_left);
    //     // printf("ON");
    // } else {
    //     motorController.setRightMotorSpeed(0);
    //     motorController.setLeftMotorSpeed(0);
    //     // printf("OFF");
    // }

    // if (i_on < 0){
    //     i_on++;
    // } else {
    //     on = !on;
    //     i_on = 0;
    // }

}

void DifferentialDrive::forward() {
    current_dir = 0;
    // printf("Forward with speeds %f, %f\n", forwardRPM_R, forwardRPM_L);
    setSpeed(forwardRPM_R, forwardRPM_L);
}

void DifferentialDrive::stop() {
    setSpeed(0, 0);
}

void DifferentialDrive::turnLeft() {
    current_dir = 1;
    // printf("Left with speeds %f, %f\n", -turnRPM_R, turnRPM_L);
    setSpeed(turnRPM_R, -turnRPM_L);
}

void DifferentialDrive::turnRight() {
    current_dir = 2;
    // printf("Right with speeds %f, %f\n", turnRPM_R, -turnRPM_L);
    setSpeed(-turnRPM_R, turnRPM_L);
}

//TODO: PID loop

