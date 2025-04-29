//
// Created by hugo on 7-4-25.
//

#ifndef DIFFERENTIALDRIVE_H_
#define DIFFERENTIALDRIVE_H_
#include "MotorController.h"
#include <string>


class DifferentialDrive {
public:
    //m/s
    float max_speed_straight;
    float acceleration;
    float deceleration;

    //Rad/s
    float max_speed_turn; //Per wheel
    float turn_acceleration;
    float turn_deceleration;


    double current_speed_right;
    double current_speed_left;

    DifferentialDrive();
    DifferentialDrive(std::string id, float max_speed_straight, float max_speed_turn);


//    void setActuator(argos::CCI_PiPuckDifferentialDriveActuator *differentialDriveActuator);
    void setSpeed(float speed_right, float speed_left);
    void forward();
    void stop();
    void turnLeft();
    void turnRight();

    void PIDLoop();


private:
    MotorController motorController;
    double forwardRPM_L = 210;//210;   // Target RPM (Adjust this to your desired RPM)
    double forwardRPM_R = 210;//210;
    double turnRPM_L = 210;//220;
    double turnRPM_R = 210;//220;

    
    
    
    double currentRPMA = 0; // Measured RPM
    double currentRPMB = 0;
    double motorSpeedA = 220; // PWM value (0-255)
    double motorSpeedB = 220;

    
    
    volatile int pulseCountA = 0;
    
    
    volatile int pulseCountB = 0;
    
    
    unsigned long prevTime = 0;
    
    
    int slotsPerRevolution = 20;  // Adjust based on your slotted wheel
    
    
    int pulseCountMs = 1000;

    // PID parameters (tune these for best results)
    // double Kp = 0.75;
    // double Ki = 0.2;
    // double Kd = 0.0;
    // PID* rpmPIDA;
    // PID* rpmPIDB;
};


#endif //DIFFERENTIALDRIVE_H_
