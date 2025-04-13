#ifndef IMPLEMENTATION_AND_EXAMPLES_BATTERYMANAGER_H
#define IMPLEMENTATION_AND_EXAMPLES_BATTERYMANAGER_H


#include <vector>
// #include <argos3/core/utility/math/vector2.h>
#include "vector2.h"
#include "MotionSystemBatteryManager.h"
#include "MicroControllerBatteryManager.h"

class BatteryManager {

public:
    struct Battery{
        float voltage;// = 6.0; //In Volts
        float capacity;// = 1000.0; //In mAh
        float charge;// = 1000.0; //In mAh

        float getStateOfCharge() const {
            return charge / capacity;
        }
    };

    BatteryManager(float robot_weight_kg, float robot_wheel_radius_m, float robot_inter_wheel_distance_m, float stall_torque_kg_cm, float no_load_rpm, float stall_current_A, float no_load_current_A, float battery_voltage, float battery_capacity);
    BatteryManager() = default;

    MotionSystemBatteryManager motionSystemBatteryManager;
    MicroControllerBatteryManager microControllerBatteryManager;

    Battery battery;

    //Most influential factors on battery life in the MIR100 robot:
    // according to https://www.mdpi.com/1424-8220/22/24/9861:
    //1. Number of turns
    //2. Traveled distance
    //3. Current state of charge
    //4. State of charge * distance

    //For the e-puck 2 robot:
    //Paper: Design and analysis of an E-Puck2 robot plug-in for the ARGoS
    //simulator
    //"We have evaluated robots moving at different speeds as we have
    //assumed that the main components draining the batteries were
    //the stepper motors."

    //We can calculate the power usage per speed and load per motor using the following formula:
    //P = (V * I) / 2
    //Where P is the power usage, V is the voltage and I is the current

    //We can calculate the current state of charge by measuring the voltage from the battery cell(s) and converting it to a percentage

    //So we need:
    //- Function to calculate power usage per speed and load per motor
    //      - P = Motor.EstimatePowerUsage(speed, load)
    //      -
    //      - But how do we know the load?
    //          - The load is related to the robot weight, wheel diameter, wheel friction, etc.

    //- Current state of charge --> Voltage from battery cell(s)
    //- Distance to travel: Euclidean at first --> Can we improve?


    //Get voltage from battery --> change to percentage
    //

    /**
     * Estimate how much power will be used by the IO board in the next given seconds
     * @return
     */
//    float EstimateBoardPowerUsage(float seconds);


    /***
     * Estimate how much power will be used in total over the given distance, with the given number of turns
     */
    std::tuple<float, float> estimateTotalPowerUsage(Agent* agent, std::vector<argos::CVector2> relativePath);

    std::tuple<float,float> estimateMotionPowerUsage(Agent* agent, std::vector<argos::CVector2> relativePath);
    std::tuple<float, float> calculateTotalPowerUsageFromMovement(Agent* agent, argos::CVector2 prevMovement, argos::CVector2 movement);
private:

};





#endif //IMPLEMENTATION_AND_EXAMPLES_BATTERYMANAGER_H