#include "BatteryManager.h"
// #include "agent.h"
#include "../../agent.h"


BatteryManager::BatteryManager(float robot_weight_kg, float robot_wheel_radius_m, float robot_inter_wheel_distance_m, float stall_torque_kg_cm, float no_load_rpm, float stall_current_A, float no_load_current_A, float battery_voltage, float battery_capacity) : microControllerBatteryManager() {
    this->motionSystemBatteryManager = MotionSystemBatteryManager(robot_weight_kg, robot_wheel_radius_m, robot_inter_wheel_distance_m, stall_torque_kg_cm, no_load_rpm, stall_current_A, no_load_current_A);
//    this->microControllerBatteryManager = MicroControllerBatteryManager();

    battery = Battery{battery_voltage, battery_capacity, battery_capacity};

}

/**
 * Estimate how much power will be used by the motors while traversing the given path in
 * Each section of the path is relative to the former in terms of direction, the first section is relative to the current heading of the agent
 * Considers both the power used by the motors and the power used by the microcontroller
 * @param agent
 * @param relativePath
 * @return
 */
std::tuple<float, float> BatteryManager::estimateTotalPowerUsage(Agent* agent, std::vector<argos::CVector2> relativePath){

    auto [motionSystemPowerUsage, pathFollowingDurationS] = motionSystemBatteryManager.estimateMotorPowerUsageAndDuration(agent, relativePath); //In Wh
    auto motionSystemPowerUsageAtVoltage = motionSystemPowerUsage / battery.voltage * 1000.0f;//In mAh


    auto microControllerPowerUsage = microControllerBatteryManager.estimateCommunicationConsumption(agent, pathFollowingDurationS); // In mAh
//    argos::LOG << "Motion system power usage: " << motionSystemPowerUsageAtVoltage << " mAh" << std::endl;
//    argos::LOG << "Microcontroller power usage: " << microControllerPowerUsage << " mAh" << std::endl;
    return {motionSystemPowerUsageAtVoltage + microControllerPowerUsage, pathFollowingDurationS};
}

std::tuple<float, float> BatteryManager::calculateTotalPowerUsageFromMovement(Agent* agent, argos::CVector2 prevMovement, argos::CVector2 movement){
    auto [motionSystemPowerUsage, pathFollowingDurationS] = motionSystemBatteryManager.estimateMotorPowerUsageAndDurationFromPastMovement(agent, prevMovement, movement, 1.0f/agent->ticks_per_second); //In Wh
    auto motionSystemPowerUsageAtVoltage = motionSystemPowerUsage / battery.voltage * 1000.0f;//In mAh


    auto microControllerPowerUsage = microControllerBatteryManager.estimateCommunicationConsumption(agent, pathFollowingDurationS); // In mAh
//    argos::LOG << "Motion system power usage: " << motionSystemPowerUsageAtVoltage << " mAh" << std::endl;
//    argos::LOG << "Microcontroller power usage: " << microControllerPowerUsage << " mAh" << std::endl;
    return {motionSystemPowerUsageAtVoltage + microControllerPowerUsage, pathFollowingDurationS};
}

std::tuple<float,float> BatteryManager::estimateMotionPowerUsage(Agent* agent, std::vector<argos::CVector2> relativePath){
    auto [motionSystemPowerUsage, pathFollowingDurationS] = motionSystemBatteryManager.estimateMotorPowerUsageAndDuration(agent, relativePath); //In Wh
    auto motionSystemPowerUsageAtVoltage = motionSystemPowerUsage / battery.voltage * 1000.0f;//In mAh

    return {motionSystemPowerUsageAtVoltage, pathFollowingDurationS};
}
