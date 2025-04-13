#ifndef IMPLEMENTATION_AND_EXAMPLES_MOTIONSYSTEMBATTERYMANAGER_H
#define IMPLEMENTATION_AND_EXAMPLES_MOTIONSYSTEMBATTERYMANAGER_H


#include <tuple>
// #include <argos3/core/utility/math/vector2.h>
#include "vector2.h"

class Agent;

class MotionSystemBatteryManager {
public:
    MotionSystemBatteryManager(float robot_weight_kg, float robot_wheel_radius_m, float robot_inter_wheel_distance_m, float stall_torque_kg_cm, float no_load_rpm, float stall_current_A, float no_load_current_A);
    MotionSystemBatteryManager() = default;

    float EstimateMotorPowerUsage(Agent *agent, float forces[], float forceDurations[]);
    std::tuple<float, float> estimateMotorPowerUsageAndDurationFromPastMovement(Agent *agent, argos::CVector2 prevMovement, argos::CVector2 movement, float time);

    std::tuple<float, float> estimateMotorPowerUsageAndDuration(Agent* agent, std::vector<argos::CVector2> & relativePath);

    float getMaxAchievableSpeed() const;

    void calculateVoltageAtSpeed(float speed_m_s);

private:

    //https://www.sgbotic.com/index.php?dispatch=products.view&product_id=2674
    //https://einstronic.com/product/tt-motor-yellow-geared-dc-motor/
    //https://www.verical.com/datasheet/adafruit-brushless-dc-motors-3777-5912007.pdf?srsltid=AfmBOooVLb6hED6HYiV6CVhpSCWuEZrjt1v1sG7b8DMTJIlFQQG7h-1Z
    //Based on the TT geared motor:
    //No load current 150mA @ 3V / 200mA @ 6V
    //At 3VDC we measured 150mA @ 120 RPM no-load, and 1.1 Amps when stalled
    //At 4.5VDC we measured 155mA @ 185 RPM no-load, and 1.2 Amps when stalled
    //At 6VDC we measured 160mA @ 250 RPM no-load, and 1.5 Amps when stalled
    //Stall Torque(3V): 0.4kg.cm
    //Stall Torque (6V): 0.8kg.cm
    //Min. Operating Speed (3V): 90RPM
    //Min. Operating Speed (6V): 200RPM



    float robot_weight_kg; //In kg
    float robot_wheel_radius_m; //In meters
    float robot_inter_wheel_distance_m; //In meters
    float rolling_force_without_acceleration_N; //In Newtons
    float stall_torque_kg_cm ; //In kg.cm @ 6V
    float no_load_rpm; //In RPM @ 6V
    float stall_current_A; //In Amps @ 6V
    float no_load_current_A ; //In Amps @ 6V

    float voltage_at_operating_speed; //In Volts
    float no_load_rpm_at_voltage;
    float no_load_current_at_voltage;
    float stall_current_at_voltage;
    float stall_torque_at_voltage;

    float rolling_resistance_coefficient = 0.01; //Typical value for rubber on concrete
    float speed_conversion = 2.0f * ARGOS_PI / 60.0f;
    float speed_conversion_inverse;

//    float forwardSpeed = 1.0;//Get speed from motor; //In m/s
//    float acceleration = 1.0;//Get acceleration from motor; //In m/s^2
//    float deceleration = 1.0;//Get deceleration from motor; //In m/s^2
//
//    float turnSpeed = 1.0;//Get turn speed from motor; //In rad/s
//    float turnAcceleration = 1.0;//Get turn acceleration from motor; //In rad/s^2
//    float turnDeceleration = 1.0;//Get turn deceleration from motor; //In rad/s^2

    void calculateForces(float (& forces)[6],Agent* agent) const;
    void calculateForcesPastMovement(float acceleration, float deceleration, float turnAcceleration, float turnDeceleration, float (& forces)[6], Agent* agent) const;

};


#endif //IMPLEMENTATION_AND_EXAMPLES_MOTIONSYSTEMBATTERYMANAGER_H