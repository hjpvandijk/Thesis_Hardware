// #include <argos3/core/utility/logging/argos_log.h>
#include "MotionSystemBatteryManager.h"
#include "../../agent.h"

MotionSystemBatteryManager::MotionSystemBatteryManager(float robot_weight_kg, float robot_wheel_radius_m,
                                                       float robot_inter_wheel_distance_m,
                                                       float stall_torque_kg_cm,
                                                       float no_load_rpm, float stall_current_A,
                                                       float no_load_current_A) {
    this->robot_weight_kg = robot_weight_kg;
    this->robot_wheel_radius_m = robot_wheel_radius_m;
    this->robot_inter_wheel_distance_m = robot_inter_wheel_distance_m;
    this->rolling_force_without_acceleration_N = rolling_resistance_coefficient * robot_weight_kg * 9.81f;
    this->stall_torque_kg_cm = stall_torque_kg_cm;
    this->no_load_rpm = no_load_rpm;
    this->stall_current_A = stall_current_A;
    this->no_load_current_A = no_load_current_A;
    this->speed_conversion *= robot_wheel_radius_m;
    this->speed_conversion_inverse = 1.0f / this->speed_conversion;
}

void MotionSystemBatteryManager::calculateForces(float (& forces)[6], Agent* agent) const {
    //(assuming equal weight distribution over the robot)
    float momentOfInertia = 0.5f * robot_weight_kg * agent->config.ROBOT_RADIUS * agent->config.ROBOT_RADIUS; //In kg.m^2 , 0.0362 is the robot radius

    float accelerationForce = robot_weight_kg * agent->differential_drive.acceleration; //In Newtons
    float decelerationForce = robot_weight_kg * agent->differential_drive.deceleration; //In Newtons
    float turnAccelerationForce =
           2 * momentOfInertia * agent->differential_drive.turn_acceleration / agent->config.ROBOT_INTER_WHEEL_DISTANCE; //In Newtons
    float turnDecelerationForce =
            2 *momentOfInertia * agent->differential_drive.turn_deceleration / agent->config.ROBOT_INTER_WHEEL_DISTANCE; //In Newtons

    float totalForceDuringAcceleration = rolling_force_without_acceleration_N + accelerationForce; //In Newtons
    float totalForceDuringDeceleration = rolling_force_without_acceleration_N + decelerationForce; //In Newtons
    float totalForceDuringTurnAcceleration = rolling_force_without_acceleration_N + turnAccelerationForce; //In Newtons
    float totalForceDuringTurnDeceleration = rolling_force_without_acceleration_N + turnDecelerationForce; //In Newtons

    forces[0] = totalForceDuringAcceleration; //In Newtons
    forces[1] = rolling_force_without_acceleration_N; //In Newtons
    forces[2] = totalForceDuringDeceleration; //In Newtons
    forces[3] = totalForceDuringTurnAcceleration; //In Newtons
    forces[4] = rolling_force_without_acceleration_N; //In Newtons
    forces[5] = totalForceDuringTurnDeceleration; //In Newtons
}

void MotionSystemBatteryManager::calculateForcesPastMovement(float acceleration, float deceleration, float turnAcceleration, float turnDeceleration, float (& forces)[6], Agent* agent) const {
    //(assuming equal weight distribution over the robot)
    float momentOfInertia = 0.5f * robot_weight_kg * agent->config.ROBOT_RADIUS * agent->config.ROBOT_RADIUS; //In kg.m^2 , 0.0362 is the robot radius

    float accelerationForce = robot_weight_kg * acceleration; //In Newtons
    float decelerationForce = robot_weight_kg * deceleration; //In Newtons
    float turnAccelerationForce =
            2 * momentOfInertia * turnAcceleration / agent->config.ROBOT_INTER_WHEEL_DISTANCE; //In Newtons
    float turnDecelerationForce =
            2 * momentOfInertia * turnDeceleration / agent->config.ROBOT_INTER_WHEEL_DISTANCE; //In Newtons

    float totalForceDuringAcceleration = rolling_force_without_acceleration_N + accelerationForce; //In Newtons
    float totalForceDuringDeceleration = rolling_force_without_acceleration_N + decelerationForce; //In Newtons
    float totalForceDuringTurnAcceleration = rolling_force_without_acceleration_N + turnAccelerationForce; //In Newtons
    float totalForceDuringTurnDeceleration = rolling_force_without_acceleration_N + turnDecelerationForce; //In Newtons

    forces[0] = totalForceDuringAcceleration; //In Newtons
    forces[1] = rolling_force_without_acceleration_N; //In Newtons
    forces[2] = totalForceDuringDeceleration; //In Newtons
    forces[3] = totalForceDuringTurnAcceleration; //In Newtons
    forces[4] = rolling_force_without_acceleration_N; //In Newtons
    forces[5] = totalForceDuringTurnDeceleration; //In Newtons
}


float
MotionSystemBatteryManager::EstimateMotorPowerUsage(Agent *agent, float forces[], float forceDurations[]) {
    float totalPowerUsage = 0.0; //In Wh

    //We are assuming the motors provide the same torque (and current draw) in both directions
    for (int i = 0; i < 6; i++) {
        float force = forces[i]; //In Newtons
        float forceDuration = forceDurations[i] / 3600; //In hours
        //Add rolling resistnace
        float force_per_wheel = force / 2; //In Newtons
        float torque_per_motor = force_per_wheel * robot_wheel_radius_m; //In N.M
        float torque_per_motor_kg_cm = torque_per_motor * 10.1971621f; //In kg.cm
        //Now we know the torque, we can estimate the current draw

        //If we are not accelerating or decelerating, we can calculate the speed at said torque, and adjust the torque if the speed is higher than the max speed
        if(i == 1 || i == 4) {
            //Torque and speed are negatively linearly related: 0 torque at max speed, 0 speed at max torque (stall)
            float rpm_at_torque =
                    no_load_rpm_at_voltage - (torque_per_motor_kg_cm / stall_torque_at_voltage) * no_load_rpm_at_voltage; //In RPM @ 6V

            float speed = rpm_at_torque * speed_conversion;//In m/s
            //Make sure the speed is not higher than the max speed, this shouldn't happen
            assert(agent->differential_drive.max_speed_straight <= speed);
            //If the achievable speed is higher than the speed we want, we need to adjust the speed and torque
            if (agent->differential_drive.max_speed_straight < speed) {
                speed = agent->differential_drive.max_speed_straight;
                //Calculate the torque at this speed
                float rpm_at_speed = speed * speed_conversion_inverse; //In RPM @ 6V
                float torque_at_speed = (1 - rpm_at_speed / no_load_rpm_at_voltage) * stall_torque_at_voltage; //In kg.cm
                torque_per_motor_kg_cm = torque_at_speed;
            }
        }
        //Calculate the current using the torque
        float current_draw_A_per_motor = no_load_current_at_voltage + (stall_current_at_voltage - no_load_current_at_voltage) *
                                                                      (torque_per_motor_kg_cm / stall_torque_at_voltage); //In Amps @ 6V

        //Calculate the power usage for both motors together
        float total_power = current_draw_A_per_motor * 2 * this->voltage_at_operating_speed; //In watts

        totalPowerUsage += total_power * forceDuration; //In Wh
    }


    return totalPowerUsage;
}

std::tuple<float, float> MotionSystemBatteryManager::estimateMotorPowerUsageAndDuration(Agent *agent,
                                                                                        std::vector<argos::CVector2> & relativePath) {
    //Estimate how long it will take the agent to travel the path

    auto max_speed_turn_rad_s =
            agent->differential_drive.max_speed_turn / this->robot_inter_wheel_distance_m; //0.0565 is inter-wheel distance in meters (pipuck)

    float totalMotorPowerUsage = 0;
    float totalDuration = 0;
    //                                      rad/s                                           rad/s                                           rad/s/s
    float angleToAccelerateTurn =
            max_speed_turn_rad_s * max_speed_turn_rad_s / (2 * agent->differential_drive.turn_acceleration); //In rad
    float angleToDecelerateTurn =
            max_speed_turn_rad_s * max_speed_turn_rad_s / (2 * agent->differential_drive.turn_deceleration); //In rad

    float distanceToAccelerateDrive =
            agent->differential_drive.max_speed_straight * agent->differential_drive.max_speed_straight /
            (2 * agent->differential_drive.acceleration); //In meters
    float distanceToDecelerateDrive =
            agent->differential_drive.max_speed_straight * agent->differential_drive.max_speed_straight /
            (2 * agent->differential_drive.deceleration); //In meters

    float forces[6];
    calculateForces(forces, agent);

    for (int i = 0; i < relativePath.size(); i++) {
        argos::CVector2 vector = relativePath[i];
        float distance = vector.Length(); //In meters
        float turnAngle = std::abs(
                vector.Angle().GetValue()); //In Radians, we assume both directions yield the same power usage. This angle is relative to the previous vector
        bool cameToAStop = false;
        if (turnAngle > agent->config.TURN_THRESHOLD_DEGREES) cameToAStop = true;
        bool willComeToAStop = false;
        if (i < relativePath.size()-1){ //If we are not at the end of the path
            argos::CVector2 nextVector = relativePath[i+1];
            float nextTurnAngle = std::abs(nextVector.Angle().GetValue());
            if (nextTurnAngle > agent->config.TURN_THRESHOLD_DEGREES) willComeToAStop = true;
        } //We don't know what happens next yet, so we don't assume we will come to a stop

        //Now we know the distance and the angle, we can estimate the time it will take to travel this path


        //We can estimate the time it will take to turn
        float totalTurnTime = 0.0;

        float timeToAccelerateTurn = 0.0;
        float timeToDecelerateTurn = 0.0;
        float timeToConstantTurn = 0.0;
        float timeToAccelerateDrive = 0.0;
        float timeToDecelerateDrive = 0.0;
        float timeToConstantDrive = 0.0;

        if (angleToAccelerateTurn + angleToDecelerateTurn > turnAngle) {
            //We can't accelerate and decelerate in time
            float maxAchievedTurnSpeed = sqrt(2 * turnAngle / (1 / agent->differential_drive.turn_acceleration + 1 /
                                                                                                                 agent->differential_drive.turn_deceleration)); //In rad/s
            timeToAccelerateTurn = maxAchievedTurnSpeed / agent->differential_drive.turn_acceleration; //In seconds
            timeToDecelerateTurn = maxAchievedTurnSpeed / agent->differential_drive.turn_deceleration; //In seconds
        } else {
            //We can accelerate and decelerate in time
            timeToAccelerateTurn = max_speed_turn_rad_s / agent->differential_drive.turn_acceleration; //In seconds
            timeToDecelerateTurn = max_speed_turn_rad_s / agent->differential_drive.turn_deceleration; //In seconds
            float angleToTravelTurn = turnAngle - angleToAccelerateTurn - angleToDecelerateTurn; //In rad
            timeToConstantTurn = angleToTravelTurn / max_speed_turn_rad_s; //In seconds
        }
        totalTurnTime = timeToAccelerateTurn + timeToDecelerateTurn + timeToConstantTurn; //In seconds

        //We can estimate the time it will take to drive
        //We can estimate the time it will take to travel the distance
        float totalDriveTime = 0.0;

        //If we didn't come to a stop, we don't accelerate
        if (!cameToAStop) {
            distanceToAccelerateDrive = 0;
        }
        //If we won't come to a stop, we don't decelerate
        if (!willComeToAStop) {
            distanceToDecelerateDrive = 0;
        }
        if (distanceToAccelerateDrive + distanceToDecelerateDrive > distance) {
            //We can't accelerate and decelerate in time
            float maxAchievedDriveSpeed = sqrt(2 * distance / (1 / agent->differential_drive.acceleration +
                                                               1 / agent->differential_drive.deceleration)); //In rad/s
            timeToAccelerateDrive = maxAchievedDriveSpeed / agent->differential_drive.acceleration; //In seconds
            timeToDecelerateDrive = maxAchievedDriveSpeed / agent->differential_drive.deceleration; //In seconds
        } else {
            //We can accelerate and decelerate in time
            timeToAccelerateDrive = distanceToAccelerateDrive == 0 ? 0 :
                                    agent->differential_drive.max_speed_straight / agent->differential_drive.acceleration; //In seconds
            timeToDecelerateDrive = distanceToDecelerateDrive == 0 ? 0 :
                                    agent->differential_drive.max_speed_straight / agent->differential_drive.deceleration; //In seconds
            float distanceToTravelDrive = distance - distanceToAccelerateDrive - distanceToDecelerateDrive; //In meters
            timeToConstantDrive = distanceToTravelDrive / agent->differential_drive.max_speed_straight; //In seconds
        }
        totalDriveTime = timeToAccelerateDrive + timeToDecelerateDrive + timeToConstantDrive; //In seconds

        totalDuration += totalDriveTime + totalTurnTime;

        float forceDurations[] = {timeToAccelerateDrive, timeToConstantDrive, timeToDecelerateDrive, timeToAccelerateTurn, timeToConstantTurn,
                                  timeToDecelerateTurn};

        //We can also estimate the power usage
        float motorPowerUsage = EstimateMotorPowerUsage(agent, forces, forceDurations); //In Wh
        totalMotorPowerUsage += motorPowerUsage;
    }

    return {totalMotorPowerUsage, totalDuration};
}

std::tuple<float, float> MotionSystemBatteryManager::estimateMotorPowerUsageAndDurationFromPastMovement(Agent *agent, argos::CVector2 prevMovement, argos::CVector2 movement, float time) {
    //Estimate how long it will take the agent to travel the path

    auto max_speed_turn_rad_s =
            agent->differential_drive.max_speed_turn / 0.0565f; //0.0565 is inter-wheel distance in meters

    float totalDuration = 0;
    float distance = movement.Length(); //In meters
    float turnAngle = std::abs(
            movement.Angle().GetValue()); //In Radians, we assume both directions yield the same power usage

    double averageSpeed = distance / time;
//    argos::LOG << distance << "/" << time << "=" << averageSpeed << std::endl;
    double averageTurnSpeed = turnAngle / time;
//    argos::LOG << turnAngle << "/" << time << "=" << averageTurnSpeed << std::endl;

    double averageSpeedPrev = prevMovement.Length() / time;
    double averageTurnSpeedPrev = prevMovement.Angle().GetValue() / time;

    float acceleration = (averageSpeed - averageSpeedPrev) / time;
    float turnAcceleration = (averageTurnSpeed - averageTurnSpeedPrev) / time;
    float deceleration = 0;
    float turnDeceleration = 0;

    if (acceleration < 0) {
        deceleration = acceleration;
        acceleration = 0;
    }
    if (turnAcceleration < 0) {
        turnDeceleration = turnAcceleration;
        turnAcceleration = 0;
    }
//
//    argos::LOG << "Acceleration: " << acceleration << " m/s^2" << std::endl;
//    argos::LOG << "Deceleration: " << deceleration << " m/s^2" << std::endl;
//    argos::LOG << "Turn Acceleration: " << turnAcceleration << " rad/s^2" << std::endl;
//    argos::LOG << "Turn Deceleration: " << turnDeceleration << " rad/s^2" << std::endl;

    float forces[6];
    calculateForcesPastMovement(acceleration, deceleration, turnAcceleration, turnDeceleration, forces, agent);

    //Now we know the distance and the angle, we can estimate the time it will take to travel this path



    float timeToAccelerateTurn = turnAcceleration == 0 ? 0 : time; //In seconds
    float timeToDecelerateTurn = turnDeceleration == 0 ? 0 : time; //In seconds
    float timeToConstantTurn = turnAcceleration + turnDeceleration == 0 && turnAngle != 0 ? time : 0; //In seconds
    float timeToAccelerateDrive = acceleration == 0 ? 0 : time; //In seconds
    float timeToDecelerateDrive = deceleration == 0 ? 0 : time; //In seconds
    float timeToConstantDrive = acceleration + deceleration == 0 && distance != 0 ? time : 0; //In seconds
    assert(timeToAccelerateTurn + timeToDecelerateTurn + timeToConstantTurn <= time);
    assert(timeToAccelerateDrive + timeToDecelerateDrive + timeToConstantDrive <= time);

    float forceDurations[] = {timeToAccelerateDrive, timeToConstantDrive, timeToDecelerateDrive, timeToAccelerateTurn, timeToConstantTurn,
                              timeToDecelerateTurn};

    totalDuration += time;

    //We can also estimate the power usage
    float motorPowerUsage = EstimateMotorPowerUsage(agent, forces, forceDurations); //In Wh

    return {motorPowerUsage, totalDuration};
}



/**
 * Get the maximum achievable speed of the robot, based on max voltage (6v)
 * @return
 */
float MotionSystemBatteryManager::getMaxAchievableSpeed() const {
    //Calculate maximum constant speed
    float force_per_wheel_constant = rolling_force_without_acceleration_N / 2; //In Newtons
    float torque_per_motor_constant = force_per_wheel_constant * robot_wheel_radius_m; //In N.M
    float torque_per_motor_kg_cm_constant = torque_per_motor_constant * 10.1971621f; //In kg.cm
    //Now we know the torque, we can estimate the current draw
    //We can estimate the current draw by looking at the torque-speed curve of the motor
    //Torque and speed are negatively linearly related: 0 torque at max speed, 0 speed at max torque (stall)
    float rpm_at_torque = no_load_rpm - (torque_per_motor_kg_cm_constant / stall_torque_kg_cm) * no_load_rpm; //In RPM @ 6V
    //TODO: we are assuming 6V is the max voltage, correct?

    float max_speed = rpm_at_torque * 2.0f * ARGOS_PI * robot_wheel_radius_m / 60.0f; //In m/s
    //Now we know the max achievable speed.
    return max_speed;
}

/**
 * Get the voltage required for achieving the given speed in m/s
 * @param speed_m_s
 * @return
 */
void MotionSystemBatteryManager::calculateVoltageAtSpeed(float speed_m_s) {
    //Calculate the torque at this speed_m_s

    float rpm_at_speed = speed_m_s * 60.0f / (2.0f * ARGOS_PI * robot_wheel_radius_m); //In RPM @ 6V
    float torque_at_speed = rolling_force_without_acceleration_N * robot_wheel_radius_m * 10.1971621f; //In kg.cm
    float torque_per_motor = torque_at_speed / 2; //In kg.cm

    float speed_at_torque_at_3V = 120.0f * (1-torque_per_motor/0.4); //In RPM @ 3V: no-load rpm * (1 - torque/stall torque);
    float speed_at_torque_at_4_5V = 185.0f * (1-torque_per_motor/0.6); //In RPM @ 4.5V: no-load rpm * (1 - torque/stall torque);
    float speed_at_torque_at_6V = 250.0f * (1-torque_per_motor/0.8); //In RPM @ 6V: no-load rpm * (1 - torque/stall torque);

    //Now we know the speed at the torque, we can calculate the voltage required to achieve the speed
    float voltage_at_speed_using_6v = rpm_at_speed/speed_at_torque_at_6V * 6.0f; //In Volts
    float voltage_at_speed_using_4_5v = rpm_at_speed/speed_at_torque_at_4_5V * 4.5f; //In Volts
    float voltage_at_speed_using_3v = rpm_at_speed/speed_at_torque_at_3V * 3.0f; //In Volts

    //Return the calculated voltage that is closest to the voltage used in calculation, this will be the most accurate
    float diff_6v = std::abs(voltage_at_speed_using_6v - 6.0f);
    float diff_4_5v = std::abs(voltage_at_speed_using_4_5v - 4.5f);
    float diff_3v = std::abs(voltage_at_speed_using_3v - 3.0f);

    //Interpolate values
    if(diff_6v < diff_4_5v && diff_6v < diff_3v) {
        this->voltage_at_operating_speed = voltage_at_speed_using_6v;
        //Interpolate no load current
    } else if(diff_4_5v < diff_6v && diff_4_5v < diff_3v) {
        this->voltage_at_operating_speed = voltage_at_speed_using_4_5v;
    } else {
        this->voltage_at_operating_speed = voltage_at_speed_using_3v;
    }
    this->no_load_rpm_at_voltage = 120.0f + (250.0f - 120.0f) * (this->voltage_at_operating_speed - 3.0f) / 3.0f;
    this->no_load_current_at_voltage = 0.15f + (0.16f - 0.15f) * (this->voltage_at_operating_speed - 3.0f) / 3.0f;
    //Assume polynomial relationship between stall current and voltage (based on data)
    this->stall_current_at_voltage = 1.5f + -0.266667f*this->voltage_at_operating_speed + 0.04444444f*this->voltage_at_operating_speed*this->voltage_at_operating_speed; //In A
    this->stall_torque_at_voltage = 0.4f + (0.8f - 0.4f) * (this->voltage_at_operating_speed - 3.0f) / 3.0f;


}
