//
// Created by hugo on 17-6-24.
//

#ifndef THESIS_ARGOS_AGENT_H
#define THESIS_ARGOS_AGENT_H

#include "feature_config.h"
#include "utils/coordinate.h"
// #include "agent_control/communication/simulation/radio.h"
// #include "agent_control/motion/simulation/DifferentialDrive.h"
#include "utils/Quadtree.h"
#include <string>
// #include <argos3/core/utility/math/vector2.h>
// #include <argos3/core/utility/math/quaternion.h>
// #include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include "vector2.h"
#include <set>
// #include "agent_control/sensing/simulation/distance_sensor/hc_sr04.h"
#ifdef WALL_FOLLOWING_ENABLED
#include "agent_implementation/agent_control/path_planning/WallFollower.h"
#endif
#ifdef SKIP_UNREACHABLE_FRONTIERS
// #include "agent_implementation/agent_control/path_planning/FrontierEvaluator.h"
#include "agent_control/path_planning/FrontierEvaluator.h"
#endif
// #include "agent_implementation/agent_control/path_planning/ForceVectorCalculator.h"
#include "agent_control/path_planning/ForceVectorCalculator.h"
// #include "agent_control/motion/simulation/DifferentialDrive.h"
#include "agent_control/battery/BatteryManager.h"
#ifdef PATH_PLANNING_ENABLED
// #include "agent_implementation/agent_control/path_planning/SimplePathPlanner.h"
#include "agent_control/path_planning/SimplePathPlanner.h"
// #include "agent_implementation/agent_control/path_planning/PathFollower.h"
#include "agent_control/path_planning/PathFollower.h"
#endif
#include "agent_control/communication/TimeSynchronizer.h"
#ifdef RANDOM_WALK_WHEN_NO_FRONTIERS
// #include "agent_implementation/agent_control/path_planning/RandomWalk.h"
#include "agent_control/path_planning/RandomWalk.h"
#endif


#include "../motion/DifferentialDrive.h"
#include "../mqtt/radio.h"
#include "../sensing/HC_SR04.h"

class Agent {
public:
    std::string id{};
    Coordinate position{};
    argos::CRadians heading;
    argos::CRadians targetHeading;
    float speed{};

    //Differential drive
    DifferentialDrive differential_drive;

    //Radio
    Radio wifi;

    //Distance sensors
    static constexpr double num_sensors = 4;
    std::array<HC_SR04, static_cast<int>(num_sensors)> distance_sensors{};

    //Time synchronizer between agents
    TimeSynchronizer timeSynchronizer;

    //Always need this one for performance measure
//#ifdef BATTERY_MANAGEMENT_ENABLED
    //Battery manager
    BatteryManager batteryManager;
//#endif
#ifdef WALL_FOLLOWING_ENABLED
    //Path planning engines
    WallFollower wallFollower;
#endif
#ifdef SKIP_UNREACHABLE_FRONTIERS
    FrontierEvaluator frontierEvaluator;
#endif
#ifdef PATH_PLANNING_ENABLED
    SimplePathPlanner pathPlanner;
    PathFollower pathFollower;
#endif

#ifdef RANDOM_WALK_WHEN_NO_FRONTIERS
    RandomWalk randomWalker;
#endif
    struct Config {
        double ROBOT_RADIUS;
        float ROBOT_WEIGHT;
        float ROBOT_WHEEL_RADIUS;
        float ROBOT_INTER_WHEEL_DISTANCE;

        float MISSION_END_TIME_S;
        float MISSION_END_BATTERY_LEVEL;

        double OBJECT_SAFETY_RADIUS;
        double AGENT_SAFETY_RADIUS;

        double TURN_THRESHOLD_DEGREES;
        float TURNING_SPEED_RATIO;
        double STEPS_360_DEGREES;

        double AGENT_LOCATION_RELEVANT_S;
        double QUADTREE_EXCHANGE_INTERVAL_S;
        double TIME_SYNC_INTERVAL_S;

        double DISTANCE_SENSOR_JITTER_CM;
        double DISTANCE_SENSOR_NOISE_FACTOR;
        double ORIENTATION_NOISE_DEGREES;
        double ORIENTATION_JITTER_DEGREES;
        double POSITION_NOISE_CM;
        double POSITION_JITTER_CM;
        double DISTANCE_SENSOR_PROXIMITY_RANGE;

        double VIRTUAL_WALL_AVOIDANCE_WEIGHT;
        double AGENT_COHESION_WEIGHT;
        double AGENT_AVOIDANCE_WEIGHT;
        double AGENT_ALIGNMENT_WEIGHT;
        double TARGET_WEIGHT;

        double FRONTIER_DISTANCE_WEIGHT;
        double FRONTIER_SIZE_WEIGHT;
        double FRONTIER_REACH_BATTERY_WEIGHT;
        double FRONTIER_REACH_DURATION_WEIGHT;
        double FRONTIER_PHEROMONE_WEIGHT;
        double FRONTIER_PHEROMONE_K;
        double FRONTIER_PHEROMONE_N;
        double FRONTIER_PHEROMONE_M;
        double FRONTIER_PHEROMONE_L;

        double FRONTIER_SEARCH_RADIUS;
//        int MAX_FRONTIER_CELLS;
        double FRONTIER_CELL_RATIO;
        int MAX_FRONTIER_REGIONS;
        double AGENT_COHESION_RADIUS;
        double AGENT_AVOIDANCE_RADIUS;
        double AGENT_ALIGNMENT_RADIUS ;
        double OBJECT_AVOIDANCE_RADIUS;
        double FRONTIER_DIST_UNTIL_REACHED;

        #ifdef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
        float PERIODIC_FEASIBILITY_CHECK_INTERVAL_S;
        float FRONTIER_SWITCH_INTERVAL_S;
        bool FEASIBILITY_CHECK_ONLY_ROUTE;
        #endif

        #if defined(SEPARATE_FRONTIERS) || defined(SKIP_UNREACHABLE_FRONTIERS)
        double FRONTIER_SEPARATION_THRESHOLD;
        #endif

        #ifdef PATH_PLANNING_ENABLED
        double MAX_ROUTE_LENGTH;
        #endif

        double P_FREE ;
        double P_OCCUPIED;
        float ALPHA_RECEIVE;
        float P_FREE_THRESHOLD;
        float P_OCCUPIED_THRESHOLD;
        float P_MAX;
        float P_MIN;
        float P_AT_MAX_SENSOR_RANGE;

        double QUADTREE_RESOLUTION;
        double QUADTREE_EVAPORATION_TIME_S;
        double QUADTREE_EVAPORATED_PHEROMONE_FACTOR;
        double QUADTREE_MERGE_MAX_VISITED_TIME_DIFF;
        double QUADTREE_MERGE_MAX_P_CONFIDENCE_DIFF;

        double BATTERY_CAPACITY;
        double BATTERY_VOLTAGE;
        double MOTOR_STALL_CURRENT;
        double MOTOR_STALL_TORQUE;
        double MOTOR_NO_LOAD_RPM;
        double MOTOR_NO_LOAD_CURRENT;

        double WIFI_SPEED_MBPS;
        double WIFI_RANGE_M;
        double MAX_JITTER_MS;
        double MESSAGE_LOSS_PROBABILITY;

    };
    Config config;


//    double DISTANCE_SENSOR_NOISE_CM = 5.0;
//    double ORIENTATION_NOISE_DEGREES = 5.0;
//    double POSITION_NOISE_CM = 5.0;


    std::map<std::string, std::tuple<Coordinate, Coordinate, double>> agentLocations; //id: (location, frontier, timestamp)
//    double AGENT_LOCATION_RELEVANT_DURATION_S = 10.0;
//    double QUADTREE_EXCHANGE_INTERVAL_S = 5.0;
    std::map<std::string, double> agentQuadtreeSent; //id: sent timestamp
    std::map<std::string, int> agentQuadtreeBytesReceived; //id: bytes received
    std::map<std::string, argos::CVector2> agentVelocities; //id: (direction, speed)

//    double TIME_SYNC_INTERVAL_S = 10.0;
    int last_time_sync_tick = 0;

    //Vector affected by swarm
    argos::CVector2 swarm_vector;
    //Force vector deciding the next position
    argos::CVector2 force_vector;

    Coordinate deploymentLocation;
    double min_distance_to_deployment_location = MAXFLOAT;
    double deployment_location_reach_distance;


    Agent() {}

    explicit Agent(std::string id, double rootbox_size, const std::string& config_file);

    void setPosition(double new_x, double new_y);

    void setPosition(Coordinate position);

    void setHeading(argos::CRadians new_heading);

    Coordinate getPosition() const;

    std::string getId() const;

    void setId(std::string id);

    void setSpeed(double speed);

    double getSpeed() const;

    Radio getWifi() const;

    void setWifi(Radio newWifi);

    void print() const;

    void updateMap();

    void setLastRangeReadings(int index, double new_range);

    void readDistanceSensor();

    void readInfraredSensor();

    void calculateNextPosition();

    void doStep();

    void startMission();

    void sendQuadtreeToCloseAgents();
    void timeSyncWithCloseAgents();
    void broadcastMessage(const std::string &message) const;
    void sendMessage(const std::string &message, const std::string& targetId);

    void checkMessages();

    void parseMessages();


    std::vector<std::string> getMessages();


    std::unique_ptr<quadtree::Quadtree> quadtree;



    double ticks_per_second = 16; //Needs to be double to prevent parsing necessity when doing elapsed ticks/ticks_per_second

    enum class State {
        NO_MISSION,
        EXPLORING,
        RETURNING,
        FINISHED_EXPLORING,
        MAP_RELAYED
    };

    State state = State::NO_MISSION;




//    double DISTANCE_SENSOR_PROXIMITY_RANGE = 2.0;

//    double TURN_THRESHOLD_DEGREES = 8.0;

//    double AGENT_ROBOT_DIAMETER = 0.08;

//    double OBJECT_SAFETY_RADIUS = 0.1;
//    double AGENT_SAFETY_RADIUS = AGENT_ROBOT_DIAMETER + 0.1;

//    double VIRTUAL_WALL_AVOIDANCE_WEIGHT = 1.1;
//    double AGENT_COHESION_WEIGHT = 0;//0.23;
//    double AGENT_AVOIDANCE_WEIGHT = 1.15;
//    double AGENT_ALIGNMENT_WEIGHT = 0;//0.5;//0.5;
//    double TARGET_WEIGHT = 0.3;
//
//    double FRONTIER_DISTANCE_WEIGHT = 0.0;//0.001;
//    double FRONTIER_SIZE_WEIGHT = 1.0;
//    double FRONTIER_REACH_BATTERY_WEIGHT = 8.0;
//    double FRONTIER_REACH_DURATION_WEIGHT = 1.0;
//    double FRONTIER_PHEROMONE_WEIGHT = 500.0;

//    double FRONTIER_SEARCH_RADIUS = 4.0;

//    double AGENT_COHESION_RADIUS = 1.5;
//    double AGENT_AVOIDANCE_RADIUS = 0.68;
//    double AGENT_ALIGNMENT_RADIUS = 1.5;
//    double OBJECT_AVOIDANCE_RADIUS = AGENT_SAFETY_RADIUS + OBJECT_SAFETY_RADIUS + 0.2;

#ifdef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
//    double FRONTIER_DIST_UNTIL_REACHED = OBJECT_AVOIDANCE_RADIUS;
//    float PERIODIC_FEASIBILITY_CHECK_INTERVAL_S = 5.0;
    int last_feasibility_check_tick = 0;
    int last_frontier_switch_tick = 0;
#endif

#ifdef SEPARATE_FRONTIERS
//    double FRONTIER_CLOSE_DISTANCE = 1.0;
#endif


    Coordinate left_right_borders = {-10, 10};
    Coordinate upper_lower_borders = {10, -10};

//    float TURNING_SPEED_RATIO = 0.1;

//    double ANGLE_INTERVAL_STEPS = 360;



//    double P_POSITION = 0.9; // 90% probability for position to be correct
//    double P_FREE = 0.6; // 60% probability for free to be correct
//    double P_OCCUPIED = 0.4; // 40% probability for occupied to be correct
//    float ALPHA_RECEIVE = 0.1; // Factor with which a received value's probability is pulled towards 0.5
//
//    float P_FREE_THRESHOLD = 0.6; //P > 0.6 means it is definitely free
//    float P_OCCUPIED_THRESHOLD = 0.4; //P < 0.3 means it is definitely occupied


    Coordinate currentBestFrontier = {MAXFLOAT, MAXFLOAT};
    Coordinate previousBestFrontier = {0, 0};
    Coordinate subTarget = {MAXFLOAT, MAXFLOAT};


    uint32_t elapsed_ticks = 0;

    std::vector<std::pair<quadtree::Box, double>> current_frontiers;
    std::vector<std::vector<std::pair<quadtree::Box, double>>> current_frontier_regions;
    std::set<argos::CDegrees> freeAnglesVisualization;
    argos::CVector2 perpendicularVectorVisualization;
    std::vector<Coordinate> lineVisualization;
    std::vector<std::pair<Coordinate, Coordinate>> route_to_best_frontier;
    std::vector<std::pair<quadtree::Box, double>> bestFrontierRegionBoxes = {};


    double sensor_reading_distance_probability;

private:
    void loadConfig(const std::string& config_file, double rootbox_size);


    void checkForObstacles();

//    void checkIfAgentFitsBetweenObstacles(quadtree::Box obstacleBox) const;

    bool isObstacleBetween(Coordinate coordinate1, Coordinate coordinate2) const;


    std::vector<std::string> messages;

    std::string GetId() const;

    void checkMissionEnd();


    quadtree::Box addObjectLocation(Coordinate objectCoordinate) const;
    void addFreeAreaBetween(Coordinate agentCoordinate, Coordinate coordinate2, quadtree::Box objectBox);
    void addFreeAreaBetween(Coordinate agentCoordinate, Coordinate coordinate2);
//    void addFreeAreaBetweenAndOccupiedAfter(Coordinate coordinate1, Coordinate coordinate2, quadtree::Box objectBox);
    void addOccupiedAreaBetween(Coordinate agentCoordinate, Coordinate coordinate2) const;

    bool frontierPheromoneEvaporated();

    void syncMissionTime(double received_time);

#ifdef RANDOM_WALK_WHEN_NO_FRONTIERS
    void randomWalk(argos::CVector2 & targetVector);
#endif



    };


#endif //THESIS_ARGOS_AGENT_H
