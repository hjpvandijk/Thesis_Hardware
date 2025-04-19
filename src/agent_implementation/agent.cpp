//
// Created by hugo on 17-6-24.
//

#include <iostream>
// #include <argos3/core/utility/logging/argos_log.h>
#include <set>
#include "agent.h"
#include <random>
#include <utility>
#include "utils/CustomComparator.h"
#include "utils/Algorithms.h"
#include <yaml-cpp/yaml.h>
#include "config_yaml_data.h"


Agent::Agent(std::string id, double rootbox_size, const std::string& config_file) {
    auto box = quadtree::Box(-rootbox_size/2, rootbox_size/2, rootbox_size);

    loadConfig(config_file, box.size);
    this->id = std::move(id);
    this->position = {0.0, 0.0};
    this->heading = argos::CRadians(0);
    this->targetHeading = argos::CRadians(0);
    this->speed = 1;
    this->swarm_vector = argos::CVector2(0, 0);
    this->force_vector = argos::CVector2(0, 1);
    this->messages = std::vector<std::string>(0);
    this->quadtree = std::make_unique<quadtree::Quadtree>(box, this->config.P_FREE_THRESHOLD, this->config.P_OCCUPIED_THRESHOLD,
                                                          this->config.P_MAX, this->config.P_MIN,
                                                          this->config.ALPHA_RECEIVE,
                                                          this->config.QUADTREE_RESOLUTION,
                                                          this->config.QUADTREE_EVAPORATION_TIME_S,
                                                          this->config.QUADTREE_EVAPORATED_PHEROMONE_FACTOR,
                                                          this->config.QUADTREE_MERGE_MAX_VISITED_TIME_DIFF,
                                                          this->config.QUADTREE_MERGE_MAX_P_CONFIDENCE_DIFF);
    double smallestBoxSize = box.size;
    while (smallestBoxSize > this->config.QUADTREE_RESOLUTION) {
        smallestBoxSize /= 2;
    }

    this->quadtree->setResolution(smallestBoxSize);
    #ifdef WALL_FOLLOWING_ENABLED
    this->wallFollower = WallFollower();
    #endif
    this->timeSynchronizer = TimeSynchronizer();

//#ifdef BATTERY_MANAGEMENT_ENABLED
    //https://e-puck.gctronic.com/index.php?option=com_content&view=article&id=7&Itemid=9
    //Motor stall values based on the tt dc gearbox motor (https://www.sgbotic.com/index.php?dispatch=products.view&product_id=2674)
    this->batteryManager = BatteryManager(this->config.ROBOT_WEIGHT, this->config.ROBOT_WHEEL_RADIUS,
                                          this->config.ROBOT_INTER_WHEEL_DISTANCE, this->config.MOTOR_STALL_TORQUE,
                                          this->config.MOTOR_NO_LOAD_RPM, this->config.MOTOR_STALL_CURRENT,
                                          this->config.MOTOR_NO_LOAD_CURRENT, this->config.BATTERY_VOLTAGE,
                                          this->config.BATTERY_CAPACITY);
    //Set the speed to the maximum achievable speed, based on the the motor specs. TODO: Put that info in differential drive instead
    auto max_achievable_speed = this->batteryManager.motionSystemBatteryManager.getMaxAchievableSpeed();
    this->differential_drive = DifferentialDrive(std::min(max_achievable_speed, this->speed),
                                                 std::min(max_achievable_speed,
                                                          this->speed * this->config.TURNING_SPEED_RATIO));
    this->speed = this->differential_drive.max_speed_straight;
    //Set the voltage to the voltage required for the current speed, and corresponding values, to use in calculations.
    this->batteryManager.motionSystemBatteryManager.calculateVoltageAtSpeed(this->speed);
//#else
//    this->differential_drive = DifferentialDrive(this->speed, this->speed*this->config.TURNING_SPEED_RATIO);
//#endif

#ifdef PATH_PLANNING_ENABLED
    this->pathPlanner = SimplePathPlanner();
    this->pathFollower = PathFollower();
#endif
#ifdef SKIP_UNREACHABLE_FRONTIERS
    this->frontierEvaluator = FrontierEvaluator(5*this->ticks_per_second); //5 seconds
#endif
#ifdef RANDOM_WALK_WHEN_NO_FRONTIERS
    this->randomWalker = RandomWalk(5*this->ticks_per_second); //5 seconds
#endif
    this->sensor_reading_distance_probability = (1-this->config.P_AT_MAX_SENSOR_RANGE) / this->config.DISTANCE_SENSOR_PROXIMITY_RANGE;

    this->deployment_location_reach_distance = this->config.FRONTIER_DIST_UNTIL_REACHED;
}


void Agent::setPosition(double new_x, double new_y) {
    this->position = {new_x, new_y};
}


void Agent::setPosition(Coordinate new_position) {
    this->position = new_position;
}

void Agent::setHeading(argos::CRadians new_heading) {
    this->heading = Coordinate::ArgosHeadingToOwn(new_heading).SignedNormalize();
}


Coordinate Agent::getPosition() const {
    return this->position;
}

std::string Agent::getId() const {
    return this->id;
}

std::string Agent::GetId() const {
    return this->id;
}

void Agent::setId(std::string new_id) {
    this->id = std::move(new_id);
}

void Agent::setSpeed(double new_speed) {
    this->speed = new_speed;
}

double Agent::getSpeed() const {
    return this->speed;
}


void Agent::print() const {
    std::cout << "Agent " << this->id << " is at position (" << this->position.x << ", " << this->position.y
              << ")" << std::endl;
}

void Agent::updateMap() {
    //Update map with new information
}

// void Agent::setLastRangeReadings(int index, double new_range) {
//     this->distance_sensors.at(index)->setDistance(new_range);
// }

void Agent::setDistanceSensorHandler(DistanceSensorHandler* distanceSensorHandler){
    this->distanceSensorHandler = distanceSensorHandler;
}


void Agent::readDistanceSensor() {

}

void Agent::readInfraredSensor() {

}

/**
 * Add free (unoccupied) coordinates between two coordinates and slightly occupied coordinates after the object
 * @param coordinate1
 * @param coordinate2
 */
//void Agent::addFreeAreaBetweenAndOccupiedAfter(Coordinate coordinate1, Coordinate coordinate2, quadtree::Box objectBox) {
//    if (sqrt(pow(coordinate1.x - coordinate2.x, 2) + pow(coordinate1.y - coordinate2.y, 2)) <
//        this->quadtree->getResolution())
//        return; //If the distance between the coordinates is smaller than the smallest box size, don't add anything
////    if (objectBox.size > 0) return;
//
//    double x = coordinate1.x;
//    double y = coordinate1.y;
//    double dx = coordinate2.x - coordinate1.x;
//    double dy = coordinate2.y - coordinate1.y;
//    double distance = sqrt(dx * dx + dy * dy);
//    double stepSize = this->quadtree->getResolution();
//    int nSteps = std::ceil(distance / stepSize);
//    double stepX = dx / nSteps;
//    double stepY = dy / nSteps;
//
//    for (int s = 0; s < nSteps; s++) {
//        if (!objectBox.contains(Coordinate{x, y})) { //Don't add a coordinate in the objectBox as free
//            double p = (this->config.P_FREE - 0.5) * (1 - double(s) / double(nSteps)) +
//                       0.5; //Increasingly more uncertain the further away from the agent
//            //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
//            this->quadtree->add(Coordinate{x + 0.0000000001, y + 0.0000000001}, p, elapsed_ticks / ticks_per_second,
//                                elapsed_ticks / ticks_per_second);
//        } else {
//            break; //If the coordinate is in the objectBox, stop adding free coordinates, because it should be the end of the ray
//        }
//        x += stepX;
//        y += stepY;
//    }
//
////    x += stepX;
////    y += stepY;
////
////    double occ_probability = this->P_FREE / Psensor;
////    for(int s = 1; s <= 2; s++){ //Add slight occupied confidence to the two steps after the object
////        if (!objectBox.contains(Coordinate{x, y})) { //Don't add a coordinate in the objectBox as more occupied
////            double p = (0.5-occ_probability) * (double(s)/double(10)) + occ_probability; //Increasingly more uncertain the further away from the agent
////            this->quadtree->add(Coordinate{x, y}, p, elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
////        }
////        x += stepX;
////        y += stepY;
////
////    }
//}

/**
 * Add free (unoccupied) coordinates between two coordinates
 * @param coordinate1
 * @param coordinate2
 */
void Agent::addFreeAreaBetween(Coordinate coordinate1, Coordinate coordinate2, quadtree::Box objectBox) {
    double dist = sqrt(pow(coordinate1.x - coordinate2.x, 2) + pow(coordinate1.y - coordinate2.y, 2));
    if (dist < this->quadtree->getResolution())
        return; //If the distance between the coordinates is smaller than the smallest box size, don't add anything
//    double x = coordinate1.x;
//    double y = coordinate1.y;
//    double dx = coordinate2.x - coordinate1.x;
//    double dy = coordinate2.y - coordinate1.y;
//    double distance = sqrt(dx * dx + dy * dy);
//    double stepSize = this->quadtree->getResolution();
//    int nSteps = std::ceil(distance / stepSize);
//    double stepX = dx / nSteps;
//    double stepY = dy / nSteps;
//
//    for (int s = 0; s < nSteps; s++) {
//        if (objectBox.size > 0) {
//            if (!objectBox.contains(Coordinate{x, y})) { //Don't add a coordinate in the objectBox as free
//                double p = (this->config.P_FREE - 0.5) * (1 - double(s) / double(nSteps)) +
//                           0.5; //Increasingly more uncertain the further away from the agent
//                //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
//                this->quadtree->add(Coordinate{x + 0.0000000001, y + 0.0000000001}, p * Psensor,
//                                    elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
//            } else {
//                break; //If the coordinate is in the objectBox, stop adding free coordinates, because it should be the end of the ray
//            }
//        }
//        x += stepX;
//        y += stepY;
//    }

    std::vector<Coordinate> linePoints = Algorithms::Amanatides_Woo_Voxel_Traversal(this, coordinate1,
                                                                                    coordinate2);
    double step_size_avg = dist / linePoints.size();
    for (int i=0; i<linePoints.size(); i++){
        auto point = linePoints[i];
        if (!objectBox.contains(Coordinate{point.x, point.y})) { //Don't add a coordinate in the objectBox as free
            double p_distance_reading = 1 - double(i)*step_size_avg * sensor_reading_distance_probability;
            double p = (this->config.P_FREE - 0.5) * p_distance_reading + 0.5; //Increasingly more uncertain the further away from the agent, as error can increase with position and orientation estimation inaccuracies.
            //            argos::LOG << "P = " << p << "with " << i << "/" << linePoints.size() << std::endl;
           //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
            auto coord = Coordinate{point.x, point.y};
            while(Algorithms::is_multiple(coord.x, this->quadtree->getResolution()) || Algorithms::is_multiple(coord.y, this->quadtree->getResolution())){
                coord = Coordinate{coord.x + 0.000001, coord.y + 0.000001};
            }
            this->quadtree->add(coord, p,
                                elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
        } else {
            break; //If the coordinate is in the objectBox, stop adding free coordinates, because it should be the end of the ray
        }
    }
}

/**
 * Add free (unoccupied) coordinates between two coordinates
 * @param coordinate1
 * @param coordinate2
 */
void Agent::addFreeAreaBetween(Coordinate coordinate1, Coordinate coordinate2) {
//    double x = coordinate1.x;
//    double y = coordinate1.y;
//    double dx = coordinate2.x - coordinate1.x;
//    double dy = coordinate2.y - coordinate1.y;
//    double distance = sqrt(dx * dx + dy * dy);
//    double stepSize = this->quadtree->getResolution();
//    int nSteps = std::ceil(distance / stepSize);
//    double stepX = dx / nSteps;
//    double stepY = dy / nSteps;
//
//    for (int s = 0; s < nSteps; s++) {
//        //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
//        this->quadtree->add(Coordinate{x + 0.0000000001, y + 0.0000000001}, this->config.P_FREE * Psensor,
//                            elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
//        x += stepX;
//        y += stepY;
//    }
    double dist = sqrt(pow(coordinate1.x - coordinate2.x, 2) + pow(coordinate1.y - coordinate2.y, 2));
    if (dist < this->quadtree->getResolution())
        return;
    std::vector<Coordinate> linePoints = Algorithms::Amanatides_Woo_Voxel_Traversal(this, coordinate1,
                                                                                    coordinate2);
    double step_size_avg = dist / linePoints.size();
    for (int i=0; i<linePoints.size(); i++){
        auto point = linePoints[i];
        double p_distance_reading = 1 - double(i)*step_size_avg * sensor_reading_distance_probability;
        double p = (this->config.P_FREE - 0.5) * p_distance_reading +0.5; //Increasingly more uncertain the further away from the agent, as error can increase with position and orientation estimation inaccuracies.
        //        argos::LOG << "P = " << p << "with " << i << "/" << linePoints.size() << std::endl;
        //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
        auto coord = Coordinate{point.x, point.y};
        while(Algorithms::is_multiple(coord.x, this->quadtree->getResolution()) || Algorithms::is_multiple(coord.y, this->quadtree->getResolution())){
            coord = Coordinate{coord.x + 0.000001, coord.y + 0.000001};
        }
        this->quadtree->add(coord, p,
                            elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
    }
}

/**
 * Add occupied coordinates between two coordinates
 * @param coordinate1
 * @param coordinate2
 */
//void Agent::addOccupiedAreaBetween(Coordinate coordinate1, Coordinate coordinate2) const {
//    double x = coordinate1.x;
//    double y = coordinate1.y;
//    double dx = coordinate2.x - coordinate1.x;
//    double dy = coordinate2.y - coordinate1.y;
//    double distance = sqrt(dx * dx + dy * dy);
//    double stepSize = this->quadtree->getResolution();
//    int nSteps = std::ceil(distance / stepSize);
//    double stepX = dx / nSteps;
//    double stepY = dy / nSteps;
//
//    for (int s = 0; s < nSteps - 1; s++) {
//        //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
//        this->quadtree->add(Coordinate{x + 0.0000000001, y + 0.0000000001}, this->config.P_OCCUPIED,
//                            elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
//        x += stepX;
//        y += stepY;
//    }
//}

//bool Agent::isObstacleBetween(Coordinate coordinate1, Coordinate coordinate2) const {
//    double x = coordinate1.x;
//    double y = coordinate1.y;
//    double dx = coordinate2.x - coordinate1.x -
//                0.02; // Subtract a small margin to detecting boxes the coordinates belong to
//    double dy = coordinate2.y - coordinate1.y - 0.02;
//    double distance = sqrt(dx * dx + dy * dy);
//    double stepSize = this->quadtree->getResolution();
//    int nSteps = std::ceil(distance / stepSize);
//    double stepX = dx / nSteps;
//    double stepY = dy / nSteps;
//
//    x += stepX * 0.1; // Add a small margin to detecting boxes the coordinates belong to
//    y += stepY * 0.1; // Add a small margin to detecting boxes the coordinates belong to
//
//
//    for (int s = 1; s < nSteps; s++) {
//        if (this->quadtree->getOccupancyFromCoordinate(Coordinate{x, y}) == quadtree::Occupancy::OCCUPIED) {
//            return true;
//        }
//        x += stepX;
//        y += stepY;
//    }
//    return false;
//}


/**
 * Add occupied object location to the quadtree
 * @param objectCoordinate
 */
quadtree::Box Agent::addObjectLocation(Coordinate objectCoordinate) const {
    //Add small margin to the x and y in case we are exactly on the corner of a box, due to the perfection of a simulated map.
    auto dist_to_object = sqrt(pow(this->position.x - objectCoordinate.x, 2) + pow(this->position.y - objectCoordinate.y, 2));

    double p_distance_reading = 1 - dist_to_object * sensor_reading_distance_probability;
    double p = 0.5-(0.5-this->config.P_OCCUPIED) * p_distance_reading ; //Increasingly more certain the closer to the agent, as the sensor reading is more accurate
    auto coord = Coordinate{objectCoordinate.x, objectCoordinate.y};
    while(Algorithms::is_multiple(coord.x, this->quadtree->getResolution()) || Algorithms::is_multiple(coord.y, this->quadtree->getResolution())){
        coord = Coordinate{coord.x + 0.000001, coord.y + 0.000001};
    }
    quadtree::Box objectBox = this->quadtree->add( coord, p, elapsed_ticks / ticks_per_second, elapsed_ticks / ticks_per_second);
#ifdef CLOSE_SMALL_AREAS
    if (objectBox.getSize() != 0) // If the box is not the zero (not added)
        checkIfAgentFitsBetweenObstacles(objectBox);
#endif
    return objectBox;
}

/**
 * Check for obstacles in front of the agent
 * If there is an obstacle within a certain range, add the free area between the agent and the obstacle to the quadtree
 * If there is no obstacle within range, add the free area between the agent and the end of the range to the quadtree
 */
void Agent::checkForObstacles() {
    // printf("Checking for obstacles\n");
    bool addedObjectAtAgentLocation = false;
    for (int sensor_index = 0; sensor_index < Agent::num_sensors; sensor_index++) {
        argos::CRadians sensor_rotation = this->heading - sensor_index * argos::CRadians::PI_OVER_TWO;
        auto sensorDistance = this->distanceSensorHandler->getDistance(sensor_index);
        printf("Sensor %d: %f ", sensor_index, sensorDistance);
        if (sensorDistance < this->config.DISTANCE_SENSOR_PROXIMITY_RANGE) {
            double opposite = argos::Sin(sensor_rotation) * sensorDistance;
            double adjacent = argos::Cos(sensor_rotation) * sensorDistance;

            Coordinate object = {this->position.x + adjacent, this->position.y + opposite};
            //If the detected object is actually another agent, add it as a free area
            //So check if the object coordinate is close to another agent
            bool close_to_other_agent = false;
            for (const auto &agentLocation: this->agentLocations) {
                if ((std::get<2>(agentLocation.second) - this->elapsed_ticks) / this->ticks_per_second >
                    this->config.AGENT_LOCATION_RELEVANT_S)
                    continue;
                argos::CVector2 objectToAgent =
                        argos::CVector2(std::get<0>(agentLocation.second).x, std::get<0>(agentLocation.second).y)
                        - argos::CVector2(object.x, object.y);

                //If detected object and another agent are not close, add the object as an obstacle
                if (objectToAgent.Length() <= this->quadtree->getResolution()) {
                    close_to_other_agent = true;
                }
            }
            //Only add the object as an obstacle if it is not close to another agent
            if (!close_to_other_agent) {
                if (sqrt(pow(this->position.x - object.x, 2) + pow(this->position.y - object.y, 2)) <
                    this->quadtree->getResolution()) {
                    addedObjectAtAgentLocation = true;
                }
                quadtree::Box objectBox = addObjectLocation(object);
                if (objectBox.size != 0) {
                    if (!addedObjectAtAgentLocation)
                        addFreeAreaBetween(this->position, object, objectBox);
                } else {
                    if (!addedObjectAtAgentLocation) addFreeAreaBetween(this->position, object);
                }
            } else {
                if (!addedObjectAtAgentLocation) addFreeAreaBetween(this->position, object);
            }


        } else {
            double opposite = argos::Sin(sensor_rotation) * this->config.DISTANCE_SENSOR_PROXIMITY_RANGE;
            double adjacent = argos::Cos(sensor_rotation) * this->config.DISTANCE_SENSOR_PROXIMITY_RANGE;


            Coordinate end_of_ray = {this->position.x + adjacent, this->position.y + opposite};
            if (!addedObjectAtAgentLocation) addFreeAreaBetween(this->position, end_of_ray);
        }
    }
    printf("\n");
}

/**
 * Check the area around the object to see if the agent fits between the object and other obstacles
 * If the agent fits, do nothing
 * If the agent does not fit, set that area as an obstacle.
 * @param objectCoordinate
 */
//void Agent::checkIfAgentFitsBetweenObstacles(quadtree::Box objectBox) const {
//    Coordinate objectCoordinate = objectBox.getCenter();
//    std::vector<quadtree::Box> occupiedBoxes = this->quadtree->queryOccupiedBoxes(objectCoordinate,
//                                                                                  3.0 *
//                                                                                  this->config.OBJECT_AVOIDANCE_RADIUS,
//                                                                                  this->elapsed_ticks /
//                                                                                  this->ticks_per_second);
//    //For each box, that is not the checked object, check if the agent fits between the object and the box
//    for (auto box: occupiedBoxes) {
//        if (box.contains(objectCoordinate)) {
//            continue;
//        }
//
//        Coordinate objectCorner{};
//        std::vector<Coordinate> objectEdges = {};
//        Coordinate boxCorner{};
//        std::vector<Coordinate> boxEdges = {};
//
//        //Check which direction the object is compared to the occupied box, and check the distance between the correct corners
//        auto center = box.getCenter();
//
//        // West
//        if (objectCoordinate.x < center.x) {
//            //Just West
//            if (objectCoordinate.y == center.y) {
//                objectCorner = Coordinate{objectBox.getBottom(), objectBox.getRight()};
//                objectEdges.push_back(Coordinate{objectBox.getRight(), objectBox.getCenter().y});
//
//                boxCorner = Coordinate{box.getBottom(), box.left};
//                boxEdges.push_back(Coordinate{box.left, box.getCenter().y});
//            }
//                // North West
//            else if (objectCoordinate.y > center.y) {
//                objectCorner = Coordinate{objectBox.getBottom(), objectBox.getRight()};
//                objectEdges.push_back(Coordinate{objectBox.getRight(), objectBox.getCenter().y});
//                objectEdges.push_back(Coordinate{objectBox.getCenter().x, objectBox.getBottom()});
//
//                boxCorner = Coordinate{box.top, box.left};
//                boxEdges.push_back(Coordinate{box.left, box.getCenter().y});
//                boxEdges.push_back(Coordinate{box.getCenter().x, box.top});
//            }
//                // South West
//            else if (objectCoordinate.y < center.y) {
//                objectCorner = Coordinate{objectBox.top, objectBox.getRight()};
//                objectEdges.push_back(Coordinate{objectBox.getRight(), objectBox.getCenter().y});
//                objectEdges.push_back(Coordinate{objectBox.getCenter().x, objectBox.top});
//
//                boxCorner = Coordinate{box.getBottom(), box.left};
//                boxEdges.push_back(Coordinate{box.left, box.getCenter().y});
//                boxEdges.push_back(Coordinate{box.getCenter().x, box.getBottom()});
//            }
//                // Not in any direction
//            else {
//                continue;
//            }
//        }
//            // East
//        else if (objectCoordinate.x > center.x) {
//            //Just East
//            if (objectCoordinate.y == center.y) {
//                objectCorner = Coordinate{objectBox.getBottom(), objectBox.left};
//                objectEdges.push_back(Coordinate{objectBox.left, objectBox.getCenter().y});
//
//                boxCorner = Coordinate{box.getBottom(), box.getRight()};
//                boxEdges.push_back(Coordinate{box.getRight(), box.getCenter().y});
//            }
//                // North East
//            else if (objectCoordinate.y > center.y) {
//                objectCorner = Coordinate{objectBox.getBottom(), objectBox.left};
//                objectEdges.push_back(Coordinate{objectBox.left, objectBox.getCenter().y});
//                objectEdges.push_back(Coordinate{objectBox.getCenter().x, objectBox.getBottom()});
//
//                boxCorner = Coordinate{box.top, box.getRight()};
//                boxEdges.push_back(Coordinate{box.getRight(), box.getCenter().y});
//                boxEdges.push_back(Coordinate{box.getCenter().x, box.top});
//            }
//                // South East
//            else if (objectCoordinate.y < center.y) {
//                objectCorner = Coordinate{objectBox.top, objectBox.left};
//                objectEdges.push_back(Coordinate{objectBox.left, objectBox.getCenter().y});
//                objectEdges.push_back(Coordinate{objectBox.getCenter().x, objectBox.top});
//
//                boxCorner = Coordinate{box.getBottom(), box.getRight()};
//                boxEdges.push_back(Coordinate{box.getRight(), box.getCenter().y});
//                boxEdges.push_back(Coordinate{box.getCenter().x, box.getBottom()});
//            }
//                // Not in any direction
//            else {
//                continue;
//            }
//        } else if (objectCoordinate.x == center.x) {
//            //Just North
//            if (objectCoordinate.y > center.y) {
//                objectCorner = Coordinate{objectBox.getBottom(), objectBox.getRight()};
//                objectEdges.push_back(Coordinate{objectBox.getCenter().x, objectBox.getBottom()});
//
//                boxCorner = Coordinate{box.top, box.getRight()};
//                boxEdges.push_back(Coordinate{box.getCenter().x, box.top});
//            }
//                //Just South
//            else if (objectCoordinate.y < center.y) {
//                objectCorner = Coordinate{objectBox.top, objectBox.getRight()};
//                objectEdges.push_back(Coordinate{objectBox.getCenter().x, objectBox.top});
//
//                boxCorner = Coordinate{box.getBottom(), box.getRight()};
//                boxEdges.push_back(Coordinate{box.getCenter().x, box.getBottom()});
//            }
//                // Not in any direction
//            else {
//                continue;
//            }
//        }
//            // Not in any direction
//        else {
//            continue;
//        }
//
//        double distance = sqrt(pow(objectCorner.x - boxCorner.x, 2) + pow(objectCorner.y - boxCorner.y, 2));
//        if (distance < 0.01) { // If they are adjacent (with a small margin)
//            continue;
//        } else if (distance <
//                   this->config.OBJECT_AVOIDANCE_RADIUS) { // If the agent does not fit between the object and the box
//            //Add the area between the object and the box as occupied if there is no occupied area between already
//            bool areaFree = true;
//            for (int edge_count = 0; edge_count < objectEdges.size(); edge_count++) {
//                if (isObstacleBetween(objectEdges[edge_count], boxEdges[edge_count])) {
//                    areaFree = false;
//                }
//            }
//            if (areaFree) {
//                addOccupiedAreaBetween(objectCoordinate, box.getCenter());
//            }
//
//        }
//    }
//}


//bool Agent::frontierPheromoneEvaporated() {
//    quadtree->queryFrontierBoxes(this->currentBestFrontier, quadtree->getResolution() / 2.0,
//                                 this->elapsed_ticks / this->ticks_per_second,
//                                 this->config.MAX_FRONTIER_CELLS); //Update pheromone of frontier cell
//    if (quadtree->isCoordinateUnknownOrAmbiguous(this->currentBestFrontier)) return true;
//    return false;
//}

void Agent::calculateNextPosition() {
    //Inspired by boids algorithm:
    //Vector determining heading
    //Vector is composed of:
    //1. Attraction to unexplored frontier
    //2. Repulsion from other agents (done)
    //3. Attraction to found target
    //4. Repulsion from objects/walls (done)


    argos::CVector2 virtualWallAvoidanceVector = ForceVectorCalculator::getVirtualWallAvoidanceVector(this);
    argos::CVector2 agentCohesionVector = ForceVectorCalculator::calculateAgentCohesionVector(this);
    argos::CVector2 agentAvoidanceVector = ForceVectorCalculator::calculateAgentAvoidanceVector(this);
    argos::CVector2 agentAlignmentVector = ForceVectorCalculator::calculateAgentAlignmentVector(this);

    argos::CVector2 targetVector = argos::CVector2(this->currentBestFrontier.x - this->position.x,
                                                   this->currentBestFrontier.y - this->position.y);

    #ifdef RANDOM_WALK_WHEN_NO_FRONTIERS
    if (randomWalker.randomWalking)
        targetVector = argos::CVector2(this->subTarget.x - this->position.x,
                                       this->subTarget.y - this->position.y);
    #endif

#ifdef PATH_PLANNING_ENABLED
    bool periodic_check_required = (this->elapsed_ticks - this->last_feasibility_check_tick) >
                                   this->ticks_per_second * this->config.PERIODIC_FEASIBILITY_CHECK_INTERVAL_S;
#endif

    if (this->state == State::EXPLORING) {

#ifdef SKIP_UNREACHABLE_FRONTIERS
        frontierEvaluator.resetFrontierAvoidance(this, targetVector);
#endif

#ifdef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
        //If the agent is close to the frontier and is heading towards it, or if it is within object avoidance radius to the frontier.
        //So we don't 'reach' frontiers through walls.
        bool allow_frontier_switch = false;
        if (!(this->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT})) {
            argos::CVector2 agentFrontierVector = argos::CVector2(this->currentBestFrontier.x - this->position.x,
                                                           this->currentBestFrontier.y - this->position.y);
            bool frontier_switch_period_elapsed = (this->elapsed_ticks - this->last_frontier_switch_tick) >
                                                   this->ticks_per_second * this->config.FRONTIER_SWITCH_INTERVAL_S;
            //If the agent is close to the frontier and is somewhat heading towards it AMD switch interval has passed, or if it is within object avoidance radius to the frontier.
            //We have the switch interval to avoid reselecting checking for niew frontiers too quickly, and maybe landing on the same frontier again. (computationally expensive)
            //When we are within object avoidance radius, we will probably select a new frontier soon, so interval not needed.
            allow_frontier_switch = frontier_switch_period_elapsed && ((agentFrontierVector.Length() <= this->config.FRONTIER_DIST_UNTIL_REACHED &&
              NormalizedDifference(this->targetHeading, agentFrontierVector.Angle()).GetValue() < //Use target vector here as we need to check if our intended heading is close to the frontier. Meaning we have determined we are free to move on that direction.
              this->config.TURN_THRESHOLD_DEGREES * 2) || agentFrontierVector.Length() <= this->config.OBJECT_AVOIDANCE_RADIUS);
        }

        //If the current best frontier is not set
        if (
            #ifdef RANDOM_WALK_WHEN_NO_FRONTIERS
            //If we have random walked far enough to try to find a new frontier
            randomWalker.randomWalking && randomWalker.randomWalkedFarEnough(this) ||
            #else
            this->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT} ||
            #endif
            #ifdef SKIP_UNREACHABLE_FRONTIERS
            //Or if we are avoiding the frontier
            frontierEvaluator.avoidingAgentTarget(this) ||
#endif
            //Or are allowed to switch frontiers based on the distance to the current frontier and the time since the last switch
            allow_frontier_switch
#ifdef PATH_PLANNING_ENABLED
            //Or if it is time for a periodic check, and we are not only checking the route
            || (periodic_check_required &&
                !this->config.FEASIBILITY_CHECK_ONLY_ROUTE)
            #endif

                ) {
#ifdef WALL_FOLLOWING_ENABLED
            //If we are not currently wall following
            if (wallFollower.wallFollowingDirection == 0) {
                //Find new frontier
                targetVector = ForceVectorCalculator::calculateUnexploredFrontierVector(this);
                this->last_feasibility_check_tick = this->elapsed_ticks;
                this->last_frontier_switch_tick = this->elapsed_ticks;
            }
#else
            //Find new frontier
            targetVector = ForceVectorCalculator::calculateUnexploredFrontierVector(this);
            this->last_feasibility_check_tick = this->elapsed_ticks;
            this->last_frontier_switch_tick = this->elapsed_ticks;
#endif
        }
#ifdef PATH_PLANNING_ENABLED
        else if (periodic_check_required && this->config.FEASIBILITY_CHECK_ONLY_ROUTE) {
#ifdef RANDOM_WALK_WHEN_NO_FRONTIERS
            //If we are random walking, we don't need to check the route
            if (!this->randomWalker.randomWalking) {
#endif
            this->route_to_best_frontier.clear();
            this->pathPlanner.getRoute(this, this->position, this->currentBestFrontier, this->route_to_best_frontier);
            if (this->route_to_best_frontier.empty()) { //If there is no route to the frontier, find a new frontier
                targetVector = ForceVectorCalculator::calculateUnexploredFrontierVector(this);
            }
#ifdef RANDOM_WALK_WHEN_NO_FRONTIERS
            }
#endif
            this->last_feasibility_check_tick = this->elapsed_ticks;
            this->last_frontier_switch_tick = this->elapsed_ticks;

        }
#endif
    } else if (this->state == State::RETURNING) {
        targetVector = argos::CVector2(this->deploymentLocation.x - this->position.x,
                                       this->deploymentLocation.y - this->position.y);

        //If we are close to the deployment location, we have returned, we can stop
        if (targetVector.Length() <= this->deployment_location_reach_distance) {
            this->state = State::FINISHED_EXPLORING;
        } else {
            //If we are getting closer to the deployment location, we can decrease the distance to reach it, 1cm at a time
            if (targetVector.Length() < this->min_distance_to_deployment_location) {
                this->deployment_location_reach_distance = std::max(this->deployment_location_reach_distance - 0.01, 0.1);
                this->min_distance_to_deployment_location = targetVector.Length();
            } //If we are not getting closer, we can increase the distance to reach it, 0.2cm at a time
            else {
                this->deployment_location_reach_distance += 0.002;
            }

            //Set our current best 'frontier' to the deployment location
            if (!(this->currentBestFrontier == this->deploymentLocation)) {
                this->currentBestFrontier = this->deploymentLocation;
                this->last_feasibility_check_tick = this->elapsed_ticks;
                //We need to find the route, so we set the periodic check required.
#ifdef PATH_PLANNING_ENABLED
                periodic_check_required = true;
#endif
            }
#ifdef PATH_PLANNING_ENABLED
            if (periodic_check_required) {
                this->route_to_best_frontier.clear();
                this->pathPlanner.getRoute(this, this->position, deploymentLocation, this->route_to_best_frontier);
                if (this->route_to_best_frontier.empty()) { //If there is no route to the deployment location, we reset the current best frontier
                    this->currentBestFrontier = Coordinate{MAXFLOAT, MAXFLOAT};
                }
                this->last_feasibility_check_tick = this->elapsed_ticks;

            }
#endif
#if defined SKIP_UNREACHABLE_FRONTIERS && defined RANDOM_WALK_WHEN_NO_FRONTIERS
            if (frontierEvaluator.avoidingCoordinate(this, this->deploymentLocation)) {
                this->currentBestFrontier = Coordinate{MAXFLOAT, MAXFLOAT};
            }
            if (randomWalker.randomWalking && randomWalker.randomWalkedFarEnough(this)) {
                //Force reset of frontier avoidance
                this->frontierEvaluator.resetFrontierAvoidance(this, {0,0});
            }
#endif
        }

    } else assert(0 && "Shouldn't be in any other state");


#else

#ifdef WALL_FOLLOWING_ENABLED
    if (this->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT} || wallFollower.wallFollowingDirection == 0) {
        //Find new frontier
        targetVector = ForceVectorCalculator::calculateUnexploredFrontierVector(this);
    }
#else
    //Find new frontier
        targetVector = ForceVectorCalculator::calculateUnexploredFrontierVector(this);
#endif
    } else if (this->state == State::RETURNING) {
        targetVector = argos::CVector2(this->deploymentLocation.x - this->position.x,
                                       this->deploymentLocation.y - this->position.y);

        //If we are close to the deployment location, we have returned, we can stop
        if (targetVector.Length() <= this->deployment_location_reach_distance) {
            this->state = State::FINISHED_EXPLORING;
        } else {
            //If we are getting closer to the deployment location, we can decrease the distance to reach it, 1cm at a time
            if (targetVector.Length() < this->min_distance_to_deployment_location) {
                this->deployment_location_reach_distance = std::max(this->deployment_location_reach_distance - 0.01, 0.1);
                this->min_distance_to_deployment_location = targetVector.Length();
            } //If we are not getting closer, we can increase the distance to reach it, 0.2cm at a time
            else {
                this->deployment_location_reach_distance += 0.002;
            }

            //Set our current best 'frontier' to the deployment location
            if (!(this->currentBestFrontier == this->deploymentLocation)) {
                this->currentBestFrontier = this->deploymentLocation;
                this->last_feasibility_check_tick = this->elapsed_ticks;

#ifdef PATH_PLANNING_ENABLED
                periodic_check_required = true;
#endif
            }
#ifdef PATH_PLANNING_ENABLED
            if (periodic_check_required) {
                this->route_to_best_frontier.clear();
                this->pathPlanner.getRoute(this, this->position, deploymentLocation, this->route_to_best_frontier);
                if (this->route_to_best_frontier.empty()) { //If there is no route to the deployment location, we reset the current best frontier
                    this->currentBestFrontier = Coordinate{MAXFLOAT, MAXFLOAT};
                }
                this->last_feasibility_check_tick = this->elapsed_ticks;

            }
#endif
#if defined SKIP_UNREACHABLE_FRONTIERS && defined RANDOM_WALK_WHEN_NO_FRONTIERS
            if (frontierEvaluator.avoidingCoordinate(this, this->deploymentLocation)) {
                this->currentBestFrontier = Coordinate{MAXFLOAT, MAXFLOAT};
            }
            if (randomWalker.randomWalking && randomWalker.randomWalkedFarEnough(this)) {
                //Force reset of frontier avoidance
                this->frontierEvaluator.resetFrontierAvoidance(this, {0,0});
            }
#endif
        }
    } else assert(0 && "Shouldn't be in any other state");
#endif
#if defined(RANDOM_WALK_WHEN_NO_FRONTIERS) || defined(PATH_PLANNING_ENABLED)
    bool noTarget = this->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT};
#ifdef RANDOM_WALK_WHEN_NO_FRONTIERS
    if (noTarget) {
        randomWalker.randomWalk(this, targetVector);
    } else {
        randomWalker.randomWalking = false;
        this->subTarget = {MAXFLOAT, MAXFLOAT};
    }
#endif

#ifdef PATH_PLANNING_ENABLED
    if (!noTarget) { //If there is a target
        if (!this->route_to_best_frontier.empty()) {
            Coordinate nextPathTarget = this->pathFollower.followPath(this);
            assert(!(nextPathTarget == Coordinate{MAXFLOAT, MAXFLOAT}));
            this->subTarget = nextPathTarget;
            targetVector = argos::CVector2(this->subTarget.x - this->position.x,
                                           this->subTarget.y - this->position.y);
        }
    }
#endif
#endif


    ForceVectorCalculator::vectors vectors{this->swarm_vector, virtualWallAvoidanceVector, agentCohesionVector,
                                           agentAvoidanceVector, agentAlignmentVector, targetVector};

    ForceVectorCalculator::checkAvoidAndNormalizeVectors(vectors);

    argos::CVector2 total_vector;
    argos::CRadians objectAvoidanceAngle;
    bool isThereAFreeAngle = ForceVectorCalculator::calculateObjectAvoidanceAngle(this, &objectAvoidanceAngle, vectors,
                                                                                  total_vector,
                                                                                  false);//targetVector.Length() == 0);
#ifdef SKIP_UNREACHABLE_FRONTIERS
//    if (this->state == State::EXPLORING)
        frontierEvaluator.skipIfFrontierUnreachable(this, objectAvoidanceAngle, total_vector);
#endif
    //If there is not a free angle to move to, do not move
    if (!isThereAFreeAngle) {
        this->force_vector = {0, 0};
    } else {


// According to the paper, the formula should be:
//        this->force_vector = {(this->previous_total_vector.GetX()) *
//                              argos::Cos(objectAvoidanceAngle) -
//                              (this->previous_total_vector.GetX()) *
//                              argos::Sin(objectAvoidanceAngle),
//                              (this->previous_total_vector.GetY()) *
//                              argos::Sin(objectAvoidanceAngle) +
//                              (this->previous_total_vector.GetY()) *
//                              argos::Cos(objectAvoidanceAngle)};

// However, according to theory (https://en.wikipedia.org/wiki/Rotation_matrix) (https://www.purplemath.com/modules/idents.htm), the formula should be:
// This also seems to give better performance.
// Edit: contacted the writer, he said below is correct
        this->force_vector = {(total_vector.GetX()) *
                              argos::Cos(objectAvoidanceAngle) -
                              (total_vector.GetY()) *
                              argos::Sin(objectAvoidanceAngle),
                              (total_vector.GetX()) *
                              argos::Sin(objectAvoidanceAngle) +
                              (total_vector.GetY()) *
                              argos::Cos(objectAvoidanceAngle)};

        argos::CRadians angle = this->force_vector.Angle();
        this->targetHeading = angle;
    }
    this->previousBestFrontier = this->currentBestFrontier;
    this->swarm_vector = total_vector;
}


void Agent::sendQuadtreeToCloseAgents() {
    std::vector<std::string> quadTreeToStrings = {};
    bool sendQuadtree = false;
    double oldest_exchange = MAXFLOAT;
    for (const auto &agentLocationPair: this->agentLocations) {
        double lastReceivedTick = std::get<2>(agentLocationPair.second);
        //If we have received the location of this agent in the last AGENT_LOCATION_RELEVANT_DURATION_S seconds (so it is probably within communication range)
        if ((this->elapsed_ticks - lastReceivedTick) / this->ticks_per_second <
            this->config.AGENT_LOCATION_RELEVANT_S) {
            //If we have not sent the quadtree to this agent yet in the past QUADTREE_EXCHANGE_INTERVAL_S seconds, send it
            if (!this->agentQuadtreeSent.count(agentLocationPair.first) ||
                this->elapsed_ticks - this->agentQuadtreeSent[agentLocationPair.first] >
                        (this->config.QUADTREE_EXCHANGE_INTERVAL_S + (rand() % 400 - 200)/100.0f) * this->ticks_per_second) { //Randomize the quadtree exchange interval a bit (between -2 and +2 seconds)
                sendQuadtree = true; //We need to send the quadtree to at least one agent
                break; //We know we have to broadcast the quadtree, so we can break
            }
        }
    }


    if (!sendQuadtree) return; //If we don't need to send the quadtree to any agent, return

    //Update the exchange time for all agents within range
    for (const auto &agentLocationPair: this->agentLocations) {
        double lastReceivedTick = std::get<2>(agentLocationPair.second);
        if ((this->elapsed_ticks - lastReceivedTick) / this->ticks_per_second <
            this->config.AGENT_LOCATION_RELEVANT_S) {
            //Find the oldest exchange, so we broadcast the quadtree with info that the agent of the oldest exchange has not received yet.
            oldest_exchange = std::min(oldest_exchange, this->agentQuadtreeSent[agentLocationPair.first]);
            this->agentQuadtreeSent[agentLocationPair.first] = this->elapsed_ticks; //We will be sending, so update the time we have sent the quadtree to this agent

        }
    }

    this->quadtree->toStringVector(&quadTreeToStrings, oldest_exchange/this->ticks_per_second);
    for (const std::string &str: quadTreeToStrings) {
        broadcastMessage("M:" + str);
    }

}

void Agent::timeSyncWithCloseAgents() {
    if (this->elapsed_ticks - this->timeSynchronizer.getLastSyncAttempt() >=
        (this->config.TIME_SYNC_INTERVAL_S + (rand() % 400 - 200)/100.0f) * this->ticks_per_second) { //Randomize the time sync interval a bit (between -2 and +2 seconds)
        for (const auto &agentLocationPair: this->agentLocations) {
            double lastReceivedTick = std::get<2>(agentLocationPair.second);
            //If we have received the location of this agent in the last AGENT_LOCATION_RELEVANT_DURATION_S seconds (so it is probably within communication range), broadcast time sync init
            if ((this->elapsed_ticks - lastReceivedTick) / this->ticks_per_second <
                this->config.AGENT_LOCATION_RELEVANT_S) {
                //If we have not time synced with this agent in the past TIME_SYNC_INTERVAL_S seconds, do it

                timeSynchronizer.initTimeSync(this); //Broadcasting time sync init
                break; //Time sync is broadcasted, so we can break
            }
        }
    }
}

void Agent::startMission() {
    this->state = State::EXPLORING;
    this->deploymentLocation = this->position;
}

void Agent::doStep() {
    // printf("Agent %s: %d\n", this->getId().c_str(), this->elapsed_ticks);
    // printf("resolution: %f\n", this->quadtree->getResolution());
    if (this->state != State::NO_MISSION) { //Don't do anything before mission
        broadcastMessage("C:" + this->position.toString() + "|" + this->currentBestFrontier.toString());
        sendQuadtreeToCloseAgents();
        argos::CVector2 velocity = {1, 0};
        velocity.Rotate(this->heading);
        broadcastMessage(
                "V:" + std::to_string(velocity.GetX()) + ";" + std::to_string(velocity.GetY()));
        timeSyncWithCloseAgents();

        checkMessages();
    }
    if (this->state == State::NO_MISSION || this->state == State::FINISHED_EXPLORING || this->state == State::MAP_RELAYED) {
        //Do nothing
        this->differential_drive.stop();
        if (this->state == State::FINISHED_EXPLORING) {
            //If we are finished exploring, and any other agent is within communication range, and we have exchanged recently, we are fully done.
            bool map_relayed = false;
            if (this->agentLocations.empty()) map_relayed = true; //Have never encountered another agent, so we assume there are no others
            for (const auto &agentLocationPair: this->agentLocations) {
                double lastReceivedTick = std::get<2>(agentLocationPair.second);
                //If we have received the location of this agent in the last AGENT_LOCATION_RELEVANT_DURATION_S seconds (so it is probably within communication range)
                if ((this->elapsed_ticks - lastReceivedTick) / this->ticks_per_second < this->config.AGENT_LOCATION_RELEVANT_S) {
                    //If we have sent the quadtree to this agent in the past QUADTREE_EXCHANGE_INTERVAL_S seconds, we assume we have received it back
                    //So we are done
                    if (this->agentQuadtreeSent.count(agentLocationPair.first) &&
                        this->elapsed_ticks - this->agentQuadtreeSent[agentLocationPair.first] <
                        this->config.QUADTREE_EXCHANGE_INTERVAL_S * this->ticks_per_second) {
                        map_relayed = true;
                        break;
                    }
                }
            }
            if (map_relayed) {
                this->state = State::MAP_RELAYED;
            }
        }
    } else { //Exploring or returning
        if (this->elapsed_ticks >= this->ticks_per_second) { //Wait one second before starting, allowing initial communication
            checkForObstacles();
            calculateNextPosition();
            //If there is no force vector, do not move
            if (this->force_vector == argos::CVector2{0, 0}) this->differential_drive.stop();
            else {

                argos::CRadians diff = (this->heading - this->targetHeading).SignedNormalize();

                argos::CDegrees diffDeg = ToDegrees(diff);


                if (diffDeg > argos::CDegrees(-this->config.TURN_THRESHOLD_DEGREES) &&
                    diffDeg < argos::CDegrees(this->config.TURN_THRESHOLD_DEGREES)) {
                    //Go straight
                    this->differential_drive.forward();
                } else if (diffDeg > argos::CDegrees(0)) {
                    //turn right
                    this->differential_drive.turnRight();
                } else {
                    //turn left
                    this->differential_drive.turnLeft();
                }
            }
        }

    }
    if (this->state != State::NO_MISSION) { //Don't update ticks before mission has started.
        //Check if the mission has ended, and if so, we will return to the deployment location
        checkMissionEnd();
        this->elapsed_ticks++;
    }
}


/**
 * Broadcast a message to all agents
 * @param message
 */
void Agent::broadcastMessage(const std::string &message) const {
    std::string messagePrependedWithId = "[" + getId() + "]" + message;
    this->wifi.broadcast_message(messagePrependedWithId);
}

/**
 * Sends a message to an agents
 * @param message
 */
void Agent::sendMessage(const std::string &message, const std::string &targetId) {
    std::string messagePrependedWithId = "[" + getId() + "]" + message;
    this->wifi.send_message(messagePrependedWithId, targetId);
}

/**
 * Check for messages from other agents
 * If there are messages, parse them
 */
void Agent::checkMessages() {
    //Read messages from other agents
    this->wifi.receive_messages(this->messages, this->elapsed_ticks / this->ticks_per_second);
    if (!this->messages.empty()) parseMessages();

}

/**
 * Get the target id from a messaged
 * @param message
 * @return
 */
std::string getTargetIdFromMessage(const std::string &message) {
    return message.substr(message.find('<') + 1, message.find('>') - 1 - message.find('<'));

}

/**
 * Get the id from a message
 * @param message
 * @return
 */
std::string getIdFromMessage(const std::string &message) {
    return message.substr(message.find('[') + 1, message.find(']') - 1 - message.find('['));

}

std::vector<std::string> splitString(const std::string &str, const std::string &delimiter) {
    std::vector<std::string> strings;
    size_t pos = 0;
    size_t lastPos = 0;
    while ((pos = str.find(delimiter, lastPos)) != std::string::npos) {
        strings.push_back(str.substr(lastPos, pos - lastPos));
        lastPos = pos + delimiter.length();
    }
    strings.push_back(str.substr(lastPos, str.length() - lastPos));
    return strings;
}

/**
 * Get a coordinate from a string
 * @param str
 * @return
 */
Coordinate coordinateFromString(std::string str) {
    std::string delimiter = ";";
    size_t pos = 0;
    std::string token;
    pos = str.find(delimiter);
    token = str.substr(0, pos);
    Coordinate newCoordinate{};
    newCoordinate.x = std::stod(token);
    str.erase(0, pos + delimiter.length());
    newCoordinate.y = std::stod(str);
    return newCoordinate;
}

/**
 * Get a QuadNode from a string
 * @param str
 * @return
 */
quadtree::QuadNode quadNodeFromString(std::string str) {
    std::string visitedDelimiter = "@";
    std::string occDelimiter = ":";
    size_t visitedPos = 0;
    size_t occPos = 0;
    std::string coordinate;
    std::string confidence;
    std::string visited;
    occPos = str.find(occDelimiter);
    coordinate = str.substr(0, occPos);
    quadtree::QuadNode newQuadNode{};
    newQuadNode.coordinate = coordinateFromString(coordinate);
    str.erase(0, occPos + occDelimiter.length());
    //Now we have the occupancy and ticks

    visitedPos = str.find(visitedDelimiter);
    confidence = str.substr(0, visitedPos);
    newQuadNode.LConfidence = static_cast<float>(std::stod(confidence));
    str.erase(0, visitedPos + visitedDelimiter.length());
    visited = str;
    newQuadNode.visitedAtS = std::stod(visited);
    return newQuadNode;
}


argos::CVector2 vector2FromString(std::string str) {
    std::string delimiter = ";";
    size_t pos = 0;
    std::string token;
    pos = str.find(delimiter);
    token = str.substr(0, pos);
    argos::CVector2 newVector;
    newVector.SetX(std::stod(token));
    str.erase(0, pos + delimiter.length());
    newVector.SetY(std::stod(str));
    return newVector;
}


/**
 * Parse messages from other agents
 */
void Agent::parseMessages() {
    std::map<std::string, int> agentQuadtreeBytesReceivedCounter;
    for (const std::string &message: this->messages) {
        std::string senderId = getIdFromMessage(message);
        if (senderId == this->getId()){
            continue; //If the message is from this agent, skip it
        }         
        std::string targetId = getTargetIdFromMessage(message);
        if (targetId != "A" && targetId != getId())
            continue; //If the message is not broadcast to all agents and not for this agent, skip it

        std::string messageContent = message.substr(message.find(']') + 1);
        if (messageContent.at(0) == 'C') {
            auto splitStrings = splitString(messageContent.substr(2), "|");
            Coordinate receivedPosition = coordinateFromString(splitStrings[0]);
            Coordinate receivedFrontier = coordinateFromString(splitStrings[1]);
            this->agentLocations[senderId] = std::make_tuple(receivedPosition, receivedFrontier, this->elapsed_ticks);
        } else if (messageContent.at(0) == 'M') {
            std::vector<std::string> chunks;
            std::stringstream ss(messageContent.substr(2));
            std::string chunk;
            //Keep track of the total amount of bytes received from the sender
            if (agentQuadtreeBytesReceivedCounter.count(senderId) ==
                0) { //If the sender has not sent any bytes this tick yet
                agentQuadtreeBytesReceivedCounter[senderId] = messageContent.size();
            } else {
                agentQuadtreeBytesReceivedCounter[senderId] += messageContent.size();
            }
            while (std::getline(ss, chunk, '|')) {
                quadtree::QuadNode newQuadNode = quadNodeFromString(chunk);
                quadtree::Box addedBox = this->quadtree->addFromOther(newQuadNode,
                                                             elapsed_ticks / ticks_per_second);
#ifdef CLOSE_SMALL_AREAS
                if (newQuadNode.occupancy == quadtree::OCCUPIED && addedBox.getSize() != 0) // If the box is not the zero (not added)
                    checkIfAgentFitsBetweenObstacles(addedBox);
#endif

            }
        } else if (messageContent[0] == 'V') {
            std::string vectorString = messageContent.substr(2);
            std::string delimiter = ":";
            size_t speedPos = 0;
            std::string vector;
            speedPos = vectorString.find(delimiter);
            vector = vectorString.substr(0, speedPos);
            argos::CVector2 newVector = vector2FromString(vector);
            agentVelocities[senderId] = newVector;
        } else if (messageContent[0] == 'T') { //Time sync message
            std::string timeSyncString = messageContent.substr(2);
            auto splitStrings = splitString(timeSyncString, "|");
            int messageType = std::stoi(splitStrings[0]);
            switch (messageType) {
                case 0: {
                    int t_TXi = std::stoi(splitStrings[1]);
                    timeSynchronizer.respondToTimeSync(senderId, this, t_TXi);
                    break;
                }
                case 1: {
                    int t_TXi = std::stoi(splitStrings[1]);
                    int t_RXj = std::stoi(splitStrings[2]);
                    int t_TXj = std::stoi(splitStrings[3]);
                    timeSynchronizer.determineT_RXi(senderId, this, t_TXi, t_RXj, t_TXj);
                    break;
                }
                case 2: {
                    int t_RXi = std::stoi(splitStrings[1]);
                    timeSynchronizer.receiveT_RXi(senderId, this, t_RXi);
                    break;
                }
                default : {
                    assert(0 && "Unknown time sync message type");
                }
            }

        }
    }
    //Update the total amount of bytes received from each sender
    for (auto &it: agentQuadtreeBytesReceivedCounter) {
        this->agentQuadtreeBytesReceived[it.first] = it.second;
    }

}

Radio Agent::getWifi() const {
    return this->wifi;
}

void Agent::setWifi(Radio newWifi) {
    this->wifi = newWifi;
    this->wifi.config(this->config.WIFI_SPEED_MBPS, this->config.MAX_JITTER_MS, this->config.MESSAGE_LOSS_PROBABILITY);

}

std::vector<std::string> Agent::getMessages() {
    return this->messages;
}

/**
 * Check if the mission needs to end
 * Either due to time or battery level
 */
void Agent::checkMissionEnd() {
    if (this->state == State::RETURNING || this->state == State::FINISHED_EXPLORING) {
        //If we have are returning to the deployment location, but we are taking over double the time at which we return, we have failed (finished).
        if (this->elapsed_ticks / this->ticks_per_second > this->config.MISSION_END_TIME_S * 2.0f) {
            this->state = State::MAP_RELAYED;
        }
    } else if (this->state == State::NO_MISSION  || this->state == State::MAP_RELAYED){
      return;
    }
    else {
        auto charge = this->batteryManager.battery.getStateOfCharge() * 100.0;
        if (this->elapsed_ticks / this->ticks_per_second > this->config.MISSION_END_TIME_S) {
            this->state = State::RETURNING;
            this->config.AGENT_ALIGNMENT_WEIGHT = 0.0;
            this->min_distance_to_deployment_location = sqrt(pow(this->deploymentLocation.x - this->position.x, 2) +
                                                             pow(this->deploymentLocation.y - this->position.y, 2));
        } else if (charge < this->config.MISSION_END_BATTERY_LEVEL) {
            this->state = State::RETURNING;
            this->config.AGENT_ALIGNMENT_WEIGHT = 0.0;
            this->min_distance_to_deployment_location = sqrt(pow(this->deploymentLocation.x - this->position.x, 2) +
                                                             pow(this->deploymentLocation.y - this->position.y, 2));
        }
    }
}

void Agent::loadConfig(const std::string& config_file, double rootbox_size) {
    // YAML::Node config_yaml = YAML::LoadFile(config_file);
    YAML::Node config_yaml = YAML::Load(reinterpret_cast<const char*>(_home_hugo_Documents_pico2w_exp_main_BICLARE_Hardware_src_agent_implementation_configs_hardware_config_yaml));
    this->config.MISSION_END_TIME_S = config_yaml["mission"]["end_time"].as<float>();
    this->config.MISSION_END_BATTERY_LEVEL = config_yaml["mission"]["end_battery_level"].as<float>();

    this->config.ROBOT_WEIGHT = config_yaml["physical"]["robot_weight"].as<float>();
    this->config.ROBOT_WHEEL_RADIUS = config_yaml["physical"]["robot_wheel_radius"].as<float>();
    this->config.ROBOT_INTER_WHEEL_DISTANCE = config_yaml["physical"]["robot_inter_wheel_distance"].as<float>();

    this->config.TURN_THRESHOLD_DEGREES = config_yaml["control"]["turn_threshold"].as<double>();
    this->config.TURNING_SPEED_RATIO = config_yaml["control"]["turn_speed_ratio"].as<float>();
    this->config.STEPS_360_DEGREES = config_yaml["control"]["360_degrees_steps"].as<double>();

    this->config.ROBOT_RADIUS = config_yaml["physical"]["robot_radius"].as<double>();
    this->config.AGENT_SAFETY_RADIUS = this->config.ROBOT_RADIUS +
                                       config_yaml["control"]["agent_safety_radius_margin"].as<double>();

    this->config.FRONTIER_DIST_UNTIL_REACHED = config_yaml["control"]["disallow_frontier_switching"]["frontier_reach_distance"].as<double>();
#ifdef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
    this->config.PERIODIC_FEASIBILITY_CHECK_INTERVAL_S = config_yaml["control"]["disallow_frontier_switching"]["target_feasibility_check_interval"].as<float>();
    this->config.FRONTIER_SWITCH_INTERVAL_S = config_yaml["control"]["disallow_frontier_switching"]["frontier_switch_interval"].as<float>();
    this->config.FEASIBILITY_CHECK_ONLY_ROUTE = config_yaml["control"]["disallow_frontier_switching"]["feasibility_check_only_route"].as<bool>();
#endif
#if defined(SEPARATE_FRONTIERS) || defined(SKIP_UNREACHABLE_FRONTIERS)
    this->config.FRONTIER_SEPARATION_THRESHOLD = config_yaml["control"]["separate_frontiers"]["distance_threshold"].as<float>();
#endif
#ifdef PATH_PLANNING_ENABLED
    this->config.MAX_ROUTE_LENGTH = config_yaml["control"]["path_planning"]["max_route_length"].as<int>();
#endif
    this->config.AGENT_LOCATION_RELEVANT_S = config_yaml["communication"]["agent_info_relevant"].as<double>();
    this->config.QUADTREE_EXCHANGE_INTERVAL_S = config_yaml["communication"]["quadtree_exchange_interval"].as<double>();
    this->config.TIME_SYNC_INTERVAL_S = config_yaml["communication"]["time_sync_interval"].as<double>();

    this->config.ORIENTATION_NOISE_DEGREES = config_yaml["sensors"]["orientation_noise"].as<double>();
    this->config.ORIENTATION_JITTER_DEGREES = config_yaml["sensors"]["orientation_jitter"].as<double>();
    this->config.DISTANCE_SENSOR_JITTER_CM = config_yaml["sensors"]["distance_sensor_jitter"].as<double>();
    this->config.DISTANCE_SENSOR_NOISE_FACTOR = config_yaml["sensors"]["distance_sensor_noise_factor"].as<double>();
    this->config.POSITION_NOISE_CM = config_yaml["sensors"]["position_noise"].as<double>();
    this->config.POSITION_JITTER_CM = config_yaml["sensors"]["position_jitter"].as<double>();
    this->config.DISTANCE_SENSOR_PROXIMITY_RANGE = config_yaml["sensors"]["distance_sensor_range"].as<double>();
//
    this->config.FRONTIER_SEARCH_RADIUS = config_yaml["forces"]["frontier_search_radius"].as<double>();
//    this->config.MAX_FRONTIER_CELLS = config_yaml["forces"]["max_frontier_cells"].as<int>();
    this->config.FRONTIER_CELL_RATIO = config_yaml["forces"]["frontier_cell_ratio"].as<double>();
    this->config.MAX_FRONTIER_REGIONS = config_yaml["forces"]["max_frontier_regions"].as<int>();
    this->config.AGENT_AVOIDANCE_RADIUS = config_yaml["forces"]["agent_avoidance_radius"].as<double>();
    this->config.AGENT_COHESION_RADIUS = config_yaml["forces"]["agent_cohesion_radius"].as<double>();
    this->config.AGENT_ALIGNMENT_RADIUS = config_yaml["forces"]["agent_alignment_radius"].as<double>();

    this->config.VIRTUAL_WALL_AVOIDANCE_WEIGHT = config_yaml["forces"]["virtual_wall_avoidance_weight"].as<double>();
    this->config.AGENT_COHESION_WEIGHT = config_yaml["forces"]["agent_cohesion_weight"].as<double>();
    this->config.AGENT_AVOIDANCE_WEIGHT = config_yaml["forces"]["agent_avoidance_weight"].as<double>();
    this->config.AGENT_ALIGNMENT_WEIGHT = config_yaml["forces"]["agent_alignment_weight"].as<double>();
    this->config.TARGET_WEIGHT = config_yaml["forces"]["target_weight"].as<double>();

    this->config.FRONTIER_DISTANCE_WEIGHT = config_yaml["forces"]["frontier_fitness"]["distance_weight"].as<double>();
    this->config.FRONTIER_SIZE_WEIGHT = config_yaml["forces"]["frontier_fitness"]["size_weight"].as<double>();
    this->config.FRONTIER_REACH_BATTERY_WEIGHT = config_yaml["forces"]["frontier_fitness"]["reach_battery_weight"].as<double>();
    this->config.FRONTIER_REACH_DURATION_WEIGHT = config_yaml["forces"]["frontier_fitness"]["reach_duration_weight"].as<double>();
    this->config.FRONTIER_PHEROMONE_WEIGHT = config_yaml["forces"]["frontier_fitness"]["pheromone_weight"].as<double>();
    this->config.FRONTIER_PHEROMONE_K = config_yaml["forces"]["frontier_fitness"]["k"].as<double>();
    this->config.FRONTIER_PHEROMONE_N = config_yaml["forces"]["frontier_fitness"]["n"].as<double>();
    this->config.FRONTIER_PHEROMONE_M = config_yaml["forces"]["frontier_fitness"]["m"].as<double>();
    this->config.FRONTIER_PHEROMONE_L = config_yaml["forces"]["frontier_fitness"]["l"].as<double>();

    this->config.P_FREE = config_yaml["confidence"]["p_free"].as<double>();
    this->config.P_OCCUPIED = config_yaml["confidence"]["p_occupied"].as<double>();
    this->config.ALPHA_RECEIVE = config_yaml["confidence"]["alpha_receive"].as<float>();
    this->config.P_FREE_THRESHOLD = config_yaml["confidence"]["p_free_threshold"].as<float>();
    this->config.P_OCCUPIED_THRESHOLD = config_yaml["confidence"]["p_occupied_threshold"].as<float>();
    this->config.P_MAX = config_yaml["confidence"]["p_max"].as<float>();
    this->config.P_MIN = config_yaml["confidence"]["p_min"].as<float>();
    this->config.P_AT_MAX_SENSOR_RANGE = config_yaml["confidence"]["p_at_max_sensor_range"].as<float>();

    this->config.QUADTREE_RESOLUTION = config_yaml["quadtree"]["resolution"].as<double>();
    this->config.QUADTREE_EVAPORATION_TIME_S = config_yaml["quadtree"]["evaporation_time"].as<double>();
    this->config.QUADTREE_EVAPORATED_PHEROMONE_FACTOR = config_yaml["quadtree"]["evaporated_pheromone_factor"].as<double>();
    this->config.QUADTREE_MERGE_MAX_VISITED_TIME_DIFF = config_yaml["quadtree"]["merge_max_visited_time_difference"].as<double>();
    this->config.QUADTREE_MERGE_MAX_P_CONFIDENCE_DIFF = config_yaml["quadtree"]["merge_max_confidence_diff"].as<double>();

    //Calculate smallest box size
    double smallestBoxSize = rootbox_size;
    while (smallestBoxSize > this->config.QUADTREE_RESOLUTION) {
        smallestBoxSize /= 2;
    }
    this->config.OBJECT_SAFETY_RADIUS = smallestBoxSize * config_yaml["control"]["object_safety_factor"].as<double>();
    this->config.OBJECT_AVOIDANCE_RADIUS = this->config.AGENT_SAFETY_RADIUS + this->config.OBJECT_SAFETY_RADIUS +
                                           config_yaml["forces"]["object_avoidance_radius_margin"].as<double>();


    this->config.BATTERY_CAPACITY = config_yaml["battery"]["capacity"].as<double>();
    this->config.BATTERY_VOLTAGE = config_yaml["battery"]["voltage"].as<double>();

    this->config.MOTOR_STALL_CURRENT = config_yaml["motor"]["stall_current"].as<double>();
    this->config.MOTOR_STALL_TORQUE = config_yaml["motor"]["stall_torque"].as<double>();
    this->config.MOTOR_NO_LOAD_RPM = config_yaml["motor"]["no_load_rpm"].as<double>();
    this->config.MOTOR_NO_LOAD_CURRENT = config_yaml["motor"]["no_load_current"].as<double>();

    this->config.WIFI_SPEED_MBPS = config_yaml["communication"]["wifi_speed"].as<double>();
    this->config.WIFI_RANGE_M = config_yaml["communication"]["wifi_range"].as<double>();
    this->config.MAX_JITTER_MS = config_yaml["communication"]["max_jitter"].as<double>();
    this->config.MESSAGE_LOSS_PROBABILITY = config_yaml["communication"]["message_loss_probability"].as<double>();
}


