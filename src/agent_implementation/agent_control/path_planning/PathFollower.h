#ifndef IMPLEMENTATION_AND_EXAMPLES_PATHFOLLOWER_H
#define IMPLEMENTATION_AND_EXAMPLES_PATHFOLLOWER_H

#include "../../feature_config.h"
#include <set>
#include "../../utils/coordinate.h"
#include "../../utils/CustomComparator.h"

class Agent;


class PathFollower {
#ifdef PATH_PLANNING_ENABLED
public:
//    void WallFollower::wallFollowing(Agent* agent, std::set<argos::CDegrees, CustomComparator>& freeAngles, argos::CDegrees *closestFreeAngle, argos::CRadians *closestFreeAngleRadians, argos::CRadians *relativeObjectAvoidanceAngle, argos::CRadians targetAngle) {
    Coordinate followPath(Agent *agent);

private:
    int current_path_section = 0;
    std::vector<std::pair<Coordinate, Coordinate>> current_route;
    bool rayTraceQuadtreeOccupiedIntersection(Agent* agent, Coordinate start, Coordinate target) const;
#endif
};


#endif //IMPLEMENTATION_AND_EXAMPLES_PATHFOLLOWER_H
