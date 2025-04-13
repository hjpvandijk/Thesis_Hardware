#ifndef IMPLEMENTATION_AND_EXAMPLES_WALLFOLLOWER_H
#define IMPLEMENTATION_AND_EXAMPLES_WALLFOLLOWER_H

#include "agent_implementation/feature_config.h"
#include "agent_implementation/utils/CustomComparator.h"
#include "agent_implementation/utils/coordinate.h"
#include "ForceVectorCalculator.h"

class Agent;

class WallFollower {
#ifdef WALL_FOLLOWING_ENABLED

public:
    int wallFollowingDirection = 0;
    Coordinate wallFollowingSubTarget = {MAXFLOAT, MAXFLOAT};
    int prevWallFollowingDirection = 0;
    Coordinate wallFollowingHitPoint = {MAXFLOAT, MAXFLOAT};

    WallFollower() = default;
    void wallFollowing(Agent* agent, ForceVectorCalculator::vectors vectors, argos::CVector2 & total_vector, std::set<argos::CDegrees>& freeAngles, argos::CDegrees *closestFreeAngle, argos::CRadians *closestFreeAngleRadians, argos::CRadians *relativeObjectAvoidanceAngle, argos::CRadians targetAngle);
#endif
};


#endif //IMPLEMENTATION_AND_EXAMPLES_WALLFOLLOWER_H
