#include <argos3/core/utility/math/vector2.h>
#include <set>
#include <utility>
#include "WallFollower.h"
#include "utils/coordinate.h"
#include "agent.h"

#ifdef WALL_FOLLOWING_ENABLED

/**
 * When the free direction to the target > 89 degrees:
 * Save agent location as the hit point
 * Select a random direction (left or right) to follow the wall
 * We follow the wall until the direction to the target is free again.
 * If we hit the same hitpoint again, change wall following direction.
 * @param freeAngles
 * @param closestFreeAngle
 * @param closestFreeAngleRadians
 * @param relativeObjectAvoidanceAngle
 * @param targetAngle
 */
void WallFollower::wallFollowing(Agent* agent, ForceVectorCalculator::vectors vectors, argos::CVector2 & total_vector, std::set<argos::CDegrees>& freeAngles, argos::CDegrees *closestFreeAngle, argos::CRadians *closestFreeAngleRadians, argos::CRadians *relativeObjectAvoidanceAngle, argos::CRadians targetAngle){
    if (!(agent->subTarget == Coordinate{MAXFLOAT, MAXFLOAT}) ||
        agent->previousBestFrontier == agent->currentBestFrontier) { // If we are still on route to the same frontier
        Coordinate target = !(agent->subTarget == Coordinate{MAXFLOAT, MAXFLOAT}) ? agent->subTarget : agent->currentBestFrontier;
        double agentToTarget = sqrt(pow(target.x - agent->position.x, 2) +
                                    pow(target.y - agent->position.y, 2));
        argos::CVector2 agentToHitPoint = argos::CVector2(agent->position.x - this->wallFollowingHitPoint.x,
                                                          agent->position.y - this->wallFollowingHitPoint.y);
        argos::CVector2 hitPointToTarget = argos::CVector2(this->wallFollowingHitPoint.x - target.x,
                                                           this->wallFollowingHitPoint.y - target.y);
        if (this->wallFollowingDirection == 0 && std::abs(ToDegrees(*relativeObjectAvoidanceAngle).GetValue()) > 89) { //The complete forward direction to the target is blocked

            if (agentToHitPoint.Length() <=
                agent->config.OBJECT_AVOIDANCE_RADIUS) { // If we are at the hit point (again).
                if (this->wallFollowingDirection == 0) {
                    if (this->prevWallFollowingDirection == 0) {
                        this->wallFollowingDirection = rand() % 2 == 0 ? 1 : -1; // Randomly choose a direction
                    } else {
                        this->wallFollowingDirection = -this->prevWallFollowingDirection;
                    }
                }
//                else { // 1 or -1
//                    if (!agent->lastTickInWallFollowingHitPoint) { // If we are not in the same hit point as last iteration (so we have once moved from the hit point)
//                        agent->wallFollowingDirection = -agent->wallFollowingDirection; // Change wall following direction
//                    }
//                }

            } else {
                if(this->wallFollowingDirection == 0) { //Only switch when we are not in wall following mode already
                    this->wallFollowingDirection = rand() % 2 == 0 ? 1 : -1; // Randomly choose a direction
                }
            }
            this->wallFollowingHitPoint = agent->position;
            this->prevWallFollowingDirection = this->wallFollowingDirection;
            //If agent is close to the target, or, we are passing 'behind' the hit point, stop wall following
        } else if (agentToTarget <= agent->quadtree->getResolution()*2 || (this->wallFollowingDirection != 0 && agentToHitPoint.Length()>= agent->quadtree->getResolution() && std::abs(ToDegrees(NormalizedDifference(hitPointToTarget.Angle(), agentToHitPoint.Angle())).GetValue()) <= agent->config.TURN_THRESHOLD_DEGREES)) {
            this->wallFollowingDirection = 0;
        } else if (std::abs(ToDegrees(*relativeObjectAvoidanceAngle).GetValue()) <
                   agent->config.TURN_THRESHOLD_DEGREES) { //Direction to the frontier is free again.
            double hitPointToFrontier = sqrt(pow(this->wallFollowingHitPoint.x - target.x, 2) +
                                             pow(this->wallFollowingHitPoint.y - target.y, 2));
            if (agentToTarget < hitPointToFrontier) { //If we are closer to the frontier than the hit point, or the angle to the target is way different than from the hitpoint to the target.
                this->wallFollowingDirection = 0;
            }
        }
    } else {
        this->wallFollowingDirection = 0;
    }



    //Wall following:
    //Choose free direction closest to current heading
    //If no wall on the wall following side of the robot, turn that way
    //Loop until direction to frontier is free.
    if (wallFollowingDirection != 0) { // If we are in wall following mode
        //Get the closest free angle to the wall following direction (90 degrees right or left)
        //Create a subtarget in that direction
        argos::CDegrees subtargetAngle = *freeAngles.begin();
        CustomComparator customComparator = CustomComparator(this->wallFollowingDirection, ToDegrees(agent->heading).GetValue(), ToDegrees(targetAngle).GetValue());
        for (auto freeAngle: freeAngles) {
            //If the angle is closer to the wall following direction than the current subtarget angle, set the new subtarget angle
            if (customComparator(freeAngle, subtargetAngle)) {
                subtargetAngle = freeAngle;
            }
        }
        //Create subtarget

        if (subtargetAngle != *closestFreeAngle) { //If angle is the same, the order of the free angles set isn't according to wall following yet
            //If the difference between the first free angle and the last is less than 90 degrees, agent will spin indefinitely. So go straight
            argos::CDegrees fst = NormalizedDifference(subtargetAngle, ToDegrees(agent->heading));
            argos::CDegrees snd = NormalizedDifference(*freeAngles.rbegin(), ToDegrees(agent->heading));
            double differenceBeginEnd = NormalizedDifference(fst, snd).GetValue() * this->wallFollowingDirection;
            if (differenceBeginEnd < -1 && differenceBeginEnd >= -90) {
                subtargetAngle = ToDegrees(agent->heading);
            }
        }
        argos::CVector2 subtargetVector = argos::CVector2(1, 0);
        subtargetVector.Rotate(ToRadians(subtargetAngle));
        subtargetVector.Normalize();
        subtargetVector *= agent->config.OBJECT_AVOIDANCE_RADIUS;

        this->wallFollowingSubTarget = {agent->position.x + subtargetVector.GetX(),
                                        agent->position.y + subtargetVector.GetY()};
        //Calculate new total force
        vectors.targetVector = argos::CVector2(this->wallFollowingSubTarget.x - agent->position.x,
                                               this->wallFollowingSubTarget.y - agent->position.y);
        ForceVectorCalculator::checkAvoidAndNormalizeVectors(vectors);
        total_vector = ForceVectorCalculator::calculateTotalVector(agent, vectors);
        targetAngle = total_vector.Angle();

        //Get closest angle to total force

        if (agent->wallFollower.wallFollowingDirection != 0) { //If wall following is not 0, find the closest free angle to the target angle.
            for (auto freeAngle: freeAngles) {
                if (std::abs(NormalizedDifference(ToRadians(freeAngle), targetAngle).GetValue()) <
                    std::abs(NormalizedDifference(ToRadians(*closestFreeAngle), targetAngle).GetValue())) {
                    *closestFreeAngle = freeAngle;
                }
            }
        }
//
////            argos::CVector2 subtargetVector = argos::CVector2(1, 0);
////            subtargetVector.Rotate(ToRadians(subtargetAngle));
////            subtargetVector.Normalize();
////            subtargetVector *= agent->OBJECT_AVOIDANCE_RADIUS;
////            this->wallFollowingSubTarget = {agent->position.x + subtargetVector.GetX(),
////                                            agent->position.y + subtargetVector.GetY()};
//
//            *closestFreeAngle = subtargetAngle;
            *closestFreeAngleRadians = ToRadians(*closestFreeAngle);
            *relativeObjectAvoidanceAngle = NormalizedDifference(*closestFreeAngleRadians, targetAngle);
//        } else {
//            argos::CVector2 subtargetVector = argos::CVector2(1, 0);
//            subtargetVector.Rotate(ToRadians(subtargetAngle));
//            subtargetVector.Normalize();
//            subtargetVector *= agent->OBJECT_AVOIDANCE_RADIUS;
//            this->wallFollowingSubTarget = {agent->position.x + subtargetVector.GetX(),
//                                            agent->position.y + subtargetVector.GetY()};        }
    } else {
#ifdef RANDOM_WALK_WHEN_NO_FRONTIERS
        if (!(agent->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT})) { // If we have a frontier to go to, so not in the walking state
#endif
            this->wallFollowingSubTarget = {MAXFLOAT, MAXFLOAT}; //Rest subtarget
#ifdef RANDOM_WALK_WHEN_NO_FRONTIERS
        }
#endif
    }
}

#endif


