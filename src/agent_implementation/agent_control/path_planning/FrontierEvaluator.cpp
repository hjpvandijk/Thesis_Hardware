#include "FrontierEvaluator.h"
// #include "agent.h"
#include "../../agent.h"

#ifdef SKIP_UNREACHABLE_FRONTIERS


/**
 * Reset the frontiers avoiding list if the agent is close to a target frontier.
 * This gives the agent the chance to select all frontiers (even with low confidence) again.
 * @param unexploredFrontierVector
 */
void FrontierEvaluator::resetFrontierAvoidance(Agent* agent, argos::CVector2 unexploredFrontierVector) {
    //If the agent is close to the frontier, reset all frontiers avoiding flags (we're giving them another chance)
#ifdef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
    if (unexploredFrontierVector.Length() <= agent->config.FRONTIER_DIST_UNTIL_REACHED) {
        this->avoidingCoordinates.clear();
    }
#endif
}

/**
 * Check if the agent's target (frontier) or subtarget is close to a coordinate we are currently avoiding.
 * If we are avoiding one of them, avoid the other one as well.
 * @return
 */
bool FrontierEvaluator::avoidingAgentTarget(Agent* agent){
    if(agent->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT}) return false;
    //If the agent's target is close to a frontier we are currently avoiding
    for (auto coord: this->avoidingCoordinates) {
        if (sqrt(pow(coord.x - agent->currentBestFrontier.x, 2) + pow(coord.y - agent->currentBestFrontier.y, 2)) < agent->config.FRONTIER_SEPARATION_THRESHOLD) {
            //Add subtarget so that we temporarily cannot go similar routes
            if (!(agent->subTarget == Coordinate{MAXFLOAT, MAXFLOAT})) {
                this->avoidingCoordinates.push_back(agent->subTarget);
            }
            return true;
        }
        if (sqrt(pow(coord.x - agent->subTarget.x, 2) + pow(coord.y - agent->subTarget.y, 2)) < agent->config.FRONTIER_SEPARATION_THRESHOLD) {
            //Add current best frontier so that we temporarily cannot go in that direction
            if (!(agent->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT})) {
                this->avoidingCoordinates.push_back(agent->currentBestFrontier);
            }
            return true;
        }
    }
    return false;
}

/**
 * Check if the coordinate is close to a coordinate we are currently avoiding.
 * @return
 */
bool FrontierEvaluator::avoidingCoordinate(Agent* agent, Coordinate coordinate){
    if(coordinate == Coordinate{MAXFLOAT, MAXFLOAT}) return false;
    //If the agent's target is close to a frontier we are currently avoiding
    for (auto coord: this->avoidingCoordinates) {
        if (sqrt(pow(coord.x - coordinate.x, 2) + pow(coord.y - coordinate.y, 2)) < agent->config.FRONTIER_SEPARATION_THRESHOLD) {
            return true;
        }
    }
    return false;
}

/**
 * Update the confidence of cells if they are around a currently unreachable frontier.
 * If the agent is hitting the same hitpoint multiple times, decrease the frontier confidence.
 */
void FrontierEvaluator::skipIfFrontierUnreachable(Agent* agent, argos::CRadians objectAvoidanceAngle, argos::CVector2 total_vector) {
    Coordinate target = agent->currentBestFrontier;
//If we have no frontier (walking state), calculate the distance to the subtarget instead
#ifdef PATH_PLANNING_ENABLED
    if (!(agent->subTarget == Coordinate{MAXFLOAT, MAXFLOAT})) target = agent->subTarget;
#else
    if (agent->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT}) target = agent->subTarget;
#endif
    //If we are random walking, use the subtarget (random walk target) as target
#ifdef RANDOM_WALK_WHEN_NO_FRONTIERS
    if (agent->randomWalker.randomWalking) target = agent->subTarget;
#endif

    if (!(target == Coordinate{MAXFLOAT, MAXFLOAT}) &&
    sqrt(pow(target.x - previousTarget.x, 2) + pow(target.y - previousTarget.y, 2)) < agent->config.FRONTIER_SEPARATION_THRESHOLD/2) { //If we are still on route to almost the same target
        if (std::abs(ToDegrees(objectAvoidanceAngle).GetValue()) > this->cantMoveAngle) { //If we cannot move within 90 degrees towards the target
            this->countNoDirectionToTarget++;
            this->cantMoveAngle = std::min(this->cantMoveAngle + int(agent->ticks_per_second/3), 89);
            if (this->countNoDirectionToTarget >= this->MAX_COUNT_NO_DIRECTION) {
                this->avoidingCoordinates.push_back(target);
                //Also avoid the current best frontier if we are on route to it
                if (target == agent->subTarget && !(agent->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT})) {
                    this->avoidingCoordinates.push_back(agent->currentBestFrontier);
                }
                this->countNoDirectionToTarget = 0;
            }
        } else {
            this->cantMoveAngle = std::max(this->cantMoveAngle-1, 0);
        }

    } else { //If we are not on route to the same frontier, reset the count and angle
        this->countNoDirectionToTarget = 0;
        this->cantMoveAngle = 89;


    }
    previousTarget = target;

}

FrontierEvaluator::FrontierEvaluator(int max_count_no_direction) {
    this->MAX_COUNT_NO_DIRECTION = max_count_no_direction;
}

#endif