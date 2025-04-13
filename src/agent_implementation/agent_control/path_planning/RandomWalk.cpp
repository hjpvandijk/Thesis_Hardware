
#include "RandomWalk.h"
// #include "agent.h"
#include "../../agent.h"

#ifdef RANDOM_WALK_WHEN_NO_FRONTIERS

/**
 * When there is no target to be found within the search range, select a random location on the edge of the root box.
 * @param targetVector
 */
void RandomWalk::randomWalk(Agent* agent, argos::CVector2 &targetVector) {
    argos::CVector2 agentToSubtarget = argos::CVector2(agent->subTarget.x - agent->position.x,
                                                       agent->subTarget.y - agent->position.y);;
    if (!this->randomWalking
        || agent->elapsed_ticks - this->ticksWalkStart > this->maxTicksSamePosition //If we have been in the same position for too long
        #ifdef DISALLOW_FRONTIER_SWITCHING_UNTIL_REACHED
        //Or we have reached the subtarget (random walk target)
        || agentToSubtarget.Length() <= agent->config.FRONTIER_DIST_UNTIL_REACHED
        #endif
        #ifdef SKIP_UNREACHABLE_FRONTIERS
        //Or we are avoiding the subtarget, so we need to find a new one
        || agent->frontierEvaluator.avoidingCoordinate(agent, agent->subTarget)
        #endif
            ) {
        //Find a random direction to walk in, by placing a subtarget on the edge of the root box in the quadtree
        quadtree::Box rootBox = agent->quadtree->getRootBox();
        Coordinate rootBoxCenter = rootBox.getCenter();
        double rootBoxSize = rootBox.getSize();
        int tries = 0;
        do{
            argos::CRadians randomAngle = (rand() % 360) * argos::CRadians::PI /
                                          180;
            argos::CVector2 subtargetVector = argos::CVector2(1, 0);
            subtargetVector.Rotate(randomAngle);
            subtargetVector.Normalize();
            subtargetVector *= rootBoxSize * 0.5;
            agent->subTarget = {rootBoxCenter.x + subtargetVector.GetX(), rootBoxCenter.y + subtargetVector.GetY()};
            agentToSubtarget = argos::CVector2(agent->subTarget.x - agent->position.x,
                                               agent->subTarget.y - agent->position.y);
            this->walkStart = agent->position;
            this->ticksWalkStart = agent->elapsed_ticks;
            tries++;
        } while (
                #ifdef SKIP_UNREACHABLE_FRONTIERS
                //If the subtarget is close to a frontier we are currently avoiding, try again
                agent->frontierEvaluator.avoidingCoordinate(agent, agent->subTarget)
                #else
                false
                #endif
                && tries < 10 //Prevent infinite loop
                );
        this->randomWalking = true;


    }
    targetVector = agentToSubtarget;
}

bool RandomWalk::randomWalkedFarEnough(Agent *agent) {
    //If the difference between walkStart and the agent is far enough, return true
    if (sqrt(pow(agent->position.x - walkStart.x, 2) + pow(agent->position.y - walkStart.y, 2)) >= this->farEnoughDistance) {
        this->walkStart = agent->position;
        return true;
    }
    return false;
}

#endif