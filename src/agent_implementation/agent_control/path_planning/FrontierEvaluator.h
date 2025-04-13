#ifndef IMPLEMENTATION_AND_EXAMPLES_FRONTIEREVALUATOR_H
#define IMPLEMENTATION_AND_EXAMPLES_FRONTIEREVALUATOR_H

#include "../../feature_config.h"
// #include "argos3/core/utility/math/vector2.h"
#include "vector2.h"
#include "../../utils/coordinate.h"

class Agent;

class FrontierEvaluator {
#ifdef SKIP_UNREACHABLE_FRONTIERS
public:
    std::vector<Coordinate> avoidingCoordinates;

    //If the agent is counts this many ticks without a direction to the target, it will skip the target
    int MAX_COUNT_NO_DIRECTION = 80;

    //Count the number of ticks the agent has no direction to the target
    int countNoDirectionToTarget = 0;

    int cantMoveAngle = 89;


    FrontierEvaluator() = default;
    FrontierEvaluator(int max_count_no_direction);
    FrontierEvaluator(int closest_coordinate_hit_count_before_decreasing_confidence, int max_ticks_no_direction);

    void resetFrontierAvoidance(Agent* agent, argos::CVector2 unexploredFrontierVector);
    bool avoidingAgentTarget(Agent* agent);
    bool avoidingCoordinate(Agent* agent, Coordinate coordinate);
    void skipIfFrontierUnreachable(Agent* agent, argos::CRadians objectAvoidanceAngle, argos::CVector2 total_vector);

private:
    Coordinate previousTarget = {MAXFLOAT, MAXFLOAT};
#endif
    };

#endif //IMPLEMENTATION_AND_EXAMPLES_FRONTIEREVALUATOR_H
