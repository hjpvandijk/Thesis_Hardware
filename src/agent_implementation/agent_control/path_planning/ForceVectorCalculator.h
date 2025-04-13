#ifndef IMPLEMENTATION_AND_EXAMPLES_FORCEVECTORCALCULATOR_H
#define IMPLEMENTATION_AND_EXAMPLES_FORCEVECTORCALCULATOR_H

#include "../../feature_config.h"
// #include "argos3/core/utility/math/vector2.h"
#include "vector2.h"
#include "../../utils/coordinate.h"
class Agent;

class ForceVectorCalculator {
public:
    struct vectors{
        argos::CVector2 previous_total_vector;
        argos::CVector2 virtualWallAvoidanceVector;
        argos::CVector2 agentCohesionVector;
        argos::CVector2 agentAvoidanceVector;
        argos::CVector2 agentAlignmentVector;
        argos::CVector2 targetVector;
    };

    ForceVectorCalculator() = default;

    static argos::CVector2 getVirtualWallAvoidanceVector(Agent *agent) ;

    static bool getAverageNeighborLocation(Agent *agent, Coordinate *averageNeighborLocation, double range);

    static argos::CVector2 calculateAgentCohesionVector(Agent *agent);

    static argos::CVector2 calculateAgentAvoidanceVector(Agent *agent);

    static argos::CVector2 calculateAgentAlignmentVector(Agent *agent);

    static argos::CVector2 calculateUnexploredFrontierVector(Agent *agent);


    static argos::CVector2 calculateTotalVector(Agent* agent, vectors vectors);

    static void checkAvoidAndNormalizeVectors(ForceVectorCalculator::vectors &vectors);

    static bool calculateObjectAvoidanceAngle(Agent* agent, argos::CRadians *relativeObjectAvoidanceAngle, ForceVectorCalculator::vectors, argos::CVector2 & total_vector, bool frontier_vector_zero);


};
#endif //IMPLEMENTATION_AND_EXAMPLES_FORCEVECTORCALCULATOR_H
