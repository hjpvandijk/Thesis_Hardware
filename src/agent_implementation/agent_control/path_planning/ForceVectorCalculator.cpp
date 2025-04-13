#include "ForceVectorCalculator.h"
// #include "agent.h"
#include "../../agent.h"
#include "../../utils/CustomComparator.h"


/** Calculate the vector to avoid the virtual walls
 * If the agent is close to the border, create a vector pointing away from the border
 * Implemented according to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
 * https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
 * However, the vector directions are flipped compared to the paper, as the paper uses a different coordinate system
 * @return a vector pointing away from the border
 *
 */
argos::CVector2 ForceVectorCalculator::getVirtualWallAvoidanceVector(Agent* agent) {
    //If the agent is close to the border, create a vector pointing away from the border
    argos::CVector2 virtualWallAvoidanceVector = {0, 0};

    if (agent->position.x < agent->left_right_borders.x) {
        virtualWallAvoidanceVector.SetX(1);
    } else if (agent->left_right_borders.x <= agent->position.x && agent->position.x <= agent->left_right_borders.y) {
        virtualWallAvoidanceVector.SetX(0);
    } else if (agent->position.x > agent->left_right_borders.y) {
        virtualWallAvoidanceVector.SetX(-1);
    }

    if (agent->position.y< agent->upper_lower_borders.x) {
        virtualWallAvoidanceVector.SetY(1);
    } else if (agent->upper_lower_borders.y <= agent->position.y && agent->position.y <= agent->upper_lower_borders.x) {
        virtualWallAvoidanceVector.SetY(0);
    } else if (agent->position.y > agent->upper_lower_borders.y) {
        virtualWallAvoidanceVector.SetY(-1);
    }


    return virtualWallAvoidanceVector;

}

bool ForceVectorCalculator::getAverageNeighborLocation(Agent* agent, Coordinate *averageNeighborLocation, double range) {
    int nAgentsWithinRange = 0;
    for (const auto &agentLocation: agent->agentLocations) {
        auto agenttime = std::get<2>(agentLocation.second);
        auto diff = (agent->elapsed_ticks - agenttime );
        auto diffSeconds = diff / agent->ticks_per_second;
        if( (agent->elapsed_ticks - std::get<2>(agentLocation.second)) / agent->ticks_per_second > agent->config.AGENT_LOCATION_RELEVANT_S) continue;
        argos::CVector2 vectorToOtherAgent =
                argos::CVector2(std::get<0>(agentLocation.second) .x, std::get<0>(agentLocation.second).y)
                - argos::CVector2(agent->position.x, agent->position.y);

        if (vectorToOtherAgent.Length() < range) {
            averageNeighborLocation->x += std::get<0>(agentLocation.second) .x;
            averageNeighborLocation->y += std::get<0>(agentLocation.second).y;
            nAgentsWithinRange++;
        }
    }
    //If no agents are within range, there is no average location
    if (nAgentsWithinRange == 0) {
        return false;
    }
    //Else calculate the average position of the agents within range
    averageNeighborLocation->x /= nAgentsWithinRange;
    averageNeighborLocation->y /= nAgentsWithinRange;

    return true;
}

argos::CVector2 ForceVectorCalculator::calculateAgentCohesionVector(Agent* agent) {
    Coordinate averageNeighborLocation = {0, 0};

    bool neighborsWithinRange = getAverageNeighborLocation(agent, &averageNeighborLocation, agent->config.AGENT_COHESION_RADIUS);
    if (!neighborsWithinRange) {
        return {0, 0};
    }

    //Create a vector between agent and the average position of the agents within range
    argos::CVector2 vectorToOtherAgents =
            argos::CVector2(averageNeighborLocation.x, averageNeighborLocation.y)
            - argos::CVector2(agent->position.x, agent->position.y);

    return vectorToOtherAgents;
}


/**
 * Calculate the vector to avoid other agents:
 * If the distance between other agents is less than a certain threshold,
 * create a vector in the opposite direction of the average location of these agents.
 * @return a vector pointing away from the average location other agents
 */
argos::CVector2 ForceVectorCalculator::calculateAgentAvoidanceVector(Agent* agent) {

    Coordinate averageNeighborLocation = {0, 0};

    bool neighborsWithinRange = getAverageNeighborLocation(agent, &averageNeighborLocation, agent->config.AGENT_AVOIDANCE_RADIUS);
    if (!neighborsWithinRange) {
        return {0, 0};
    }

    //Create a vector between agent agent and the average position of the agents within range
    argos::CVector2 vectorToOtherAgents =
            argos::CVector2(averageNeighborLocation.x, averageNeighborLocation.y)
            - argos::CVector2(agent->position.x, agent->position.y);

    return vectorToOtherAgents * -1;

//    //Create a vector 90 degrees to the vector between agent and the average position of the agents within range
//    auto xfactor = 1;
//    auto yfactor = -1 * xfactor; //Assert that xfactor and yfactor are orthogonal
//    argos::CVector2 vectorToOtherAgents90Degrees = argos::CVector2(xfactor * vectorToOtherAgents.GetY(), yfactor * vectorToOtherAgents.GetX());

//    return vectorToOtherAgents90Degrees;
}

/**
 * Calculate the vector to align with other agents:
 * If the distance between other agents is less than a certain threshold,
 * create a vector in the direction of the average vector of these agents, considering speed.
 * @return
 */
argos::CVector2 ForceVectorCalculator::calculateAgentAlignmentVector(Agent* agent) {
    argos::CVector2 alignmentVector = {0, 0};
    int nAgentsWithinRange = 0;


    //Get the velocities of the agents within range
    for (const auto &agentVelocity: agent->agentVelocities) {
        std::string agentID = agentVelocity.first;
        Coordinate otherAgentLocation = std::get<0>(agent->agentLocations[agentID]);
        //If the agent has not been updated in the last AGENT_LOCATION_RELEVANT_DURATION_S seconds, skip
        if( (agent->elapsed_ticks - std::get<2>(agent->agentLocations[agentID])) / agent->ticks_per_second > agent->config.AGENT_LOCATION_RELEVANT_S) continue;
        argos::CVector2 agentVector = agentVelocity.second;
        argos::CVector2 vectorToOtherAgent = argos::CVector2(otherAgentLocation.x, otherAgentLocation.y)
                                             - argos::CVector2(agent->position.x, agent->position.y);
        if (vectorToOtherAgent.Length() < agent->config.AGENT_ALIGNMENT_RADIUS) {
            alignmentVector += agentVector;
            nAgentsWithinRange++;
        }
    }

    //If no agents are within range, there is no average velocity
    if (nAgentsWithinRange == 0) {
        return {0, 0};
    }

    //Else calculate the average velocity of the agents within range
    alignmentVector /= nAgentsWithinRange;
    return alignmentVector;
}

argos::CVector2
ForceVectorCalculator::calculateTotalVector(Agent* agent, vectors vectors) {
    vectors.virtualWallAvoidanceVector = agent->config.VIRTUAL_WALL_AVOIDANCE_WEIGHT * vectors.virtualWallAvoidanceVector;
    vectors.agentCohesionVector = agent->config.AGENT_COHESION_WEIGHT * vectors.agentCohesionVector; //Normalize first
    vectors.agentAvoidanceVector = agent->config.AGENT_AVOIDANCE_WEIGHT * vectors.agentAvoidanceVector;
    vectors.agentAlignmentVector = agent->config.AGENT_ALIGNMENT_WEIGHT * vectors.agentAlignmentVector;
    vectors.targetVector = agent->config.TARGET_WEIGHT * vectors.targetVector;

    //According to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
    //https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
    argos::CVector2 total_vector = vectors.previous_total_vector +
                                   vectors.virtualWallAvoidanceVector +
                                   vectors.agentCohesionVector +
                                   vectors.agentAvoidanceVector +
                                   vectors.agentAlignmentVector +
                                   vectors.targetVector;
    if (total_vector.Length() != 0) total_vector.Normalize();

    return total_vector;
}

// Union-Find (Disjoint-Set) data structure
class UnionFind {
public:
    void add(int x) {
        if (parent.find(x) == parent.end()) {
            parent[x] = x;
            rank[x] = 0;
        }
    }

    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    }

    void unionSets(int x, int y) {
        int rootX = find(x);
        int rootY = find(y);
        if (rootX != rootY) {
            if (rank[rootX] > rank[rootY]) {
                parent[rootY] = rootX;
            } else if (rank[rootX] < rank[rootY]) {
                parent[rootX] = rootY;
            } else {
                parent[rootY] = rootX;
                rank[rootX]++;
            }
        }
    }

private:
    std::unordered_map<int, int> parent;
    std::unordered_map<int, int> rank;
};

void truncateMaxRegions(std::vector <std::vector<std::pair<quadtree::Box, double>>> &frontierRegions, int max_regions) {
    //Truncate the regions to the maximum amount of regions, by removing the smallest regions
    if (frontierRegions.size() > max_regions) {
        std::sort(frontierRegions.begin(), frontierRegions.end(), [](const auto &a, const auto &b) {
            return a.size() > b.size();
        });
        frontierRegions.resize(max_regions);
    }
}

void mergeAdjacentFrontiers(const std::vector<std::pair<quadtree::Box, double>> &frontiers,
                            std::vector<std::vector<std::pair<quadtree::Box, double>>> &frontierRegions, int max_regions) {
    UnionFind uf;
    std::unordered_map<int, std::pair<quadtree::Box, double>> boxMap;

    // Initialize union-find structure
    for (int i = 0; i < frontiers.size(); ++i) {
        uf.add(i);
        boxMap[i] = frontiers[i];
    }

    // Check adjacency and union sets
    for (int i = 0; i < frontiers.size(); ++i) {
        auto boxI = frontiers[i].first;
        for (int j = i + 1; j < frontiers.size(); ++j) {
            auto boxJ = frontiers[j].first;
            Coordinate centerI = boxI.getCenter();
            Coordinate centerJ = boxJ.getCenter();
            double distance = sqrt(pow(centerI.x - centerJ.x, 2) + pow(centerI.y - centerJ.y, 2));
            double threshold = sqrt(2 * pow(boxI.getSize() * 0.5 + boxJ.getSize() * 0.5, 2));
            if (distance <= threshold) {
                uf.unionSets(i, j);
            }
        }
    }

    // Group boxes by their root set representative
    std::unordered_map<int, std::vector<std::pair<quadtree::Box, double>>> regions;
    for (int i = 0; i < frontiers.size(); ++i) {
        int root = uf.find(i);
        regions[root].push_back(frontiers[i]);
    }

    // Convert to the required format
    for (const auto &region: regions) {
        frontierRegions.push_back(region.second);
    }

    truncateMaxRegions(frontierRegions, max_regions);


}



/**
 * Calculate the pheromone frontier fitness value.
 * Based on the gaussian curve, where each tail corresponds to the occupied and free thresholds.
 * @param agent
 * @param pheromone
 * @return
 */
double getValueOnPheromoneCurve(Agent* agent, double pheromone){
//    if (pheromone < 0.5) {
//        return -sqrt(0.5-pheromone) * (0.5-agent->config.P_OCCUPIED_THRESHOLD);
//    } else if (pheromone > 0.5) {
//        return -sqrt(pheromone-0.5) * (agent->config.P_FREE_THRESHOLD-0.5);
//    } else {
//        auto average_certainty_threshold = ((agent->config.P_FREE_THRESHOLD-0.5) + (0.5-agent->config.P_OCCUPIED_THRESHOLD))/2;
//        return sqrt(average_certainty_threshold) * average_certainty_threshold;
//    }
    auto s = 0.0f;
    if (pheromone < 0.5) {
        auto xl = 0.5f-agent->config.P_OCCUPIED_THRESHOLD;
        s = (0-xl)/2;
    } else if (pheromone >= 0.5) {
        auto xr = agent->config.P_FREE_THRESHOLD-0.5f;
        s = (0-xr)/2;
    }
    auto val = exp(-pow((pheromone-0.5), 2)/(2*pow(s, 2)));
    return val;

}

/**
 * Calculate the pheromone frontier fitness value.
 * Based on the gaussian curve, where each tail corresponds to the occupied and free thresholds.
 * @param agent
 * @param pheromone
 * @return
 */
double calculateRegionVisitProbability(Agent* agent, std::map<Coordinate, std::tuple<std::vector<std::pair<quadtree::Box, double>>, double, double, double, double>>& region_summed_certainties_and_reach_time_battery, Coordinate coordinate, double denominator){
    auto region = std::get<0>(region_summed_certainties_and_reach_time_battery[coordinate]);
    auto region_repulsion = std::get<1>(region_summed_certainties_and_reach_time_battery[coordinate]);
    auto distance = std::get<2>(region_summed_certainties_and_reach_time_battery[coordinate]);

    #ifdef BATTERY_MANAGEMENT_ENABLED
    auto reach_time = std::get<3>(region_summed_certainties_and_reach_time_battery[coordinate]);
    auto reach_battery = std::get<4>(region_summed_certainties_and_reach_time_battery[coordinate]);
    auto numerator = std::pow(agent->config.FRONTIER_PHEROMONE_K + region_repulsion, -agent->config.FRONTIER_PHEROMONE_N) * std::pow(region.size(), agent->config.FRONTIER_PHEROMONE_L) /
                     std::pow(agent->config.FRONTIER_DISTANCE_WEIGHT * distance
                                + agent->config.FRONTIER_REACH_DURATION_WEIGHT * reach_time
                                + agent->config.FRONTIER_REACH_BATTERY_WEIGHT*reach_battery
                     , agent->config.FRONTIER_PHEROMONE_M);

//    argos::LOG << "[" << agent->id << "] " << "Numerator: (" << agent->config.FRONTIER_PHEROMONE_K + region_repulsion << ")^-" << agent->config.FRONTIER_PHEROMONE_N << " / (" << agent->config.FRONTIER_REACH_DURATION_WEIGHT * reach_time + agent->config.FRONTIER_REACH_BATTERY_WEIGHT*reach_battery << ")^-"
//               << agent->config.FRONTIER_PHEROMONE_M << " = " << std::pow(agent->config.FRONTIER_PHEROMONE_K + region_repulsion, -agent->config.FRONTIER_PHEROMONE_N) << " / " << std::pow(agent->config.FRONTIER_DISTANCE_WEIGHT * distance + agent->config.FRONTIER_REACH_DURATION_WEIGHT * reach_time + agent->config.FRONTIER_REACH_BATTERY_WEIGHT*reach_battery, agent->config.FRONTIER_PHEROMONE_M)
//               << "= " << numerator << std::endl;
    #else
    auto numerator = std::pow(agent->config.FRONTIER_PHEROMONE_K + region_repulsion, -agent->config.FRONTIER_PHEROMONE_N) * std::pow(region.size(), agent->config.FRONTIER_PHEROMONE_L) /
            std::pow(agent->config.FRONTIER_DISTANCE_WEIGHT * distance, agent->config.FRONTIER_PHEROMONE_M);
    #endif
    //if numerator is infinite, return 1
    if (numerator == INFINITY){
//        argos::LOG << "[" << agent->id << "] " << "Probability (infinity): 1" << std::endl;
        return 1;
    }
    return numerator/denominator; //Probability of visiting this region, based on the summed certainties

}

/**
 * Calculate the pheromone frontier fitness value.
 * Based on the gaussian curve, where each tail corresponds to the occupied and free thresholds.
 * @param agent
 * @param pheromone
 * @return
 */
double calculateRegionVisitProbabilityDenominator(Agent* agent, const std::map<Coordinate, std::tuple<std::vector<std::pair<quadtree::Box, double>>, double, double, double, double>>& region_summed_certainties_and_reach_time_battery){

    double denominator = 0.0;
    for (auto &it : region_summed_certainties_and_reach_time_battery) {
        auto region = std::get<0>(it.second);
        auto region_repulsion = std::get<1>(it.second);
        auto distance = std::get<2>(it.second);
        #ifdef BATTERY_MANAGEMENT_ENABLED
        auto reach_time = std::get<3>(it.second);
        auto reach_battery = std::get<4>(it.second);
        denominator += std::pow(agent->config.FRONTIER_PHEROMONE_K + region_repulsion, -agent->config.FRONTIER_PHEROMONE_N) * std::pow(region.size(), agent->config.FRONTIER_PHEROMONE_L) /
                       std::pow(agent->config.FRONTIER_DISTANCE_WEIGHT * distance
                                + agent->config.FRONTIER_REACH_DURATION_WEIGHT * reach_time
                                + agent->config.FRONTIER_REACH_BATTERY_WEIGHT*reach_battery
                               , agent->config.FRONTIER_PHEROMONE_M);
//        argos::LOG << "[" << agent->id << "] " << "Numerator: (" << agent->config.FRONTIER_PHEROMONE_K + region_repulsion << ")^-" << agent->config.FRONTIER_PHEROMONE_N << " / (" << agent->config.FRONTIER_REACH_DURATION_WEIGHT * reach_time + agent->config.FRONTIER_REACH_BATTERY_WEIGHT*reach_battery << ")^-"
//                   << agent->config.FRONTIER_PHEROMONE_M << " = " << std::pow(agent->config.FRONTIER_PHEROMONE_K + region_repulsion, -agent->config.FRONTIER_PHEROMONE_N) << " / " << std::pow(agent->config.FRONTIER_DISTANCE_WEIGHT * distance + agent->config.FRONTIER_REACH_DURATION_WEIGHT * reach_time + agent->config.FRONTIER_REACH_BATTERY_WEIGHT*reach_battery, agent->config.FRONTIER_PHEROMONE_M)
//                   << "= " << std::pow(agent->config.FRONTIER_PHEROMONE_K + region_repulsion, -agent->config.FRONTIER_PHEROMONE_N) /
//                              std::pow(agent->config.FRONTIER_DISTANCE_WEIGHT * distance
//                                       + agent->config.FRONTIER_REACH_DURATION_WEIGHT * reach_time
//                                       + agent->config.FRONTIER_REACH_BATTERY_WEIGHT*reach_battery
//                                      , agent->config.FRONTIER_PHEROMONE_M) << std::endl;
        #else
        denominator += std::pow(agent->config.FRONTIER_PHEROMONE_K + region_repulsion, -agent->config.FRONTIER_PHEROMONE_N) * std::pow(region.size(), agent->config.FRONTIER_PHEROMONE_L) /
                std::pow(agent->config.FRONTIER_DISTANCE_WEIGHT * distance, agent->config.FRONTIER_PHEROMONE_M);
        #endif
    }
    return denominator; //Probability of visiting this region, based on the summed certainties

}

argos::CVector2 ForceVectorCalculator::calculateUnexploredFrontierVector(Agent* agent) {
    //According to Dynamic frontier-led swarming:
    //https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
    //F* = arg min (Ψ_D||p-G^f||_2 - Ψ_S * J^f) for all frontiers F^f in F
    //F^f is a frontier, a segment that separates explored cells from unexplored cells.

    //Where Ψ_D is the frontier distance weight
    //p is the agent position
    //G^f is the frontier position defined as G^f = (Sum(F_j^f))/J^f So the sum of the cell locations divided by the amount of cells
    //Ψ_S is the frontier size weight
    //J^f is the number of cells in the frontier F^f

    //A cell is a frontier iff:
    //1. Occupancy = explored
    //2. At least one neighbor is unexplored using the 8-connected Moore neighbours. (https://en.wikipedia.org/wiki/Moore_neighborhood)

    //TODO: Need to keep search area small for computation times. Maybe when in range only low scores, expand range or search a box besides.
    std::vector<std::pair<quadtree::Box, double>> frontiers = agent->quadtree->queryFrontierBoxes(agent->position, agent->config.FRONTIER_SEARCH_RADIUS,
                                                                              agent->elapsed_ticks /
                                                                              agent->ticks_per_second, agent->config.FRONTIER_CELL_RATIO);
    agent->current_frontiers = frontiers;

    // Initialize an empty vector of vectors to store frontier regions
    std::vector<std::vector<std::pair<quadtree::Box, double>>> frontierRegions = {};

//    mergeAdjacentFrontiers(frontiers, frontierRegions, agent->config.MAX_FRONTIER_REGIONS);


//// Iterate over each frontier box to merge adjacent ones into regions
    for (auto [frontierbox, frontierpheromone]: frontiers) {
        bool added = false; // Flag to check if the current frontier has been added to a region

        // Iterate over existing regions to find a suitable one for the current frontier
        for (auto &region: frontierRegions) {
            for (auto [box, pheromone]: region) {
                // Calculate the center coordinates of the current box and the frontier
                Coordinate boxCenter = box.getCenter();
                Coordinate frontierCenter = frontierbox.getCenter();

                // Check if the distance between the box and the frontier is less than or equal to
                // the average diagonal of the two boxes (ensuring adjacency)
                if (sqrt(pow(boxCenter.x - frontierCenter.x, 2) + pow(boxCenter.y - frontierCenter.y, 2)) <=
                    sqrt(2 * pow(frontierbox.getSize() * 0.5 + box.getSize() * 0.5, 2))) {
                    region.emplace_back(frontierbox, frontierpheromone); // Add the frontier to the current region
                    added = true; // Mark the frontier as added
                    break; // Exit the loop since the frontier has been added to a region
                }
            }
            if (added) break; // Exit the loop since the frontier has been added to a region
        }

        // If the frontier was not added to any existing region, create a new region with it
        if (!added) {
            frontierRegions.push_back({{frontierbox, frontierpheromone}});
        }
    }
    truncateMaxRegions(frontierRegions, agent->config.MAX_FRONTIER_REGIONS);


    //Add the frontier region from the last best frontier to the options, if it is now out of range. To prevent unneccesary switching
    //If we haven't reached the frontier yet, add the region to the list of frontier regions
    if (!(agent->currentBestFrontier == Coordinate{MAXFLOAT, MAXFLOAT}) && (sqrt(pow(agent->currentBestFrontier.x - agent->position.x, 2) + pow(agent->currentBestFrontier.y - agent->position.y, 2)) > agent->config.FRONTIER_SEARCH_RADIUS)) {
        if (!agent->bestFrontierRegionBoxes.empty()) frontierRegions.push_back(agent->bestFrontierRegionBoxes);
//        argos::LOG << "[" << agent->id << "] Found " << "Adding previous best frontier region to the list of frontier regions" << std::endl;
    }

    agent->current_frontier_regions = frontierRegions;
//    argos::LOG << "[" << agent->id << "] Found " << frontierRegions.size() << " frontier regions" << std::endl;

    //Now we have all frontier cells merged into frontier regions
    //Find F* by using the formula above
    //Ψ_D = FRONTIER_DISTANCE_WEIGHT
    //Ψ_S = FRONTIER_SIZE_WEIGHT

    //Initialize variables to store the best frontier region and its score
    std::vector<quadtree::Box> bestFrontierRegion = {};
    Coordinate bestFrontierRegionCenter = {MAXFLOAT, MAXFLOAT};
//    double highestFrontierFitness = -std::numeric_limits<double>::max();
    std::vector<std::pair<Coordinate, Coordinate>> bestRoute = {};
#ifdef PATH_PLANNING_ENABLED
    int bestFrontierRouteWallFollowingDirection;
#endif
//                    `                           region,                       s certainties, distance, reach time, reach battery
    std::map<Coordinate, std::tuple<std::vector<std::pair<quadtree::Box, double>>, double, double, double, double>> region_summed_certainties_and_reach_dist_time_battery;
    std::map<Coordinate, std::pair<std::vector<std::pair<Coordinate, Coordinate>>, int>> region_routes; //Route, wall following direction

    for (const auto &region: frontierRegions) {
        double sumX = 0;
        double sumY = 0;
        double totalNumberOfCellsInRegion = 0;
        int nUnknown = 0;
        int nAmbiguous = 0;
//        double averagePheromoneCertainty = 0;
        double pheromoneCurve = 0;
        double region_repulsion = 0;
        for (auto [box, pheromone]: region) {
            double cellsInBox = box.getSize() / agent->quadtree->getResolution();
            assert(cellsInBox == 1);
            sumX += box.getCenter().x *
                    cellsInBox; //Take the box size into account (parent nodes will contain the info about all its children)
            sumY += box.getCenter().y * cellsInBox;
            totalNumberOfCellsInRegion += cellsInBox;
//            averagePheromoneCertainty += std::abs(pheromone - 0.5);
//            pheromoneCurve += getValueOnPheromoneCurve(agent, pheromone);
            if (pheromone == 0.5) {
                nUnknown++;
            } else {
                nAmbiguous++;
            }
//            argos::LOG << "[" << agent->id << "] " << region_repulsion << " += (" << agent->config.FRONTIER_PHEROMONE_K << " + " << std::abs(pheromone - 0.5) << ")^-" << agent->config.FRONTIER_PHEROMONE_N << " = " << std::pow(agent->config.FRONTIER_PHEROMONE_K + std::abs(pheromone - 0.5), -agent->config.FRONTIER_PHEROMONE_N) << std::endl;
            region_repulsion += std::abs(pheromone - 0.5);


        }
        double frontierRegionX = sumX / totalNumberOfCellsInRegion;
        double frontierRegionY = sumY / totalNumberOfCellsInRegion;
//        argos::LOG << "[" << agent->id << "] " << "Frontier region " << frontierRegionX << ", " << frontierRegionY << " has repulsion " << region_repulsion << " / " << totalNumberOfCellsInRegion << " = " << region_repulsion / totalNumberOfCellsInRegion << std::endl;
        region_repulsion /= totalNumberOfCellsInRegion; //Average

//    }
//
//
//
//

        //Iterate over all frontier regions to find the best one
//    for (const auto &region_and_summed_certainty: region_summed_certainties) {
        //Calculate the average position of the frontier region_and_summed_certainty
//        auto frontierRegionX = region_and_summed_certainty.first.x;
//        auto frontierRegionY = region_and_summed_certainty.first.y;
//        auto region = region_and_summed_certainty.second.first;

//        auto region_visit_probability = calculateRegionVisitProbability(agent, region_summed_certainties, {frontierRegionX, frontierRegionY}, region_visit_probability_denominator);

//        pheromoneCurve /= totalNumberOfCellsInRegion;

//        argos::LOG << "Frontier region_and_summed_certainty " << frontierRegionX << ", " << frontierRegionY << " with pheromonecurve " << pheromoneCurve << " has " << totalNumberOfCellsInRegion << " cells of which " << nUnknown << " unknown and " << nAmbiguous << " ambiguous" << std::endl;
//        argos::LOG << "Average certainty: " << averagePheromoneCertainty << std::endl;

//        //Skip too small areas
//        if (totalNumberOfCellsInRegion <= 5){
//            skipFrontier = true;
//            argos::LOG << "Skipping frontier region_and_summed_certainty " << frontierRegionX << ", " << frontierRegionY
//                       << " because it is too small" << std::endl;
//        }
//        argos::LOG << "Checking frontier region " << frontierRegionX << ", " << frontierRegionY << std::endl;

        //If the frontier is mostly unknown, and if the frontier is close to our agent, skip it
//        if (sqrt(pow(frontierRegionX - agent->position.x, 2) + pow(frontierRegionY - agent->position.y, 2)) <
//            agent->config.FRONTIER_DIST_UNTIL_REACHED) {
//            skipFrontier = true;
////            argos::LOG << "Skipping frontier region " << frontierRegionX << ", " << frontierRegionY
////                       << " because it is close to our agent" << std::endl;
//        } else {
#if defined(SEPARATE_FRONTIERS) || defined(SKIP_UNREACHABLE_FRONTIERS)
        bool skipFrontier = false;
            for (auto agentLocationTuple: agent->agentLocations) {
#ifdef SEPARATE_FRONTIERS
                if ((std::get<2>(agentLocationTuple.second) - agent->elapsed_ticks) / agent->ticks_per_second >
                    agent->config.AGENT_LOCATION_RELEVANT_S)
                    continue;

                double distanceFromOtherAgentsFrontier = sqrt(
                        pow(frontierRegionX - std::get<1>(agentLocationTuple.second).x, 2) +
                        pow(frontierRegionY - std::get<1>(agentLocationTuple.second).y, 2));

                double distanceFromOtherAgentsPosition = sqrt(
                        pow(frontierRegionX - std::get<0>(agentLocationTuple.second).x, 2) +
                        pow(frontierRegionY - std::get<0>(agentLocationTuple.second).y, 2));

                if (distanceFromOtherAgentsFrontier <= agent->config.FRONTIER_SEPARATION_THRESHOLD) {
                    //If the frontier is close to another agent's frontier, the agent with the highest ID has priority
//                    if (agentLocationTuple.first > agent->id) {
                    //If the other agent has a higher ID, so we can't select this frontier
                    skipFrontier = true;
//                    argos::LOG << "Skipping frontier region " << frontierRegionX << ", " << frontierRegionY
//                               << " because it is close to another agent's frontier" << std::endl;
//                    argos::LOG << "Distance from other agent's frontier: " << distanceFromOtherAgentsFrontier << std::endl;
                    break;
//                    }
                }
                if (distanceFromOtherAgentsPosition <= agent->config.FRONTIER_DIST_UNTIL_REACHED) {
                    //If the frontier is close to another agent's frontier, the agent with the highest ID has priority
//                    if (agentLocationTuple.first > agent->id) {
                    //If the other agent has a higher ID, so we can't select this frontier
                    skipFrontier = true;
//                    argos::LOG << "Skipping frontier region " << frontierRegionX << ", " << frontierRegionY
//                               << " because it is close to another agent's position" << std::endl;
//                    argos::LOG << "Distance from other agent's position: " << distanceFromOtherAgentsPosition
//                               << std::endl;
                    break;
                }
//                }
//            }
#endif
#ifdef SKIP_UNREACHABLE_FRONTIERS
            //If we are avoiding the frontier
            if (agent->frontierEvaluator.avoidingCoordinate(agent, {frontierRegionX, frontierRegionY})) {
                skipFrontier = true;
//                argos::LOG << "Skipping frontier region " << frontierRegionX << ", " << frontierRegionY
//                           << " because we are avoiding it" << std::endl;
            }
#endif
        }
        if (skipFrontier) continue;
#endif


        argos::CVector2 vectorToFrontier = argos::CVector2(frontierRegionX - agent->position.x,
                                                           frontierRegionY - agent->position.y).Rotate(-agent->heading);
        std::vector<std::pair<Coordinate, Coordinate>> route_to_frontier = {
                {agent->position, Coordinate{frontierRegionX, frontierRegionY}}};

        //Calculate the distance between the agent and the frontier region_and_summed_certainty
#ifdef PATH_PLANNING_ENABLED
        //Calculate distance of route
//        double distance = 0;
        route_to_frontier.clear();
        auto [wall_following_direction, relative_route, distance] = agent->pathPlanner.getRoute(agent, agent->position,
                                                                                                {frontierRegionX,
                                                                                                 frontierRegionY},
                                                                                                route_to_frontier);
        if (route_to_frontier.empty()) continue; //If no route is found, skip this frontier
//        for (auto edge: relative_route) {
//            distance += edge.Length();
//        }
#else
        double distance = sqrt(pow(frontierRegionX - agent->position.x, 2) + pow(frontierRegionY - agent->position.y, 2));
        std::vector<argos::CVector2> relative_route = {vectorToFrontier.Rotate(-agent->heading)};
        int wall_following_direction = 1; //Doesn't matter
#endif
        //Relative vector to heading

#ifdef BATTERY_MANAGEMENT_ENABLED
        //Only need to compare the motion power usage of the agent to the frontier region_and_summed_certainty
        auto [powerUsage, duration] = agent->batteryManager.estimateMotionPowerUsage(agent, relative_route);
        region_summed_certainties_and_reach_dist_time_battery[{frontierRegionX, frontierRegionY}] = std::make_tuple(region, region_repulsion, distance, duration, powerUsage);
#else
        region_summed_certainties_and_reach_dist_time_battery[{frontierRegionX, frontierRegionY}] = std::make_tuple(region, region_repulsion, distance, 0, 0);
#endif
        region_routes[{frontierRegionX, frontierRegionY}] = {route_to_frontier, wall_following_direction};
    }
    double region_visit_probability_denominator = calculateRegionVisitProbabilityDenominator(agent, region_summed_certainties_and_reach_dist_time_battery);

    int random = rand() % 100;
    double cumulative_probability = 0;
//    argos::LOG << "[" << agent->id << "] " << "Considering " << region_summed_certainties_and_reach_dist_time_battery.size() << " frontier regions" << std::endl;
    for (const auto &region_and_summed_certainty: region_summed_certainties_and_reach_dist_time_battery) {
//    Calculate the average position of the frontier region_and_summed_certainty
        auto frontierRegionX = region_and_summed_certainty.first.x;
        auto frontierRegionY = region_and_summed_certainty.first.y;
        auto region = std::get<0>(region_and_summed_certainty.second);

//#ifdef BATTERY_MANAGEMENT_ENABLED
//        argos::LOG << "[" << agent->id << "] " << "Frontier region_and_summed_certainty: " << frontierRegionX << ", " << frontierRegionY << " with repulsion " << std::get<1>(region_and_summed_certainty.second) << " has " << std::get<2>(region_and_summed_certainty.second) << " distance, " << std::get<3>(region_and_summed_certainty.second) << " duration and " << std::get<4>(region_and_summed_certainty.second) << " power usage" << std::endl;
        auto region_visit_probability = calculateRegionVisitProbability(agent, region_summed_certainties_and_reach_dist_time_battery, {frontierRegionX, frontierRegionY}, region_visit_probability_denominator);
//        argos::LOG << "[" << agent->id << "] " << "Frontier region_and_summed_certainty: " << frontierRegionX << ", " << frontierRegionY << " has " << region_visit_probability << " probability" << std::endl;

//        //Calculate the cost of the frontier region_and_summed_certainty
//        double cost =
//                agent->FRONTIER_DISTANCE_WEIGHT * distance - agent->FRONTIER_SIZE_WEIGHT * totalNumberOfCellsInRegion -
//                agent->FRONTIER_REACH_BATTERY_WEIGHT * powerUsage - agent->FRONTIER_REACH_DURATION_WEIGHT * duration -
//                agent->FRONTIER_PHEROMONE_WEIGHT * averagePheromoneCertainty;

        //Calculate the fitness of the frontier region_and_summed_certainty
//        double fitness =
//                -agent->config.FRONTIER_DISTANCE_WEIGHT * distance + agent->config.FRONTIER_SIZE_WEIGHT * totalNumberOfCellsInRegion -
//                agent->config.FRONTIER_REACH_BATTERY_WEIGHT * powerUsage - agent->config.FRONTIER_REACH_DURATION_WEIGHT * duration -
//                agent->config.FRONTIER_PHEROMONE_WEIGHT * averagePheromoneCertainty;

//        double fitness =
//                -agent->config.FRONTIER_DISTANCE_WEIGHT * distance + agent->config.FRONTIER_PHEROMONE_WEIGHT * pheromoneCurve
//                + agent->config.FRONTIER_SIZE_WEIGHT * totalNumberOfCellsInRegion
//                - agent->config.FRONTIER_REACH_BATTERY_WEIGHT * powerUsage - agent->config.FRONTIER_REACH_DURATION_WEIGHT * duration;
//        if (agent->id == "pipuck1") {
//            argos::LOG << "[" << agent->id << "] " << "Frontier region_and_summed_certainty: " << frontierRegionX << ", " << frontierRegionY << " Score: " << fitness
//                       << std::endl;
//            argos::LOG << "[" << agent->id << "] " << "Distance * weight = " << distance << " * " << -agent->config.FRONTIER_DISTANCE_WEIGHT << " = "
//                       << -distance * agent->config.FRONTIER_DISTANCE_WEIGHT << std::endl;
//            argos::LOG << "[" << agent->id << "] " << "Pheromone curve * weight = " << pheromoneCurve << " * " << agent->config.FRONTIER_PHEROMONE_WEIGHT
//                          << " = " << pheromoneCurve * agent->config.FRONTIER_PHEROMONE_WEIGHT << std::endl;
//            argos::LOG << "[" << agent->id << "] " << "Cell number * weight = " << totalNumberOfCellsInRegion << " * " << agent->config.FRONTIER_SIZE_WEIGHT
//                            << " = " << totalNumberOfCellsInRegion * agent->config.FRONTIER_SIZE_WEIGHT << std::endl;
//            argos::LOG << "[" << agent->id << "] " << "Power usage * weight = " << powerUsage << " * " << -agent->config.FRONTIER_REACH_BATTERY_WEIGHT
//                            << " = " << -powerUsage * agent->config.FRONTIER_REACH_BATTERY_WEIGHT << std::endl;
//            argos::LOG << "[" << agent->id << "] " << "Duration * weight = " << duration << " * " << -agent->config.FRONTIER_REACH_DURATION_WEIGHT
//                            << " = " << -duration * agent->config.FRONTIER_REACH_DURATION_WEIGHT << std::endl;
//        }
//        double fitness =
//                (1 - region_visit_probability) *
//                (-agent->config.FRONTIER_DISTANCE_WEIGHT * distance - agent->config.FRONTIER_REACH_BATTERY_WEIGHT * powerUsage - agent->config.FRONTIER_REACH_DURATION_WEIGHT * duration);

//        double fitness = region_visit_probability;

//        argos::LOG << "[" << agent->id << "] " << "Frontier region_and_summed_certainty: " << frontierRegionX << ", " << frontierRegionY << " Score: " << fitness
//                   << std::endl;
//        argos::LOG << "[" << agent->id << "] " << "(1 - probability) = " << "(1 - " << region_visit_probability << ") = " << 1 - region_visit_probability << std::endl;
//        argos::LOG << "[" << agent->id << "] " << "Distance * weight = " << distance << " * " << -agent->config.FRONTIER_DISTANCE_WEIGHT << " = "
//                   << -distance * agent->config.FRONTIER_DISTANCE_WEIGHT << std::endl;
//        argos::LOG << "[" << agent->id << "] " << "Power usage * weight = " << powerUsage << " * " << -agent->config.FRONTIER_REACH_BATTERY_WEIGHT
//                     << " = " << -powerUsage * agent->config.FRONTIER_REACH_BATTERY_WEIGHT << std::endl;
//        argos::LOG << "[" << agent->id << "] " << "Duration * weight = " << duration << " * " << -agent->config.FRONTIER_REACH_DURATION_WEIGHT
//                        << " = " << -duration * agent->config.FRONTIER_REACH_DURATION_WEIGHT << std::endl;


//#else
        //Calculate the cost of the frontier region_and_summed_certainty
//        double fitness = -agent->config.FRONTIER_DISTANCE_WEIGHT * distance + agent->config.FRONTIER_SIZE_WEIGHT * totalNumberOfCellsInRegion -
//                         agent->config.FRONTIER_PHEROMONE_WEIGHT * pheromoneCurve;
//#endif



        //If the cost is lower than the best cost, update the best cost and best frontier region_and_summed_certainty
//        if (fitness > highestFrontierFitness) {
//            highestFrontierFitness = fitness;
//            bestFrontierRegionCenter = {frontierRegionX, frontierRegionY};
//            agent->bestFrontierRegionBoxes = region;
//            bestRoute = route_to_frontier;
//#ifdef PATH_PLANNING_ENABLED
//            bestFrontierRouteWallFollowingDirection = wall_following_direction;
//#endif
//        }
        cumulative_probability += region_visit_probability;
//        argos::LOG << "Random: " << random << " < " << cumulative_probability * 100 << std::endl;
        if (random < cumulative_probability * 100) { //Chance of choosing this region
//            argos::LOG << "Selected frontier because of probability" << std::endl;
//            highestFrontierFitness = fitness;
            bestFrontierRegionCenter = {frontierRegionX, frontierRegionY};
            agent->bestFrontierRegionBoxes = region;
            bestRoute = region_routes[{frontierRegionX, frontierRegionY}].first;
#ifdef PATH_PLANNING_ENABLED
            bestFrontierRouteWallFollowingDirection = region_routes[{frontierRegionX, frontierRegionY}].second;
#endif
            break;
        }
//        else {
//            argos::LOG << "Skipped frontier because " << random << " > " << cumulative_probability * 100 << std::endl;
//        }
    }


    agent->currentBestFrontier = bestFrontierRegionCenter;
    agent->route_to_best_frontier = bestRoute;
#ifdef PATH_PLANNING_ENABLED
    agent->pathPlanner.setTarget(bestFrontierRegionCenter);
    agent->pathPlanner.setWallFollowingDirection(bestFrontierRouteWallFollowingDirection);
#endif

//    argos::LOG << "selected frontier region " << bestFrontierRegionCenter.x << ", " << bestFrontierRegionCenter.y << std::endl;


    //Calculate the vector to the best frontier region
    argos::CVector2 vectorToBestFrontier = argos::CVector2(bestFrontierRegionCenter.x, bestFrontierRegionCenter.y)
                                           - argos::CVector2(agent->position.x, agent->position.y);

    return vectorToBestFrontier;
}

/**
 * Calculate the vector to avoid objects:
 * Find the closest relative angle that is free of objects within a certain range.
 * According to "Dynamic Frontier-Led Swarming: Multi-Robot Repeated Coverage in Dynamic Environments" paper
 * https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=10057179&tag=1
 * @return The vector towards the free direction
 */

bool ForceVectorCalculator::calculateObjectAvoidanceAngle(Agent* agent, argos::CRadians *relativeObjectAvoidanceAngle, ForceVectorCalculator::vectors vectors, argos::CVector2 & total_vector, bool frontier_vector_zero) {

    total_vector = ForceVectorCalculator::calculateTotalVector(agent, vectors);

    argos::CRadians targetAngle = total_vector.Angle();

    //Get occupied boxes within range
    std::vector<quadtree::Box> occupiedBoxes = agent->quadtree->queryOccupiedBoxes(agent->position,
                                                                                   agent->config.OBJECT_AVOIDANCE_RADIUS * 2.0,
                                                                                  agent->elapsed_ticks /
                                                                                  agent->ticks_per_second);

    double angleInterval = argos::CDegrees(360 / agent->config.STEPS_360_DEGREES).GetValue();

    //Create set of free angles ordered to be used for wall following
//    std::set<argos::CDegrees, CustomComparator> freeAngles(
//            CustomComparator(agent->wallFollower.wallFollowingDirection, ToDegrees(agent->heading).GetValue(), ToDegrees(targetAngle).GetValue()));

    std::set<argos::CDegrees> freeAngles;

//Add free angles from -180 to 180 degrees
    for (int a = 0; a < agent->config.STEPS_360_DEGREES; a++) {
        auto angle = argos::CDegrees(a * 360 / agent->config.STEPS_360_DEGREES - 180);
        freeAngles.insert(angle);
    }


    //For each occupied box, find the angles that are blocked relative to the agent
    for (auto box: occupiedBoxes) {


        argos::CVector2 OC = argos::CVector2(box.getCenter().x - agent->position.x,
                                             box.getCenter().y - agent->position.y);
        if (OC.Length() > agent->config.OBJECT_AVOIDANCE_RADIUS) continue;

        argos::CRadians Bq = argos::ASin(
                std::min(agent->config.AGENT_SAFETY_RADIUS + agent->config.OBJECT_SAFETY_RADIUS, OC.Length()) / OC.Length());
        argos::CRadians Eta_q = OC.Angle();
//        if (agent->config.AGENT_SAFETY_RADIUS + agent->config.OBJECT_SAFETY_RADIUS > OC.Length())
//            argos::LOGERR << "AGENT_SAFETY_RADIUS + OBJECT_SAFETY_RADIOS > OC.Length(): " << agent->config.AGENT_SAFETY_RADIUS
//                           << " + " << agent->config.OBJECT_SAFETY_RADIUS << ">" << OC.Length() << std::endl;

        argos::CDegrees minAngle = ToDegrees((Eta_q - Bq).SignedNormalize());
        argos::CDegrees maxAngle = ToDegrees((Eta_q + Bq).SignedNormalize());

        if (maxAngle < minAngle) {
            argos::CDegrees temp = minAngle;
            minAngle = maxAngle;
            maxAngle = temp;
        }


        //Round to multiples of 360/config.STEPS_360_DEGREES
        double minAngleVal = minAngle.GetValue();
        double intervalDirectionMin = (minAngleVal < 0) ? -angleInterval : angleInterval;
//
        auto minRoundedAngle1 = (int) (minAngleVal / angleInterval) * angleInterval;
        auto minRoundedAngle2 = minRoundedAngle1 + intervalDirectionMin;

        auto roundedMinAngle = argos::CDegrees(
                (abs(minAngleVal - minRoundedAngle1) >= abs(minRoundedAngle2 - minAngleVal)) ? minRoundedAngle2
                                                                                             : minRoundedAngle1);

        double maxAngleVal = maxAngle.GetValue();
        double intervalDirectionMax = (maxAngleVal < 0) ? -angleInterval : angleInterval;

        auto maxRoundedAngle1 = (int) (maxAngleVal / angleInterval) * angleInterval;
        auto maxRoundedAngle2 = maxRoundedAngle1 + intervalDirectionMax;

        auto roundedMaxAngle = argos::CDegrees(
                (abs(maxAngleVal - maxRoundedAngle1) >= abs(maxRoundedAngle2 - maxAngleVal)) ? maxRoundedAngle2
                                                                                             : maxRoundedAngle1);

        //Block all angles within range
        for (int a = 0; a < agent->config.STEPS_360_DEGREES; a++) {
            auto angle = argos::CDegrees(a * 360 / agent->config.STEPS_360_DEGREES - 180);

            if (NormalizedDifference(ToRadians(roundedMinAngle), Eta_q) <= argos::CRadians(0) &&
                NormalizedDifference(ToRadians(roundedMaxAngle), Eta_q) >= argos::CRadians(0)) {
                if (angle >= roundedMinAngle && angle <= roundedMaxAngle) {
                    freeAngles.erase(angle);
                }

            } else if (NormalizedDifference(ToRadians(roundedMinAngle), Eta_q) >= argos::CRadians(0) &&
                       NormalizedDifference(ToRadians(roundedMaxAngle), Eta_q) <= argos::CRadians(0)) {

                if (angle <= roundedMinAngle || angle >= roundedMaxAngle) {
                    freeAngles.erase(angle);
                }
            } 
            // else {
            //     argos::LOGERR << "Error: Eta_q not within range" << std::endl;
            // }
        }

    }

    //Block angles according to the distance sensors
    for (int i = 0; i < agent->distance_sensors.size(); i++) {
        //Only handle the forward sensor
        if (i != 0) continue;
        argos::CRadians sensor_rotation = agent->heading - i * argos::CRadians::PI_OVER_TWO;
        if (agent->distance_sensors[i].getDistance() < agent->config.OBJECT_AVOIDANCE_RADIUS/2) {
            //Erase all angles 90 degrees to the left and right of the sensor
            //12.5% to the left
            argos::CDegrees minAngle = argos::CDegrees(int(ToDegrees(sensor_rotation - argos::CRadians::PI_OVER_FOUR).GetValue())).SignedNormalize();
            argos::CDegrees maxAngle = argos::CDegrees(int(ToDegrees(sensor_rotation + argos::CRadians::PI_OVER_FOUR).GetValue())).SignedNormalize();

            if (maxAngle.GetValue() < minAngle.GetValue()) {
                argos::CDegrees temp = minAngle;
                minAngle = maxAngle;
                maxAngle = temp;
            }

            auto diffMinSensor = NormalizedDifference(ToRadians(minAngle), sensor_rotation);
            auto diffMaxSensor = NormalizedDifference(ToRadians(maxAngle), sensor_rotation);


            if (diffMinSensor >= argos::CRadians(0) && diffMaxSensor <= argos::CRadians(0)) {
                for (int a = 0; a < 90; a++) {
                    auto angle = (minAngle - argos::CDegrees(a)).SignedNormalize();
                    freeAngles.erase(argos::CDegrees(angle));
                }
            } else if (diffMinSensor <= argos::CRadians(0) && diffMaxSensor >= argos::CRadians(0)) {
                for (int a = 0; a < 90; a++) {
                    auto angle = (minAngle + argos::CDegrees(a)).SignedNormalize();
                    freeAngles.erase(argos::CDegrees(angle));
                }
            } else {
                assert(0);
            }

//            if (minAngle.GetValue() < 0 && maxAngle.GetValue() >= 0) {
//                for (int a = 0; a < 90; a++) {
//                    auto angle = (minAngle - argos::CDegrees(a)).SignedNormalize();
//                    freeAngles.erase(argos::CDegrees(angle));
//                }
//            } else if (minAngle.GetValue() >= 0 && maxAngle.GetValue() >= 0) {
//                for (int a = 0; a < 90; a++) {
//                    auto angle = (maxAngle + argos::CDegrees(a)).SignedNormalize();
//                    freeAngles.erase(argos::CDegrees(angle));
//                }
//            } else if (minAngle.GetValue() < 0 && maxAngle.GetValue() < 0) {
//                for (int a = 0; a < 90; a++) {
//                    auto angle = (minAngle + argos::CDegrees(a)).SignedNormalize();
//                    freeAngles.erase(argos::CDegrees(angle));
//                }
//            } else {
//                for (int a = 0; a < 90; a++) {
//                    auto angle = (maxAngle + argos::CDegrees(a)).SignedNormalize();
//                    freeAngles.erase(argos::CDegrees(angle));
//                }
//            }
//            }
//            //Erase all in between from freeAngles
//            for (int a = int(minAngle.GetValue()); a < int(maxAngle.GetValue()); a++){
//                freeAngles.erase(argos::CDegrees(a));
//            }
        }
    }


    //If there are no free angles, see if there are any sensors that have no close intersection.
    if (freeAngles.empty()) {
        for (int i = 0; i < agent->distance_sensors.size(); i++) {
            argos::CRadians sensor_rotation = agent->heading - i * argos::CRadians::PI_OVER_TWO;
            if (agent->distance_sensors[i].getDistance() > agent->config.OBJECT_AVOIDANCE_RADIUS) {
                argos::CDegrees minAngle = argos::CDegrees(
                        int(ToDegrees(sensor_rotation - argos::CRadians::PI / 18.0).GetValue())).SignedNormalize();
                argos::CDegrees maxAngle = argos::CDegrees(int(ToDegrees(
                        sensor_rotation + argos::CRadians::PI_OVER_SIX / 18.0).GetValue())).SignedNormalize();

                if (maxAngle.GetValue() < minAngle.GetValue()) {
                    argos::CDegrees temp = minAngle;
                    minAngle = maxAngle;
                    maxAngle = temp;
                }

                auto diffMinSensor = NormalizedDifference(ToRadians(minAngle), sensor_rotation);
                auto diffMaxSensor = NormalizedDifference(ToRadians(maxAngle), sensor_rotation);


                if (diffMinSensor >= argos::CRadians(0) && diffMaxSensor <= argos::CRadians(0)) {
                    for (int a = 0; a < 10; a++) {
                        auto angle = (minAngle - argos::CDegrees(a)).SignedNormalize();
                        freeAngles.insert(argos::CDegrees(angle));
                    }
                } else if (diffMinSensor <= argos::CRadians(0) && diffMaxSensor >= argos::CRadians(0)) {
                    for (int a = 0; a < 10; a++) {
                        auto angle = (minAngle + argos::CDegrees(a)).SignedNormalize();
                        freeAngles.insert(argos::CDegrees(angle));
                    }
                } else {
                    assert(0);
                }
            }
        }
    }
    agent->freeAnglesVisualization.clear();
    auto closestFreeAngle = *freeAngles.begin();
    CustomComparator customComparator(0, ToDegrees(agent->heading).GetValue(), ToDegrees(targetAngle).GetValue());
    for (auto freeAngle: freeAngles) {
        agent->freeAnglesVisualization.insert(freeAngle);
        if (customComparator(freeAngle, closestFreeAngle)) {
            closestFreeAngle = freeAngle;
        }
    }
    //If we still have no free angles, return false
    if (freeAngles.empty()) return false;


    argos::CRadians closestFreeAngleRadians = ToRadians(closestFreeAngle);
    *relativeObjectAvoidanceAngle = NormalizedDifference(closestFreeAngleRadians, targetAngle);


//    if (frontier_vector_zero) return true; //If the frontier vector is zero, we must follow the force vector, so can't do wall/path following
#ifdef WALL_FOLLOWING_ENABLED//If wall following is enabled
    agent->wallFollower.wallFollowing(agent, vectors, total_vector, freeAngles, &closestFreeAngle, &closestFreeAngleRadians, relativeObjectAvoidanceAngle, targetAngle);
#endif
    return true;

}

/**
 * Check if we need to avoid an agent, and adjust the target vector accordingly
 * Then normalize all agents
 * @param agent
 * @param vectors
 */
void ForceVectorCalculator::checkAvoidAndNormalizeVectors(ForceVectorCalculator::vectors &vectors) {
    //If there are agents to avoid, do not explore
    if (vectors.agentAvoidanceVector.Length() != 0) vectors.targetVector = {0, 0};

    //Normalize vectors if they are not zero
    if (vectors.virtualWallAvoidanceVector.Length() != 0) vectors.virtualWallAvoidanceVector.Normalize();
    if (vectors.agentCohesionVector.Length() != 0) vectors.agentCohesionVector.Normalize();
    if (vectors.agentAvoidanceVector.Length() != 0) vectors.agentAvoidanceVector.Normalize();
    if (vectors.agentAlignmentVector.Length() != 0) vectors.agentAlignmentVector.Normalize();
    if (vectors.targetVector.Length() != 0) vectors.targetVector.Normalize();

}
