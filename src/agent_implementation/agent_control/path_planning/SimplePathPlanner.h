#ifndef IMPLEMENTATION_AND_EXAMPLES_SIMPLEPATHPLANNER_H
#define IMPLEMENTATION_AND_EXAMPLES_SIMPLEPATHPLANNER_H

#include "../../feature_config.h"
#include <vector>
#include "../../utils/coordinate.h"
#include "../../utils/Quadtree.h"
// #include <argos3/core/utility/math/vector2.h>
#include "vector2.h"

class Agent;

class SimplePathPlanner {
public:
//    /**
//     * Get the route from start to end
//     * Each vector is relative to the former
//     * @param start
//     * @param end
//     * @param quadtree
//     * @return
//     */
    std::tuple<int, std::vector<argos::CVector2>, double> getRoute(Agent* agent, Coordinate start, Coordinate target, std::vector<std::pair<Coordinate, Coordinate>> & route) const;
    std::vector<argos::CVector2> coordinateRouteToRelativeVectors(const std::vector<std::pair<Coordinate, Coordinate>> & route, argos::CRadians agent_heading) const;
    void setWallFollowingDirection(int direction) {this->current_wall_following_direction = direction;}
    void setTarget(Coordinate target) {this->current_target = target;}
private:
    Coordinate current_target;
    int current_wall_following_direction;
    std::vector<std::pair<Coordinate, Coordinate>> getRouteSections(Agent* agent, Coordinate start, Coordinate target, int wall_following_direction, bool switched_direction, std::vector<std::pair<Coordinate, Coordinate>> & route) const;
    void getWallFollowingRoute(Agent* agent, quadtree::Quadtree::Cell * cell, double box_size, int edge_index, Coordinate target, std::vector<std::pair<Coordinate, Coordinate>> & route, int wall_following_direction,  bool switched_direction) const;
    std::tuple<int, quadtree::Quadtree::Cell *, int, Coordinate> directionToTargetFree(Agent* agent, quadtree::Quadtree::Cell * cell, double box_size, int edge_index, Coordinate target, int wall_following_direction, std::vector<std::pair<Coordinate, Coordinate>> & route) const;
    int directionToTargetFree(Agent* agent, Coordinate start, double box_size, Coordinate target, const std::vector<std::pair<Coordinate, Coordinate>> & route) const;

    static std::vector<std::pair<quadtree::Quadtree::Cell*, int>> elegible_edges(Agent* agent, quadtree::Quadtree::Cell * cell, int edge_index, double box_size) ;

    std::tuple<quadtree::Quadtree::Cell *, quadtree::Box, int, double> rayTraceQuadtreeOccupiedIntersection(Agent* agent, Coordinate start, Coordinate end) const;

    [[nodiscard]] Coordinate liang_barsky(Coordinate p1, Coordinate p2, quadtree::Box box) const;

    [[nodiscard]] std::pair<Coordinate, Coordinate> getEdgeCoordinates(quadtree::Box box, int edge_index) const;
};


#endif //IMPLEMENTATION_AND_EXAMPLES_SIMPLEPATHPLANNER_H

