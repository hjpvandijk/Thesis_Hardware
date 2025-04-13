
#include "SimplePathPlanner.h"
// #include "agent_implementation/utils/Box.h"
// #include "agent_implementation/utils/Algorithms.h"
// #include "agent_implementation/agent.h"
#include "../../utils/Box.h"
#include "../../utils/Algorithms.h"
#include "../../agent.h"

#ifdef PATH_PLANNING_ENABLED

std::tuple<int, std::vector<argos::CVector2>, double> SimplePathPlanner::getRoute(Agent* agent, Coordinate start, Coordinate target, std::vector<std::pair<Coordinate, Coordinate>> & route) const {
//    srand(target.x * 100 + target.y * 100);
    int wall_following_direction;
    std::vector<argos::CVector2> relativeRoute;
    double route_length = 0;
     if (agent->state != Agent::State::RETURNING && sqrt (pow(target.x - this->current_target.x, 2) + pow(target.y - this->current_target.y, 2)) < 2){
        wall_following_direction = this->current_wall_following_direction;
        getRouteSections(agent, start, target, this->current_wall_following_direction, false, route);
        relativeRoute = agent->pathPlanner.coordinateRouteToRelativeVectors(route, agent->heading);
        for (auto edge: relativeRoute) {
            route_length += edge.Length();
        }
    } else {
         //Check both directions, and return the shortest path
        std::vector<std::pair<Coordinate, Coordinate>> route1;
        std::vector<std::pair<Coordinate, Coordinate>> route2;
        getRouteSections(agent, start, target, 1, false, route1);
        getRouteSections(agent, start, target, -1, false, route2);
        std::vector<argos::CVector2> relativeRoute1 = agent->pathPlanner.coordinateRouteToRelativeVectors(route1, agent->heading);
        std::vector<argos::CVector2> relativeRoute2 = agent->pathPlanner.coordinateRouteToRelativeVectors(route2, agent->heading);
        //If one of the routes is empty, return the other
        if (route1.empty()) {
            wall_following_direction = -1;
            route = route2;
            relativeRoute = relativeRoute2;
            route_length = 0;
            for (auto edge: relativeRoute2) {
                route_length += edge.Length();
            }
            return {wall_following_direction, relativeRoute, route_length};
        } else if (route2.empty()) {
            wall_following_direction = 1;
            route = route1;
            relativeRoute = relativeRoute1;
            route_length = 0;
            for (auto edge: relativeRoute1) {
                route_length += edge.Length();
            }
            return {wall_following_direction, relativeRoute, route_length};
        } else { //If both routes are not empty, return the shortest one
            double length_route_1 = 0;
            double length_route_2 = 0;
            for (auto edge: relativeRoute1) {
                length_route_1 += edge.Length();
            }
            for (auto edge: relativeRoute2) {
                length_route_2 += edge.Length();
            }

            if (length_route_1 < length_route_2) {
                wall_following_direction = 1;
                route = route1;
                relativeRoute = relativeRoute1;
                route_length = length_route_1;
            } else {
                wall_following_direction = -1;
                route = route2;
                relativeRoute = relativeRoute2;
                route_length = length_route_2;
            }
        }

    }
    return {wall_following_direction, relativeRoute, route_length};
}

/**
 * Relative route, this means every edge is relative to the previous edge. So the difference in angle. Length is the same.
 * @param route
 * @param agent_heading
 * @return
 */
std::vector<argos::CVector2> SimplePathPlanner::coordinateRouteToRelativeVectors(const std::vector<std::pair<Coordinate, Coordinate>> & route, argos::CRadians agent_heading) const {
    std::vector<argos::CVector2> relative_vectors;
    for (int i = 0; i < route.size(); i++) {
        auto [begin, end] = route[i];
        if (i==0) {
            relative_vectors.emplace_back(argos::CVector2(end.x - begin.x, end.y - begin.y).Rotate(-agent_heading)); //First edge is relative to the agent's heading
        } else {
            auto prev = route[i-1];
            auto prev_vector = argos::CVector2(prev.second.x - prev.first.x, prev.second.y - prev.first.y);
            auto prev_angle = prev_vector.Angle();
            relative_vectors.emplace_back(argos::CVector2(end.x - begin.x, end.y - begin.y).Rotate(prev_angle)); //Other edges are relative to the previous edge
        }
    }
    return relative_vectors;
}

std::vector<std::pair<Coordinate, Coordinate>> SimplePathPlanner::getRouteSections(Agent* agent, Coordinate start, Coordinate target, int wall_following_direction, bool switched_direction, std::vector<std::pair<Coordinate, Coordinate>> & route) const {
    //Small offset towards the target, so the raytracing method selects the correct cell
    auto start_with_offset = Coordinate{start.x + (target.x - start.x) / 1000, start.y + (target.y - start.y) / 1000};
    //Small offset towards the start, so the raytracing method selects the correct cell
    auto target_with_offset = Coordinate{target.x + (start.x - target.x) / 1000, target.y + (start.y - target.y) / 1000};

    auto [cell, box, edge_index, _] = rayTraceQuadtreeOccupiedIntersection(agent, start_with_offset, target_with_offset);
    if (cell == nullptr) route = {{start, target}};
    else {
        auto [start_edge, end_edge] = getEdgeCoordinates(box, edge_index);
        auto edge_middle = Coordinate{(start_edge.x + end_edge.x) / 2, (start_edge.y + end_edge.y) / 2};

        //Line from agent to middle of the intersection edge
        route.emplace_back(agent->position, edge_middle);



//    //Line from middle of the intersection edge to the correct side of the line, depending on the distance of that end to the agent
//    auto distance_to_start = sqrt(pow(start_edge.x - agent->position.x, 2) + pow(start_edge.y - agent->position.y, 2));
//    auto distance_to_end = sqrt(pow(end_edge.x - agent->position.x, 2) + pow(end_edge.y - agent->position.y, 2));
//    if (distance_to_start >= distance_to_end) {
//        route.emplace_back(edge_middle, start_edge);
//    } else {
//        route.emplace_back(edge_middle, end_edge);
//    }

        //Line from middle of the intersection edge to the correct side of the line, depending on the distance of that end to the agent
        if (wall_following_direction == 1) {
            route.emplace_back(edge_middle, start_edge);
        } else {
            route.emplace_back(edge_middle, end_edge);
        }

        getWallFollowingRoute(agent, cell, box.size, edge_index, target, route, wall_following_direction,
                              switched_direction);
        if (route.empty()) return route; //If the route is empty, we can't reach our target, return empty route
        int a = 0;
        for (int i = 0; i < route.size(); i++) {
            auto [begin, end] = route[i];
            if (begin == agent->position) {
                //Erase all edges before this one
                a = i;
            }
        }
        route.erase(route.begin(), route.begin() + a);

        //Append last point to target
        route.emplace_back(route.rbegin()->second, target);
    }

    return route;
}

const double epsilon = 1e-6;

auto isApproximatelyEqual(const Coordinate& a, const Coordinate& b) {
    return std::abs(a.x - b.x) < epsilon && std::abs(a.y - b.y) < epsilon;
};

void SimplePathPlanner::getWallFollowingRoute(Agent* agent, quadtree::Quadtree::Cell * cell, double box_size, int edge_index, Coordinate target, std::vector<std::pair<Coordinate, Coordinate>> & route, int wall_following_direction, bool switched_direction) const {
    //If the route is longer than allowed, return an empty route.
    if (agent->state != Agent::State::RETURNING && route.size() > agent->config.MAX_ROUTE_LENGTH){
        route.clear();
        return;
    }

    //If the direction to the target is free from the cell, return
    auto [direction_free, intersection_cell, intersection_edge_index, intersection_edge_coordinate] = directionToTargetFree(agent, cell, box_size, edge_index, target, wall_following_direction, route);
    if (direction_free > 0) { //If the direction is free (to some extent)
        //Delete the last full edge, and with a line to the middle of the last edge
//        auto [begin_last_edge, end_last_edge] = *route.rbegin();
//        auto edge_middle = Coordinate{(begin_last_edge.x + end_last_edge.x) / 2, (begin_last_edge.y + end_last_edge.y) / 2};
//        route.erase(route.end() - 1);
//
//        route.emplace_back(begin_last_edge, edge_middle);
        if (direction_free == 1) { //We are hitting fan edge not in route yet, continue exploring that edge
//            route.emplace_back(edge_middle, intersection_edge_coordinate);
            getWallFollowingRoute(agent, intersection_cell, box_size, intersection_edge_index, target, route, wall_following_direction, switched_direction);
        }
        return;
    }
    auto [begin_last_edge, end_last_edge] = *route.rbegin();
    auto edges = elegible_edges(agent, cell, edge_index, box_size);

    quadtree::Quadtree::Cell * bestCell = nullptr;
    int bestEdgeIndex = -1;
    Coordinate bestEdgeStart = {MAXFLOAT, MAXFLOAT};
    Coordinate bestEdgeEnd = {MAXFLOAT, MAXFLOAT};

    for (auto [c, e]: edges) {
        auto cBoxTopLeft = Coordinate{c->quadNode.coordinate.x - box_size / 2,
                                      c->quadNode.coordinate.y + box_size / 2};
        auto cBox = quadtree::Box(cBoxTopLeft, box_size);
            std::pair<Coordinate, Coordinate> edge_coordinates = getEdgeCoordinates(cBox, e);
            auto& start_edge = edge_coordinates.first;
            auto& end_edge = edge_coordinates.second;
            if (isApproximatelyEqual(start_edge, end_last_edge) || isApproximatelyEqual(end_edge, end_last_edge)) { // Only use the edge that is connected to the point we are at (end point of last edge)
                if (isApproximatelyEqual(end_last_edge, end_edge)) {
                    //Flip start and end
                    std::swap(start_edge, end_edge);
                }
                auto mid_edge = Coordinate{(start_edge.x + end_edge.x) / 2, (start_edge.y + end_edge.y) / 2};
                //If the edge (either way) is not already in the route, and the distance to the edge is the largest, save the cell and edge
                if (std::find_if(route.begin(), route.end(), [&](const std::pair<Coordinate, Coordinate>& edge) {
                    return (isApproximatelyEqual(edge.first, start_edge) && isApproximatelyEqual(edge.second, end_edge)) ||
                           (isApproximatelyEqual(edge.first, end_edge) && isApproximatelyEqual(edge.second, start_edge)) ||
                           (isApproximatelyEqual(edge.first, start_edge) && isApproximatelyEqual(edge.second, mid_edge)) ||
                           (isApproximatelyEqual(edge.first, end_edge) && isApproximatelyEqual(edge.second, mid_edge));
                }) == route.end()) {
                    bestCell = c;
                    bestEdgeIndex = e;
                    bestEdgeStart = start_edge;
                    bestEdgeEnd = end_edge;
                }
            }
    }
    //TODO: What if we can't find a new edge to go to? We should trace to the target, and go the other direction as in the route
    if (bestCell != nullptr) {
        //If the direction from the agent to the end of the edge is free, don't add the edge to the route, as the we can go directly to the next edge
        if (directionToTargetFree(agent, agent->position, box_size, bestEdgeEnd, route) == 2) {
            route.emplace_back(agent->position, bestEdgeStart);
        }
        route.emplace_back(bestEdgeStart, bestEdgeEnd);
        getWallFollowingRoute(agent, bestCell, box_size, bestEdgeIndex, target, route, wall_following_direction, switched_direction);
    } else {
        //If we can't find a new edge to go to, we should clear the route
        route.clear();
//        if (switched_direction) return; //If we have already switched direction, return empty route
//        getRouteSections(agent, agent->position, target, -wall_following_direction, true, route);
    }

}


/**
 * Checks if the line from the agent to the target is free from obstacles
 * If it is completely free, return 2
 * If it is free, for at least box_size/2, return 1, and the cell and edge information
 * If it is not free, return 0
 * @param agent
 * @param cell
 * @param box_size
 * @param edge_index
 * @param target
 * @param route
 * @return
 */
std::tuple<int, quadtree::Quadtree::Cell *, int, Coordinate> SimplePathPlanner::directionToTargetFree(Agent* agent, quadtree::Quadtree::Cell * cell, double box_size, int edge_index, Coordinate target, int wall_following_direction, std::vector<std::pair<Coordinate, Coordinate>> & route) const{
    Coordinate start = cell->quadNode.coordinate;




    //Get the middle of the edge
    if (edge_index == 0) start.x = start.x - box_size / 2;
    else if (edge_index == 1) start.y = start.y + box_size / 2;
    else if (edge_index == 2) start.x = start.x + box_size / 2;
    else if (edge_index == 3) start.y = start.y - box_size / 2;

    //If the target lies within the cell we are in, the direction technically not free, but we have pretty much arrived
    if (sqrt(pow(target.x - start.x, 2) + pow(target.y - start.y, 2)) < box_size/2) {
        //Check if the target is in the same cell as the agent
        return {2, nullptr, -1, Coordinate{0,0}};
    }



    //Find the intersection with the quadtree
    auto [intersection_cell, intersection_box, intersection_edge, distance_to_intersection] = rayTraceQuadtreeOccupiedIntersection(agent, start, target);

    if (intersection_cell != nullptr) {
        //If we intersect the same cell, the line is going through the cell, so the direction is not free
        //If the intersection edge has a neighbor on it, it is a raytracing accuracy mistake, so the direction is not free
        if (intersection_cell == cell || intersection_cell->neighbors[intersection_edge] != nullptr)
            return {0, nullptr, -1, Coordinate{0,0}};

        std::pair<Coordinate, Coordinate> edge_coordinates = getEdgeCoordinates(intersection_box, intersection_edge);
        auto& start_edge_intersection = edge_coordinates.first;
        auto& end_edge_intersection = edge_coordinates.second;
        auto mid_edge_intersection = Coordinate{(start_edge_intersection.x + end_edge_intersection.x) / 2, (start_edge_intersection.y + end_edge_intersection.y) / 2};
        if (std::find_if(route.begin(), route.end(), [&](const std::pair<Coordinate, Coordinate>& edge) {
            return (isApproximatelyEqual(edge.first, start_edge_intersection) && isApproximatelyEqual(edge.second, end_edge_intersection)) ||
                   (isApproximatelyEqual(edge.first, end_edge_intersection) && isApproximatelyEqual(edge.second, start_edge_intersection)) ||
                   (isApproximatelyEqual(edge.first, mid_edge_intersection) && isApproximatelyEqual(edge.second, start_edge_intersection)) ||
                   (isApproximatelyEqual(edge.first, mid_edge_intersection) && isApproximatelyEqual(edge.second, end_edge_intersection));
        }) != route.end()) {
            return {0, nullptr, -1, Coordinate{0,0}};
        }
        if (distance_to_intersection > box_size / 2){
            auto [begin_last_edge, end_last_edge] = *route.rbegin();
            auto mid_last_edge = Coordinate{(begin_last_edge.x + end_last_edge.x) / 2, (begin_last_edge.y + end_last_edge.y) / 2};
            route.erase(route.end() - 1);

            route.emplace_back(begin_last_edge, mid_last_edge);

            route.emplace_back(mid_last_edge, mid_edge_intersection);

            //If the intersection edge is connected to the last edge, we can go directly to the edge
            if (isApproximatelyEqual(end_last_edge, start_edge_intersection)) {
                route.emplace_back(mid_edge_intersection, end_edge_intersection);
            } else if (isApproximatelyEqual(end_last_edge, end_edge_intersection)) {
                route.emplace_back(mid_edge_intersection, start_edge_intersection);
            } else {

//                auto distance_to_start = sqrt(pow(start_edge_intersection.x - agent->position.x, 2) +
//                                              pow(start_edge_intersection.y - agent->position.y, 2));
//                auto distance_to_end = sqrt(pow(end_edge_intersection.x - agent->position.x, 2) +
//                                            pow(end_edge_intersection.y - agent->position.y, 2));
//                if (distance_to_start >= distance_to_end) {
//                    route.emplace_back(mid_edge_intersection, start_edge_intersection);
//                } else {
//                    route.emplace_back(mid_edge_intersection, end_edge_intersection);
//                }

                    if (wall_following_direction == 1) {
                        route.emplace_back(mid_edge_intersection, start_edge_intersection);
                    } else {
                        route.emplace_back(mid_edge_intersection, end_edge_intersection);
                    }
            }
            return {1, intersection_cell, intersection_edge, mid_edge_intersection}; //The intersection is far enough away to go to the edge
        }
        return {0, nullptr, -1, Coordinate{0,0}};
    }
    auto [begin_last_edge, end_last_edge] = *route.rbegin();
    auto edge_middle = Coordinate{(begin_last_edge.x + end_last_edge.x) / 2, (begin_last_edge.y + end_last_edge.y) / 2};
    route.erase(route.end() - 1);

    route.emplace_back(begin_last_edge, edge_middle);
    return {2, nullptr, -1, Coordinate{0,0}}; //No intersection, so the direction is free

}

int SimplePathPlanner::directionToTargetFree(Agent* agent, Coordinate start, double box_size, Coordinate target, const std::vector<std::pair<Coordinate, Coordinate>> & route) const{

    //Find the intersection with the quadtree
    auto [intersection_cell, intersection_box, intersection_edge, distance_to_intersection] = rayTraceQuadtreeOccupiedIntersection(agent, start, target);

    if (intersection_cell != nullptr) {
        if (intersection_cell->neighbors[intersection_edge] != nullptr) return 0; //If the intersection edge has a neighbor on it, it is a raytracing accuracy mistake, so the direction is not free
        std::pair<Coordinate, Coordinate> edge_coordinates = getEdgeCoordinates(intersection_box, intersection_edge);
        auto& start_edge = edge_coordinates.first;
        auto& end_edge = edge_coordinates.second;
        auto mid_edge = Coordinate{(start_edge.x + end_edge.x) / 2, (start_edge.y + end_edge.y) / 2};
//        if (std::find(route.begin(), route.end(), std::pair{start_edge, end_edge}) !=
//            route.end() ||
//            std::find(route.begin(), route.end(), std::pair{end_edge, start_edge}) !=
//            route.end() ||
//            std::find(route.begin(), route.end(), std::pair{mid_edge, start_edge}) !=
//            route.end() ||
//            std::find(route.begin(), route.end(), std::pair{mid_edge, end_edge}) !=
//            route.end()) { //Can't go back to edge already in route
        if (std::find_if(route.begin(), route.end(), [&](const std::pair<Coordinate, Coordinate>& edge) {
            return (isApproximatelyEqual(edge.first, start_edge) && isApproximatelyEqual(edge.second, end_edge)) ||
                   (isApproximatelyEqual(edge.first, end_edge) && isApproximatelyEqual(edge.second, start_edge)) ||
                   (isApproximatelyEqual(edge.first, mid_edge) && isApproximatelyEqual(edge.second, start_edge)) ||
                   (isApproximatelyEqual(edge.first, mid_edge) && isApproximatelyEqual(edge.second, end_edge));
        }) != route.end()) {
            return 0;

        }
        if (distance_to_intersection > box_size / 2) return 1;
        
        return 0;
    }
    return 2; //No intersection, so the direction is free

}


 std::vector<std::pair<quadtree::Quadtree::Cell*, int>> SimplePathPlanner::elegible_edges(Agent* agent, quadtree::Quadtree::Cell * cell, int edge_index, double box_size) {
    std::vector<std::pair<quadtree::Quadtree::Cell*, int>> edges;
    int opposite_edge_index = edge_index < 2 ? edge_index + 2 : edge_index - 2;

//    bool edge1_in_list = true;
//    bool edge2_in_list = true;
//
//    edges.emplace_back(cell, (edge_index+1)%4);
//    edges.emplace_back(cell, (edge_index+3)%4);

    for (int i = 0; i < 4; i++) {
        if (cell->neighbors.at(i) != nullptr) {
//            auto index = std::find(edges.begin(), edges.end(), std::pair{cell, i});
//            if (index != edges.end()) {
//                edges.erase(index);
//            }
            if (i != edge_index && i != opposite_edge_index) {
                if (cell->neighbors.at(i)->neighbors.at(edge_index) != nullptr) {
                    auto opposite_i = i < 2 ? i + 2 : i - 2;
                    edges.emplace_back(cell->neighbors.at(i)->neighbors.at(edge_index), opposite_i);
                } else {
                    edges.emplace_back(cell->neighbors.at(i), edge_index);
                }
            }
        } else {
            if (i != edge_index && i != opposite_edge_index) {
                //If there is not an occupied cell diagonally from this cell, we can add this edge

                auto x_factor = -1;
                auto y_factor = -1;

                if(edge_index == 0){
                    x_factor = -1;
                    if (i == 1) y_factor = 1;
                    if (i == 3 ) y_factor = -1;
                } else if (edge_index == 1){
                    y_factor = 1;
                    if (i == 0) x_factor = -1;
                    if (i == 2) x_factor = 1;
                } else if (edge_index == 2){
                    x_factor = 1;
                    if (i == 1) y_factor = 1;
                    if (i == 3) y_factor = -1;
                } else if (edge_index == 3){
                    y_factor = -1;
                    if (i == 0) x_factor = -1;
                    if (i == 2) x_factor = 1;
                }

                auto diagonal_cell = Coordinate{cell->quadNode.coordinate.x + box_size*x_factor, cell->quadNode.coordinate.y + box_size*y_factor};
                auto cell_and_box = agent->quadtree->getCellandBoxFromCoordinate(diagonal_cell);

                //If we haven't discovered the cell yet, or the cell is not occupied, we can add the edge
                if (cell_and_box.first == nullptr || cell_and_box.first->quadNode.occupancy != quadtree::Occupancy::OCCUPIED) {
                    edges.emplace_back(cell, i);
                } else { //Add edge on that cell
                    auto opposite_i = i < 2 ? i + 2 : i - 2;
                    edges.emplace_back(cell_and_box.first, opposite_i);
                }
            }
        }
    }


    return edges;
}


//std::tuple<quadtree::Quadtree::Cell*, quadtree::Box, int, double> SimplePathPlanner::rayTraceQuadtreeOccupiedIntersection(Agent* agent, Coordinate start, Coordinate target) const {
//    auto x = start.x;
//    auto y = start.y;
//    auto dx = target.x - start.x;
//    auto dy = target.y - start.y;
//    auto distance = sqrt(dx * dx + dy * dy);
//    auto stepSize = agent->quadtree->getResolution()/4;
//    auto nSteps = std::ceil(distance / stepSize);
//    auto stepX = dx / nSteps;
//    auto stepY = dy / nSteps;
//
//    //Already do one step to avoid the start cell
//    x += stepX;
//    y += stepY;
//    for (int s = 1; s < nSteps; s++) {
//        auto cell_and_box = agent->quadtree->getCellandBoxFromCoordinate(Coordinate{x, y});
//        auto cell = cell_and_box.first;
//        auto box = cell_and_box.second;
//        if (cell != nullptr) {
//            if (cell->quadNode.occupancy == quadtree::Occupancy::OCCUPIED) {
//                Coordinate intersection = liang_barsky(start, target, box);
//                //Get the edge that the intersection is on (with small margin)
//                //left = 0, top = 1, right = 2, bottom = 3
//                auto edge_index = -1;
//                if (std::abs(intersection.x - box.left)<0.00001) edge_index = 0;
//                else if (std::abs(intersection.y - box.top)<0.00001) edge_index = 1;
//                else if (std::abs(intersection.x - box.getRight())<0.00001) edge_index = 2;
//                else if (std::abs(intersection.y - box.getBottom())<0.00001) edge_index = 3;
//
//                auto dist_to_intersection = sqrt(pow(intersection.x - start.x, 2) + pow(intersection.y - start.y, 2));
//
//                return {cell, box, edge_index, dist_to_intersection};
//            }
//        }
//        x += stepX;
//        y += stepY;
//    }
//    return {nullptr, quadtree::Box(), -1, 0};
//
//}

/**
 * Use the breseham line algorithm to get the points between the start and target
 * Then check if the cell of the point is occupied
 * If it is, return the cell, the box of the cell, the edge index of the intersection (liang_barsky), and the distance to the intersection
 * @param agent
 * @param start
 * @param target
 * @return
 */
std::tuple<quadtree::Quadtree::Cell*, quadtree::Box, int, double> SimplePathPlanner::rayTraceQuadtreeOccupiedIntersection(Agent* agent, Coordinate start, Coordinate target) const {
//    auto [start_cell, start_box] = agent->quadtree->getCellandBoxFromCoordinate(start);
//    while (start_box.size > agent->quadtree->getResolution()) { //Make sure we are in the smallest box
//        int quadrant = start_box.getQuadrant(start);
//        if (quadrant == 4) start_box = start_box.boxFromQuadrant(0); //If the start is in the center, we can't get the quadrant, so we just take the top left
//        else start_box = start_box.boxFromQuadrant(quadrant);
//    }
//    auto [target_cell, target_box] = agent->quadtree->getCellandBoxFromCoordinate(target);
//    while (target_box.size > agent->quadtree->getResolution()) {
//        int quadrant = target_box.getQuadrant(target);
//        if (quadrant == 4) target_box = target_box.boxFromQuadrant(0); //If the target is in the center, we can't get the quadrant, so we just take the top left
//        else target_box = target_box.boxFromQuadrant(quadrant);
//
//    }
    std::vector<Coordinate> linePoints = Algorithms::Amanatides_Woo_Voxel_Traversal(agent, start,
                                                                                    target);
    for (const auto& point: linePoints) {
        auto cell_and_box = agent->quadtree->getCellandBoxFromCoordinate(point);
        auto cell = cell_and_box.first;
        auto box = cell_and_box.second;
        if (cell != nullptr) {
            if (cell->quadNode.occupancy == quadtree::Occupancy::OCCUPIED) {
                Coordinate intersection = liang_barsky(start, target, box);
                //Get the edge that the intersection is on (with small margin)
                //left = 0, top = 1, right = 2, bottom = 3
                auto edge_index = -1;
                if (std::abs(intersection.x - box.left)<0.00001) edge_index = 0;
                else if (std::abs(intersection.y - box.top)<0.00001) edge_index = 1;
                else if (std::abs(intersection.x - box.getRight())<0.00001) edge_index = 2;
                else if (std::abs(intersection.y - box.getBottom())<0.00001) edge_index = 3;
                if (edge_index != -1) { //If the intersection is not on an edge, it is not a valid intersection
                    auto dist_to_intersection = sqrt(
                            pow(intersection.x - start.x, 2) + pow(intersection.y - start.y, 2));

                    return {cell, box, edge_index, dist_to_intersection};
                }
            }
        }
    }
    return {nullptr, quadtree::Box(), -1, 0};

}

/**
 * From the python implementation of Liang-Barsky: https://www.geeksforgeeks.org/liang-barsky-algorithm/
 * @param p1
 * @param p2
 * @param box
 * @return
 */
Coordinate SimplePathPlanner::liang_barsky(Coordinate p1, Coordinate p2, quadtree::Box box) const{
    double t0 = 0.0;
    double t1 = 1.0;
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double p = 0.0;
    double q = 0.0;
    double r = 0.0;

    for (int edge = 0; edge < 4; edge++) {
        switch (edge) {
            case 0:
                p = -dx;
//                q = -(box.left - p1.x);
                q = p1.x - box.left;
                break;
            case 1:
                p = dx;
                q = box.getRight() - p1.x;
                break;
            case 2:
                p = -dy;
//                q = -(box.getBottom() - p1.y);
                q = p1.y - box.getBottom();
                break;
            case 3:
                p = dy;
                q = box.top- p1.y;
                break;
        }

        if (p == 0 && q < 0) return Coordinate{MAXFLOAT, MAXFLOAT};
        if (p == 0) continue;

        r = q / p;
        if (p < 0) {
            if (r > t1) return Coordinate{MAXFLOAT, MAXFLOAT};
            else if (r > t0) t0 = r;
        } else {
            if (r < t0) return Coordinate{MAXFLOAT, MAXFLOAT};
            else if (r < t1) t1 = r;
        }
    }

    auto t0x = p1.x + t0 * dx;
    auto t0y = p1.y + t0 * dy;
    auto t1x = p1.x + t1 * dx;
    auto t1y = p1.y + t1 * dy;

    if (t0 == 0.0) {
        t0x = MAXFLOAT;
        t0y = MAXFLOAT;
    } else if (t1 == 1.0) {
        t1x = MAXFLOAT;
        t1y = MAXFLOAT;
    }
    return t0x == MAXFLOAT ? Coordinate{t1x, t1y} : Coordinate{t0x, t0y};
}

std::pair<Coordinate, Coordinate> SimplePathPlanner::getEdgeCoordinates(quadtree::Box box, int edge_index) const {
    switch (edge_index) {
        case 0:
            return {Coordinate{box.left, box.top}, Coordinate{box.left, box.getBottom()}};
        case 1:
            return {Coordinate{box.getRight(), box.top}, Coordinate{box.left, box.top}};
        case 2:
            return {Coordinate{box.getRight(), box.getBottom()}, Coordinate{box.getRight(), box.top}};
        case 3:
            return {Coordinate{box.left, box.getBottom()}, Coordinate{box.getRight(), box.getBottom()}};
        default:
            return {Coordinate{MAXFLOAT, MAXFLOAT}, Coordinate{MAXFLOAT, MAXFLOAT}};
    }
}

#endif
