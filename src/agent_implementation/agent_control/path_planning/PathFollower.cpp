//
// Created by hugo on 28-11-24.
//

#include "PathFollower.h"
// #include "agent_implementation/agent.h"
#include "../../agent.h"
#include "../../utils/Algorithms.h"

#ifdef PATH_PLANNING_ENABLED
Coordinate PathFollower::followPath(Agent *agent) {
    if (agent->route_to_best_frontier.empty()) return Coordinate{MAXFLOAT, MAXFLOAT};

//    for (auto [start, end]: agent->route_to_best_frontier) {
//        argos::LOG << "Route: " << start.x << ", " << start.y << " to " << end.x << ", " << end.y << std::endl;
//    }
//    for (auto [start, end]: current_route){
//        argos::LOG << "Current route: " << start.x << ", " << start.y << " to " << end.x << ", " << end.y << std::endl;
//    }
    if (current_route != agent->route_to_best_frontier) {
        current_path_section = 0;
    }

    current_route = agent->route_to_best_frontier;

    //If we are past the last section, return the last section
    if (current_path_section >= agent->route_to_best_frontier.size()) {
        return agent->route_to_best_frontier.back().second;
    }

//    argos::LOG << "route size: " << agent->route_to_best_frontier.size() << " and current path section: " << current_path_section << std::endl;
    auto [start, end] = agent->route_to_best_frontier.at(current_path_section);

    //Our target is the end of the edge
    Coordinate sub_target = end;

    //If we are close to the target, go to the next section
    if (sqrt(pow(sub_target.x - agent->position.x, 2) + pow(sub_target.y - agent->position.y, 2)) < agent->config.OBJECT_AVOIDANCE_RADIUS*3) {
        //Check if direction to the sub_target is also free
        if (!rayTraceQuadtreeOccupiedIntersection(agent, agent->position, sub_target)) {
            current_path_section++;
            //If we are past the end of the path, return the last section
            if (current_path_section >= agent->route_to_best_frontier.size()) {
                return agent->route_to_best_frontier.back().second;
            }
            auto [next_start, next_end] = agent->route_to_best_frontier.at(current_path_section);
            sub_target = next_end;
        }
    }


    return sub_target;

}

bool PathFollower::rayTraceQuadtreeOccupiedIntersection(Agent* agent, Coordinate start, Coordinate target) const {
//    auto x = start.x;
//    auto y = start.y;
//    auto dx = target.x - start.x;
//    auto dy = target.y - start.y;
//    auto distance = sqrt(dx * dx + dy * dy);
//    auto stepSize = agent->quadtree->getResolution()/4;
//    auto nSteps = std::ceil(distance / stepSize);
//    auto stepX = dx / nSteps;
//    auto stepY = dy / nSteps;
//    quadtree::Box prev_box = quadtree::Box();
//
//    for (int s = 0; s < nSteps; s++) {
//        auto coordinate = Coordinate{x, y};
//        if (!prev_box.contains(coordinate)) { //We don't have to check the same box twice
//            auto cell_and_box = agent->quadtree->getCellandBoxFromCoordinate(Coordinate{x, y});
//            auto cell = cell_and_box.first;
//            auto box = cell_and_box.second;
//            if (cell != nullptr) {
//                if (cell->quadNode.occupancy == quadtree::Occupancy::OCCUPIED) {
//                    return true;
//                }
//            }
//        }
//        x += stepX;
//        y += stepY;
//    }

    std::vector<Coordinate> linePoints = Algorithms::Amanatides_Woo_Voxel_Traversal(agent, start,
                                                                                    target);
    for (const auto& point: linePoints) {
        auto cell_and_box = agent->quadtree->getCellandBoxFromCoordinate(point);
        auto cell = cell_and_box.first;
        if (cell != nullptr) {
            if (cell->quadNode.occupancy == quadtree::Occupancy::OCCUPIED) {
                return true;
            }
        }
    }
    return false;

}
//bool PathFollower::rayTraceQuadtreeOccupiedIntersection(Agent* agent, Coordinate start, Coordinate target) const {
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
//    std::vector<Coordinate> linePoints = Algorithms::Amanatides_Woo_Voxel_Traversal(agent, start_box.getCenter(), target_box.getCenter());
//    for (const auto& point: linePoints) {
//        auto cell_and_box = agent->quadtree->getCellandBoxFromCoordinate(point);
//        auto cell = cell_and_box.first;
//        auto box = cell_and_box.second;
//        if (cell != nullptr) {
//            if (cell->quadNode.occupancy == quadtree::Occupancy::OCCUPIED) {
//                return true;
//            }
//        }
//    }
//    return false;
//
//}

#endif