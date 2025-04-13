//
// Created by hugo on 7-1-25.
//

#include "Algorithms.h"
// #include "agent_implementation/agent.h"
#include "../agent.h"

/**
 * Checks if a is a multiple of b, with a small epsilon
 * @param a
 * @param b
 * @param epsilon
 * @return
 */
bool Algorithms::is_multiple(double a, double b, double epsilon) {
    double remainder = fmod(a, b);
    return (std::fabs(remainder) < epsilon || std::fabs(std::fabs(remainder) - b) < epsilon);
}

/**
 * Adapted from https://www.cse.yorku.ca/~amana/research/grid.pdf?
 * @param agent
 * @param coordinate1
 * @param coordinate2
 * @return
 */
std::vector<Coordinate> Algorithms::Amanatides_Woo_Voxel_Traversal(Agent* agent, Coordinate coordinate1, Coordinate coordinate2) {
    double box_size = agent->quadtree->getResolution();

//    coordinate1.x = round(coordinate1.x * 1000000)/1000000;
//    coordinate1.y = round(coordinate1.y * 1000000)/1000000;
//    coordinate2.x = round(coordinate2.x * 1000000)/1000000;
//    coordinate2.y = round(coordinate2.y * 1000000)/1000000;

    //Because coordinate1 is on an edge, we will add a small offset to the coordinate1, towards the coordinate2, so the raytracing method selects the correct cell
    while (is_multiple(coordinate1.x, box_size) || is_multiple(coordinate1.y, box_size)) {
        auto addx = 0.000001;
        if (coordinate2.x < coordinate1.x) addx = -0.000001;
        auto addy = 0.000001;
        if (coordinate2.y < coordinate1.y) addy = -0.000001;
        coordinate1 = Coordinate{coordinate1.x + addx, coordinate1.y + addy};
    }
    //In case the coordinate2 is on the edge, we will add a small offset to the coordinate2, away from the coordinate1, so the raytracing method selects the correct cell
    while (is_multiple(coordinate2.x, box_size) || is_multiple(coordinate2.y, box_size)) {
        auto addx = 0.000001;
        if (coordinate1.x < coordinate2.x) addx = -0.000001;
        auto addy = 0.000001;
        if (coordinate1.y < coordinate2.y) addy = -0.000001;
        coordinate2 = Coordinate{coordinate2.x + addx, coordinate2.y + addy};
    }

    std::vector<Coordinate> points;

    double x1 = coordinate1.x;
    double y1 = coordinate1.y;
    double x2 = coordinate2.x;
    double y2 = coordinate2.y;
    double x = floor(x1/box_size);
    double y = floor(y1/box_size);
    double xEnd = floor(x2/box_size);
    double yEnd = floor(y2/box_size);

    double dx = x2 - x1;
    double dy = y2 - y1;

    int stepX = dx > 0 ? 1 : -1;
    int stepY = dy > 0 ? 1 : -1;
    //If the line is vertical or horizontal, we can't divide by dx or dy, so we set the tMax to infinity
    double tMaxX = dx == 0 ? MAXFLOAT : (dx > 0 ? (floor(x1/box_size) + 1) * box_size - x1 : x1 - floor(x1/box_size) * box_size) / abs(dx);
    double tMaxY = dy == 0 ? MAXFLOAT : (dy > 0 ? (floor(y1/box_size) + 1) * box_size - y1 : y1 - floor(y1/box_size) * box_size) / abs(dy);
    double tDeltaX = dx == 0 ? MAXFLOAT : box_size / abs(dx);
    double tDeltaY = dy == 0 ? MAXFLOAT : box_size / abs(dy);

    points.push_back(Coordinate{(x+0.5)*box_size, (y+0.5)*box_size});

    while (x != xEnd || y != yEnd) {
        if (tMaxX < tMaxY) {
            tMaxX += tDeltaX;
            x += stepX;
        }  else {
            tMaxY += tDeltaY;
            y += stepY;
        }
        points.push_back(Coordinate{(x+0.5)*box_size, (y+0.5)*box_size});
    }


    return points;
}