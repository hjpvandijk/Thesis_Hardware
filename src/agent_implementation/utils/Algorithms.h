//
// Created by hugo on 7-1-25.
//

#ifndef IMPLEMENTATION_AND_EXAMPLES_ALGORITHMS_H
#define IMPLEMENTATION_AND_EXAMPLES_ALGORITHMS_H

#include <vector>
#include "coordinate.h"

class Agent;


class Algorithms {
public:
    static bool is_multiple(double a, double b,  double epsilon = 1e-10);
    static std::vector<Coordinate> Amanatides_Woo_Voxel_Traversal(Agent *agent, Coordinate coordinate1, Coordinate coordinate2);

};


#endif //IMPLEMENTATION_AND_EXAMPLES_ALGORITHMS_H
