#pragma once
#include <functional>

#include <cassert>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <random>
#include <type_traits>
#include <vector>
#include "Box.h"
#include <iostream>
#include <fstream>

//Adapted from https://github.com/pvigier/Quadtree

namespace quadtree {

    enum Occupancy {
        UNKNOWN,
        FREE,
        OCCUPIED,
        ANY,
        AMBIGUOUS
    };


    struct QuadNode {
        Coordinate coordinate;
        Occupancy occupancy;
        double visitedAtS;
        double LConfidence = 0.0; //The L(n) confidence of the reachability of this node by the agent. Positive is FREE, negative is OCCUPIED.

        bool operator==(const QuadNode &rhs) const {
            return coordinate.x == rhs.coordinate.x && coordinate.y == rhs.coordinate.y;
        }

    };

    class Quadtree {

    public:
        int numberOfLeafNodes = 0; //Number of leaf nodes not UNKNOWN or ANY
        int numberOfCells = 0; //Number of cells in the quadtree
        int numberOfNodesPerMessage = 9999;//50; //Number of nodes to send per message

        struct Cell {
            Cell * parent;
            std::array<std::unique_ptr<Cell>, 4> children;
            std::array<Cell *, 4> neighbors; //Neighbors with the same occupancy: [left, top, right, bottom]
            //            std::vector<QuadNode> values;
            QuadNode quadNode = QuadNode{Coordinate{0, 0}, Occupancy::UNKNOWN, -1};
            int& cellCounter;

            Cell(int& counter) : cellCounter(counter){
                parent = nullptr;
                children = {nullptr, nullptr, nullptr, nullptr};
                neighbors = {nullptr, nullptr, nullptr, nullptr};
                cellCounter++;
            }

            ~Cell(){
                cellCounter--;
            }
//
//
//
//            // Custom copy constructor
//            Cell(const Cell& other) : parent(other.parent), neighbors(other.neighbors), quadNode(other.quadNode) {
//                for (size_t i = 0; i < children.size(); ++i) {
//                    if (other.children[i]) {
//                        children[i] = std::make_unique<Cell>(*other.children[i]);
//                    }
//                }
//            }

            void add_occupied_neighbors(double boxSize){
                if (this->parent == nullptr) return;

                auto left = Coordinate{this->quadNode.coordinate.x-boxSize, this->quadNode.coordinate.y};
                auto top = Coordinate{this->quadNode.coordinate.x, this->quadNode.coordinate.y+boxSize};
                auto right = Coordinate{this->quadNode.coordinate.x+boxSize, this->quadNode.coordinate.y};
                auto bottom = Coordinate{this->quadNode.coordinate.x, this->quadNode.coordinate.y-boxSize};
                Coordinate neighbor_array[] = {left, top, right, bottom};


                for (int neighbor_index = 0; neighbor_index < 4; neighbor_index++) {
                    if (this->neighbors.at(neighbor_index) != nullptr) continue;
                    auto current_cell = this->parent;
                    auto currentBoxTopLeft = Coordinate{this->parent->quadNode.coordinate.x - boxSize, this->parent->quadNode.coordinate.y + boxSize};
                    auto currentBox = Box{currentBoxTopLeft, boxSize * 2};

                    auto neighbor_coordinate = neighbor_array[neighbor_index];
                    auto atRoot = false;
                    while(!currentBox.contains(neighbor_coordinate)){
                        if (current_cell->parent == nullptr){
                            atRoot = true; //If we are at the root, we can't go further (when we are at the edge of the mbox)
                            break;
                        }
                        currentBoxTopLeft = Coordinate{current_cell->parent->quadNode.coordinate.x - currentBox.size, current_cell->parent->quadNode.coordinate.y + currentBox.size};
                        currentBox = Box{currentBoxTopLeft, currentBox.size * 2};
                        current_cell = current_cell->parent;
                    }
                    if (atRoot) continue; //If we are at the root, we can't go further (when we are at the edge of the mbox)
                    if (currentBox.contains(neighbor_coordinate)) {
                        auto atEnd = false;
                        //Find the cell
                        while (static_cast<bool>(current_cell->children.at(0))) { //Until leaf
                            auto childIndex = currentBox.getQuadrant(neighbor_coordinate);
                            auto child = current_cell->children.at(childIndex).get();
                            if (child->quadNode.visitedAtS == -1) { //The child is never set
                                atEnd = true;
                                break;
                            }
                            current_cell = current_cell->children.at(childIndex).get();
                            currentBoxTopLeft = Coordinate{current_cell->quadNode.coordinate.x - currentBox.size / 4, current_cell->quadNode.coordinate.y + currentBox.size / 4};
                            currentBox = Box{currentBoxTopLeft, currentBox.size / 2};
                        }
                        if (atEnd) continue; //The corresponding cell is not visited yet
                        if (current_cell->quadNode.occupancy==OCCUPIED){
                            this->neighbors.at(neighbor_index) = current_cell;
                            auto opposite_neighbor_index = neighbor_index < 2 ? neighbor_index + 2 : neighbor_index - 2;
                            current_cell->neighbors.at(opposite_neighbor_index) = this;
                        }
                    }
                }

            }

            /**
             * @brief Remove the neighbors of this cell, and remove this cell from the neighbors
             */
            void remove_neighbors(){
                for (int i = 0; i < 4; i++) {
                    if (this->neighbors.at(i) != nullptr) {
                        auto opposite_neighbor_index = i < 2 ? i + 2 : i - 2;
                        //Delete this cell from the neighbor
                        this->neighbors.at(i)->neighbors.at(opposite_neighbor_index) = nullptr;
                        //Delete the neighbor from this cell
                        this->neighbors.at(i) = nullptr;
                    }
                }
            }


        };

        Quadtree(const Box &box, float P_FREE_THRESHOLD, float P_OCCUPIED_THRESHOLD, float P_MAX, float P_MIN, float ALPHA_RECEIVE, double RESOLUTION, double EVAPORATION_TIME_S, double EVAPORATED_PHEROMONE_FACTOR, double MERGE_MAX_VISITED_TIME_DIFF, double MERGE_MAX_P_CONFIDENCE_DIFF) :
                mBox(box), mRoot(std::make_unique<Cell>(this->numberOfCells)), P_FREE_THRESHOLD(P_FREE_THRESHOLD), P_OCCUPIED_THRESHOLD(P_OCCUPIED_THRESHOLD), ALPHA_RECEIVE(ALPHA_RECEIVE), RESOLUTION(RESOLUTION), EVAPORATION_TIME_S(EVAPORATION_TIME_S), EVAPORATED_PHEROMONE_FACTOR(EVAPORATED_PHEROMONE_FACTOR), MERGE_MAX_VISITED_TIME_DIFF(MERGE_MAX_VISITED_TIME_DIFF), MERGE_MAX_P_CONFIDENCE_DIFF(MERGE_MAX_P_CONFIDENCE_DIFF) {
            mRoot->quadNode = QuadNode{box.getCenter(), UNKNOWN, -1};
            L_FREE_THRESHOLD = L(P_FREE_THRESHOLD);
            l_max = L(P_MAX);
            l_min = L(P_MIN);
            numberOfCells = 0;
        }

        /**
         * @brief Add a coordinate to the quadtree with the given occupancy
         * @param coordinate
         * @param occupancy
         */
        Box add(Coordinate coordinate, float Pn_zt, double visitedAtS, double currentTimeS) {
            auto LConfidence = L(Pn_zt);
            double pheromone = calculatePheromone(visitedAtS, Pn_zt, currentTimeS);
            Occupancy occ = AMBIGUOUS;
            if (pheromone >= P_FREE_THRESHOLD){
                occ = FREE;
            } else if (pheromone <= P_OCCUPIED_THRESHOLD){
                occ = OCCUPIED;
            }
            auto node = QuadNode{coordinate, occ, visitedAtS, LConfidence};
            return add(node, currentTimeS);
        }

        /**
         * @brief Add a QuadNode to the quadtreee
         * @param value
         */
        Box add(const QuadNode &value, double currentTimeS) {
            //Adding from own observation
            return add(mRoot.get(), mBox, value, currentTimeS, true);
        }

        /**
         * @brief Add a QuadNode to the quadtree with a given a factor. Which decides how much the value is pulled towards 0.5P = ambiguous..
         * @param value
         */
        Box addFromOther(QuadNode &value, double currentTimeS) {
            double pheromone = P(value.LConfidence);
            if (value.visitedAtS != currentTimeS) //If the node was visited at the current time, the time factor will be 1.
                pheromone = calculatePheromone(value.visitedAtS, pheromone, currentTimeS);
            Occupancy occ = AMBIGUOUS;
            if (pheromone >= P_FREE_THRESHOLD){
                occ = FREE;
            } else if (pheromone <= P_OCCUPIED_THRESHOLD){
                occ = OCCUPIED;
            }
            value.occupancy = occ;
            //Adding from other agent's belief
            return add(mRoot.get(), mBox, value, currentTimeS, false);
        }


        void remove(const QuadNode &value) {
            remove(mRoot.get(), mBox, value);
        }

        /**
         * Returns all the occupied QuadNodes surrounding the given coordinate within the given area size
         * @param coordinate
         * @param areaSize
         * @return
         */
        std::vector<QuadNode> queryOccupied(Coordinate coordinate, double areaSize) const {
            // Create a box centered at the given coordinate
            Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);


            return query(box, OCCUPIED);
        }


        /**
         * Returns all the occupied boxes surrounding the given coordinate within the given area size
         * @param coordinate
         * @param areaSize
         * @return
         */
        std::vector<Box> queryOccupiedBoxes(Coordinate coordinate, double areaSize, double currentTimeS) {
            // Create a box centered at the given coordinate
            Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);

            return queryBoxes(box, {OCCUPIED}, currentTimeS);
        }

        /**
         * Get all boxes surrounding the given coordinate within the given area size
         * @param coordinate
         * @param areaSize
         * @param currentTimeS
         * @return
         */
        std::vector<std::pair<Box, double>> queryBoxesAndPheromones(Coordinate coordinate, double areaSize, double currentTimeS) {
            // Create a box centered at the given coordinate
            Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);

            return queryBoxesAndPheromones(box, {FREE, OCCUPIED, AMBIGUOUS}, currentTimeS);
        }

//        /**
//         * Returns all the unexplored boxes surrounding the given coordinate within the given area size
//         * @param coordinate
//         * @param areaSize
//         * @return
//         */
//        std::vector<Box> queryUnexploredBoxes(Coordinate coordinate, double areaSize, double currentTimeS) const {
//            // Create a box centered at the given coordinate
//            Box box = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);
//
//
//            return queryBoxes(box, UNKNOWN, currentTimeS);
//        }


        void processBox(const Box &box, std::vector<std::pair<Box, double>> &frontierBoxes, int current_quadrant, double currentTimeS) const {
            if (std::abs(box.size - RESOLUTION) < 1e-6) { //If the box is the smallest resolution (account for floating point / rounding errors)
                double MooreNeighborPheromone = isMooreNeighbourUnknownOrAmbighous(box, current_quadrant, currentTimeS);
                if (MooreNeighborPheromone != -1) {
                    frontierBoxes.emplace_back(box, MooreNeighborPheromone);
                }
            } else {
                for (int i = 0; i < 4; i++) {
//                    if (current_quadrant + i !=
//                        3) {  //Only process the boxes that are at the outer edges of the queried box
                        Box childBox = computeBox(box, i);
                        processBox(childBox, frontierBoxes, i, currentTimeS);
//                    }
                }

            }
        }

/**
* Returns all the frontier boxes surrounding the given coordinate within the given area size
*/
        std::vector<std::pair<Box, double>> queryFrontierBoxes(Coordinate coordinate, double areaRadius, double currentTimeS, double cell_ratio) {
            Box box = Box(Coordinate{coordinate.x - areaRadius, coordinate.y + areaRadius}, areaRadius*2.0);
            std::vector<Box> exploredBoxes = queryBoxes(box, {FREE, AMBIGUOUS}, currentTimeS); //Get FREE and AMBIGUOUS boxes, as the latter might be FREE
            auto max_cells = std::max(1, static_cast<int>(exploredBoxes.size() * cell_ratio)); //Get random subset of the explored boxes, of size max_cells
            //Get random subset of the explored boxes, of size max_cells
            if (exploredBoxes.size() > max_cells) {
                std::shuffle(exploredBoxes.begin(), exploredBoxes.end(), std::default_random_engine(std::rand()));  // Temporary RNG
                exploredBoxes.resize(max_cells);
            }

            std::vector<std::pair<Box, double>> frontierBoxesAndConfidence;

            for (const auto &exploredBox: exploredBoxes) {
                processBox(exploredBox, frontierBoxesAndConfidence, -1, currentTimeS);
            }

            return frontierBoxesAndConfidence;
        }

        /**
         * Find if at least one of the 8-connected moore neighboring quadnodes of a given box is unexplored or ambiguous
         * Returns the pheromone of the unexplored or ambiguous node. Or -1 if none is found.
         * @param box
         * @return
         */
        double isMooreNeighbourUnknownOrAmbighous(const Box &box, int current_quadrant, double currentTimeS) const {
//            argos::LOG << "Checking moore neighbours of box: " << box.getCenter().x << " " << box.getCenter().y << " of size " << box.getSize() << std::endl;
            //0=NW, 1=NE, 2=SW, 3=SE

            //If we find an unexplored (pheromone == 0.5) cell, return that pheromone, else return the minimum pheromone found

            double pheromones[8] = {-1, -1, -1, -1, -1, -1, -1, -1};

            //Only check the moore neighbours that are at the outer edges of the queried box
            //So only check if current quadrant in the WEST (left)
            if (current_quadrant == -1 || current_quadrant == 0 || current_quadrant == 2) {
                //See if coordinate to the left is in the quadtree and get its occupancy
                Coordinate left = Coordinate{box.getCenter().x - box.size, box.getCenter().y};
                if (mBox.contains(left)) {
//                argos::LOG << "left: " << left.x << " " << left.y << std::endl;
                    double pheromone = getPheromoneFromCoordinateOnlyAmbiguous(left, currentTimeS);
                    if (pheromone == 0.5) {
                        return pheromone;
                    } else {
                        pheromones[0] = pheromone;
                    }
                }
            }
            //So only check if current quadrant in the EAST (right)
            if (current_quadrant == -1 || current_quadrant == 1 || current_quadrant == 3) {
                //See if coordinate to the right is in the quadtree and get its occupancy
                Coordinate right = Coordinate{box.getCenter().x + box.size, box.getCenter().y};
                if (mBox.contains(right)) {
//                argos::LOG << "right: " << right.x << " " << right.y << std::endl;
                    double pheromone = getPheromoneFromCoordinateOnlyAmbiguous(right, currentTimeS);
                    if (pheromone == 0.5) {
                        return pheromone;
                    } else {
                        pheromones[1] = pheromone;
                    }
                }
            }
            //So only check if current quadrant in the NORTH (top)
            if (current_quadrant == -1 || current_quadrant == 0 || current_quadrant == 1) {
                //See if coordinate to the top is in the quadtree and get its occupancy
                Coordinate top = Coordinate{box.getCenter().x, box.getCenter().y + box.size};
                if (mBox.contains(top)) {
//                argos::LOG << "top: " << top.x << " " << top.y << std::endl;
                    double pheromone = getPheromoneFromCoordinateOnlyAmbiguous(top, currentTimeS);
                    if (pheromone == 0.5) {
                        return pheromone;
                    } else {
                        pheromones[2] = pheromone;
                    }
                }
            }
            //So only check if current quadrant in the SOUTH (bottom)
            if (current_quadrant == -1 || current_quadrant == 2 || current_quadrant == 3) {
                //See if coordinate to the bottom is in the quadtree and get its occupancy
                Coordinate bottom = Coordinate{box.getCenter().x, box.getCenter().y - box.size};
                if (mBox.contains(bottom)) {
//                argos::LOG << "bottom: " << bottom.x << " " << bottom.y << std::endl;
                    double pheromone = getPheromoneFromCoordinateOnlyAmbiguous(bottom, currentTimeS);
                    if (pheromone == 0.5) {
                        return pheromone;
                    } else {
                        pheromones[3] = pheromone;
                    }
                }
            }
            //So don't check if current quadrant in the SOUTH EAST (bottom right)
            if (current_quadrant == -1 || current_quadrant == 0 || current_quadrant == 1 || current_quadrant == 2) {
                //See if coordinate to the top left is in the quadtree and get its occupancy
                Coordinate topLeft = Coordinate{box.getCenter().x - box.size, box.getCenter().y + box.size};
                if (mBox.contains(topLeft)) {
//                argos::LOG << "topLeft: " << topLeft.x << " " << topLeft.y << std::endl;
                    double pheromone = getPheromoneFromCoordinateOnlyAmbiguous(topLeft, currentTimeS);
                    if (pheromone == 0.5) {
                        return pheromone;
                    } else {
                        pheromones[4] = pheromone;
                    }
                }
            }
            //So don't check if current quadrant in the SOUTH WEST (bottom left)
            if (current_quadrant == -1 || current_quadrant == 0 || current_quadrant == 1 || current_quadrant == 3) {
                //See if coordinate to the top right is in the quadtree and get its occupancy
                Coordinate topRight = Coordinate{box.getCenter().x + box.size, box.getCenter().y + box.size};
                if (mBox.contains(topRight)) {
//                argos::LOG << "topRight: " << topRight.x << " " << topRight.y << std::endl;
                    double pheromone = getPheromoneFromCoordinateOnlyAmbiguous(topRight, currentTimeS);
                    if (pheromone == 0.5) {
                        return pheromone;
                    } else {
                        pheromones[5] = pheromone;
                    }
                }
            }
            //So don't check if current quadrant in the NORTH EAST (top right)
            if (current_quadrant == -1 || current_quadrant == 0 || current_quadrant == 2 || current_quadrant == 3) {
                //See if coordinate to the bottom left is in the quadtree and get its occupancy
                Coordinate bottomLeft = Coordinate{box.getCenter().x - box.size, box.getCenter().y - box.size};
                if (mBox.contains(bottomLeft)) {
//                argos::LOG << "bottomLeft: " << bottomLeft.x << " " << bottomLeft.y << std::endl;
                    double pheromone = getPheromoneFromCoordinateOnlyAmbiguous(bottomLeft, currentTimeS);
                    if (pheromone == 0.5) {
                        return pheromone;
                    } else {
                        pheromones[6] = pheromone;
                    }
                }
            }
            //So don't check if current quadrant in the NORTH WEST (top left)
            if (current_quadrant == -1 || current_quadrant == 1 || current_quadrant == 2 || current_quadrant == 3) {
                //See if coordinate to the bottom right is in the quadtree and get its occupancy
                Coordinate bottomRight = Coordinate{box.getCenter().x + box.size, box.getCenter().y - box.size};
                if (mBox.contains(bottomRight)) {
//                argos::LOG << "bottomRight: " << bottomRight.x << " " << bottomRight.y << std::endl;
                    double pheromone = getPheromoneFromCoordinateOnlyAmbiguous(bottomRight, currentTimeS);
                    if (pheromone == 0.5) {
                        return pheromone;
                    } else {
                        pheromones[7] = pheromone;
                    }
                }
            }

            int minDiffIndex;
            double minDiff = 1;
            for (int i = 0; i < 8; i++) {
                auto pheromone = pheromones[i];
                if (pheromone != -1) {
                    auto diff = std::abs(pheromone - 0.5);
                    if (diff < minDiff) {
                        minDiff = diff;
                        minDiffIndex = i;
                    }
                }
            }
            if (minDiff < 1) {
                return pheromones[minDiffIndex];
            }

            return -1;

        }

        /**
         * Returns QuadNodes that intersect with or are contained by the given box
         * @param box
         * @param occupancy
         * @return
         */

        std::vector<QuadNode> query(const Box &box, Occupancy occupancy) const {
            auto queried_values = std::vector<QuadNode>();
            query(mRoot.get(), mBox, box, queried_values, occupancy);
            return queried_values;
        }

        /**
         * Returns all the values that intersect with or are contained by given box
         * @param box
         * @param occupancy
         * @return
         */
        std::vector<Box> queryBoxes(const Box &box, std::vector<Occupancy> occupancies, double currentTimeS) {
            auto boxes = std::vector<Box>();
            //While querying we also check if pheromones are expired, we remember those and remove them after.
            std::vector<QuadNode> values_to_be_removed = {};
            queryBoxes(mRoot.get(), mBox, box, boxes, occupancies, currentTimeS, values_to_be_removed);
//            for (auto &value: values_to_be_removed) {
//                remove(value);
//            }
            return boxes;
        }

        /**
          * Returns all the values that intersect with or are contained by given box
          * @param box
          * @param occupancy
          * @return
          */
        std::vector<std::pair<Box, double>> queryBoxesAndPheromones(const Box &box, std::vector<Occupancy> occupancies, double currentTimeS) {
            auto boxesAndPheromones = std::vector<std::pair<Box, double>>();
            //While querying we also check if pheromones are expired, we remember those and remove them after.
            std::vector<QuadNode> values_to_be_removed = {};
            queryBoxesAndPheromones(mRoot.get(), mBox, box, boxesAndPheromones, occupancies, currentTimeS, values_to_be_removed);
//            for (auto &value: values_to_be_removed) {
//                remove(value);
//            }
            return boxesAndPheromones;
        }


        [[nodiscard]] std::pair<Cell *, Box> getCellandBoxFromCoordinate(Coordinate coordinate) const {
            return getCellandBoxFromCoordinate(mRoot.get(), mBox, coordinate);
        }

            /**
             * Returns the Occupancy of the QuadNode containing the coordinate
             * @param coordinate
             */
        Occupancy getOccupancyFromCoordinate(Coordinate coordinate) const {
            QuadNode quadNode = getQuadNodeFromCoordinate(mRoot.get(), mBox, coordinate);
            return quadNode.occupancy;
        }

        /**
         * Returns the LConfidence of the QuadNode containing the coordinate
         * @param coordinate
         */
        float getConfidenceFromCoordinate(Coordinate coordinate) const {
            QuadNode quadNode = getQuadNodeFromCoordinate(mRoot.get(), mBox, coordinate);
            return quadNode.LConfidence;
        }

        /**
         * Returns if the QuadNode containing the coordinate has occupancy UNKNOWN or AMBIGUOUS
         * @param coordinate
         */
        bool isCoordinateUnknownOrAmbiguous(Coordinate coordinate) const {
            QuadNode quadNode = getQuadNodeFromCoordinate(mRoot.get(), mBox, coordinate);
            return (quadNode.occupancy == UNKNOWN || quadNode.occupancy == AMBIGUOUS);
        }

        /**
         * Return the calculated pheromone of the QuadNode containing the coordinate
         * @param coordinate
         */
        double getPheromoneFromCoordinateOnlyAmbiguous(Coordinate coordinate, double currentTimeS) const {
            QuadNode quadNode = getQuadNodeFromCoordinate(mRoot.get(), mBox, coordinate);
            if (quadNode.occupancy == UNKNOWN) return 0.5; //Unknown is ambiguous, and time factor will be 0
            else if (quadNode.occupancy == AMBIGUOUS) {
                double pheromone = calculatePheromone(quadNode.visitedAtS, P(quadNode.LConfidence), currentTimeS);
                return pheromone;
            }
            return -1;
        }

        /**
       * Return the calculated pheromone of the QuadNode containing the coordinate
       * @param coordinate
       */
        double getPheromoneFromCoordinate(Coordinate coordinate, double currentTimeS) const {
            QuadNode quadNode = getQuadNodeFromCoordinate(mRoot.get(), mBox, coordinate);
            if (quadNode.occupancy == UNKNOWN) return 0.5; //Unknown is ambiguous, and time factor will be 0
            else if (quadNode.occupancy != ANY) {
                double pheromone = calculatePheromone(quadNode.visitedAtS, P(quadNode.LConfidence), currentTimeS);
                return pheromone;
            }
            return -1;
        }

        /**
         * @brief Find all intersections between values stored in the quadtree
         * @return
         */
        std::vector<std::pair<QuadNode, QuadNode>> findAllIntersections() const {
            auto intersections = std::vector<std::pair<QuadNode, QuadNode>>();
            findAllIntersections(mRoot.get(), intersections);
            return intersections;
        }

        Box getBox() const {
            return mBox;
        }

//        void updateConfidence(Coordinate coordinate, double areaSize, double Preading, double currentTimeS) {
//            Box functionSpace = Box(Coordinate{coordinate.x - areaSize / 2.0, coordinate.y + areaSize / 2.0}, areaSize);
//            updateConfidence(mRoot.get(), mBox, functionSpace, Preading, currentTimeS);
//        }


        /**
         * @brief Export the quadtree to a file
         * @param filename
         */
        void exportQuadtreeToFile(const std::string &filename) {
            std::ofstream file(filename + ".txt");
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << filename << std::endl;
                return;
            }

            std::function<void(const Cell *, const Box &)> traverse;
            traverse = [&](const Cell *cell, const Box &box) {
                if (cell == nullptr) return;

                file << box.left << " " << box.top << " " << box.size << " " << "\n";

                // Write the bounding box, it's size, and occupancy of this cell to the file
                auto topLeft = box.getTopLeft();
                auto size = box.getSize();
                file << box.left << " " << box.top << " " << box.size << " " << cell->quadNode.occupancy << " " << "\n";

                // Traverse the children
                for (int i = 0; i < 4; ++i) {
                    if (cell->children.at(i)) {
                        traverse(cell->children.at(i).get(), computeBox(box, i));
                    }
                }
            };

            traverse(mRoot.get(), mBox);
            file.close();
        }

        /**
         * @brief Create a vector of strings from the quadtree
         */
        void toStringVector(std::vector<std::string> *strings, double after_time_s) {
            std::function<void(const Cell *, const Box &, int &, std::string &)> traverse;
            std::string grouped_message = "";
            grouped_message.clear();
            int counter = 0;

            traverse = [&](const Cell *cell, const Box &box, int &counter, std::string &grouped_message) {
                if (cell == nullptr) return;

                bool allSameOccupancy = false;

                // Write the bounding box, it's size, and occupancy of this cell to the file
                // If the occupancy is ANY, we don't need to store it, as the children will have new info
                // If the occupancy is UNKNOWN, we don't need to store it, as a child not existing will also yield in an UNKNOWN
//                    if (value.occupancy == ANY || value.occupancy == UNKNOWN)
//                        continue;
                // If the occupancy is OCCUPIED or FREE or AMBIGUOUS, we want to exchange that information. And we don't have to send any children as they will be all the same.
                if (cell->quadNode.occupancy != ANY && cell->quadNode.occupancy != UNKNOWN) {
                    allSameOccupancy = true;
                    if (cell->quadNode.visitedAtS > after_time_s) { //Only send nodes that are updated after the given time
                        std::string str =
                                std::to_string(box.getCenter().x) + ';' + std::to_string(box.getCenter().y) + ':' +
                                std::to_string(cell->quadNode.LConfidence) + '@' +
                                std::to_string(cell->quadNode.visitedAtS);

                        //Group every numberOfNodesPerMessage nodes
                        grouped_message.append(str);

                        if (counter == this->numberOfNodesPerMessage - 1) {
                            strings->emplace_back(grouped_message);
                            grouped_message.clear();
                            counter = 0;
                        } else {
                            grouped_message.append("|");
                            counter++;
                        }
                    }
                }

                // If all children have the same occupancy, we don't need to send the children, as they will all have the same occupancy.
                if (!allSameOccupancy) {
                    // Traverse the children
                    for (int i = 0; i < 4; i++) {
                        if (cell->children.at(i)) {
                            traverse(cell->children.at(i).get(), computeBox(box, i), counter, grouped_message);
                        }
                    }
                }
            };

            traverse(mRoot.get(), mBox, counter, grouped_message);
            //If there is an incomplete group, also send it.
            if (!grouped_message.empty()) {
                grouped_message.pop_back(); //Delete the last delimiter
                strings->emplace_back(grouped_message);
            }
        }

        /**
         * Get all the boxes in the quadtree
         * @return
         */
        std::vector<std::tuple<Box, double>> getAllBoxes(double currentTimeS) {
            std::vector<std::tuple<Box, double>> boxesAndPheromones = {};
            std::function<void(const Cell *, const Box &, std::vector<std::tuple<Box, double>> *)> traverse;
            traverse = [&](const Cell *cell, const Box &box,
                           std::vector<std::tuple<Box, double>> *boxesAndConfidenceAndTicks) {
                if (cell == nullptr) return;
                bool allSameOccupancy = false;
//                    if (value.occupancy == ANY || value.occupancy == UNKNOWN)
//                        continue;
                // If the occupancy is OCCUPIED or FREE or AMBIGUOUS, we want to exchange that information. And we don't have to send any children as they will be all the same.
                if (cell->quadNode.occupancy != ANY && cell->quadNode.occupancy != UNKNOWN) {
                    allSameOccupancy = true;
                    auto pheromone = calculatePheromone(cell->quadNode.visitedAtS, P(cell->quadNode.LConfidence), currentTimeS);
                    boxesAndConfidenceAndTicks->emplace_back(
                            box, pheromone);
                }

                // If all children have the same occupancy, we don't need to send the children, as they will all have the same occupancy.
                if (!allSameOccupancy) {
                    for (int i = 0; i < 4; i++) {
                        if (cell->children.at(i)) {
                            traverse(cell->children.at(i).get(), computeBox(box, i), boxesAndConfidenceAndTicks);
                        }
                    }
                }
            };

            traverse(mRoot.get(), mBox, &boxesAndPheromones);

            return boxesAndPheromones;

        }

        /**
         * Get all the boxes in the quadtree
         * @return
         */
        std::vector<std::tuple<Coordinate, Coordinate>> getAllNeighborPairs() {
            std::vector<std::tuple<Coordinate, Coordinate>> pairs = {};
            std::function<void(const Cell *, const Box &, std::vector<std::tuple<Coordinate, Coordinate>> *)> traverse;
            traverse = [&](const Cell *cell, const Box &box,
                           std::vector<std::tuple<Coordinate, Coordinate>> *pairs) {
                if (cell == nullptr) return;
                bool allSameOccupancy = false;
//                    if (value.occupancy == ANY || value.occupancy == UNKNOWN)
//                        continue;
                // If the occupancy is OCCUPIED or FREE or AMBIGUOUS, we want to exchange that information. And we don't have to send any children as they will be all the same.
                if (cell->quadNode.occupancy != ANY && cell->quadNode.occupancy != UNKNOWN) {
                    allSameOccupancy = true;
                    for (auto neighbor: cell->neighbors) {
                        if (neighbor == nullptr) continue;
                        pairs->emplace_back(cell->quadNode.coordinate, neighbor->quadNode.coordinate);
                    }
                }

                // If all children have the same occupancy, we don't need to send the children, as they will all have the same occupancy.
                if (!allSameOccupancy) {
                    for (int i = 0; i < 4; i++) {
                        if (cell->children.at(i)) {
                            traverse(cell->children.at(i).get(), computeBox(box, i), pairs);
                        }
                    }
                }
            };

            traverse(this->mRoot.get(), this->mBox, &pairs);


            return pairs;

        }

        double getResolution() {
            return this->RESOLUTION;
        }

        void setResolution(double minSize) {
            this->RESOLUTION = minSize;
        }

        Box getRootBox() const {
            return mBox;
        }


    private:
        static constexpr auto Threshold = std::size_t(16);
        double RESOLUTION;
        double EVAPORATION_TIME_S;
        double EVAPORATED_PHEROMONE_FACTOR;
        double MERGE_MAX_VISITED_TIME_DIFF;
        double MERGE_MAX_P_CONFIDENCE_DIFF;
        double l_min = -3.5;
        double l_max = 2;
        double L_FREE_THRESHOLD;
        double P_FREE_THRESHOLD;
//        float l_occupied = -0.41; //P=0.4
        double P_OCCUPIED_THRESHOLD;
        float ALPHA_RECEIVE; //How much the received value should be weighted



        Box mBox;
        std::unique_ptr<Cell> mRoot;

        /**
         * @brief Check if the given node is a leaf i.e. had no children
         * @param node
         * @return
         */
        bool isLeaf(const Cell *node) const {
            return !static_cast<bool>(node->children.at(0));


        }

        /**
         * @brief Compute the box of the i-th child of the given box
         * @param box
         * @param i
         * @return
         */
        Box computeBox(const Box &box, int i) const {
            auto origin = box.getTopLeft();
            auto childSize = box.getSize() / 2.0;
            switch (i) {
                // North West
                case 0:
                    return Box(origin, childSize);
                    // Norst East
                case 1:
                    return Box(Coordinate{origin.x + childSize, origin.y}, childSize);
                    // South West
                case 2:
                    return Box(Coordinate{origin.x, origin.y - childSize}, childSize);
                    // South East
                case 3:
                    return Box(Coordinate{origin.x + childSize, origin.y - childSize}, childSize);
                default:
                    assert(false && "Invalid child index");
                    return Box();
            }
        }

        /**
         * @brief Get the quadrant which the given box should go in in the node box
         * @param nodeBox
         * @param valueBox
         * @return
         */
        int getQuadrant(const Box &nodeBox, const Box &valueBox) const {
            auto center = nodeBox.getCenter();

            // West
            if (valueBox.getRight() < center.x) {
                // North West
                if (valueBox.getBottom() > center.y)
                    return 0;
                    // South West
                else if (valueBox.top <= center.y)
                    return 2;
                    // Not contained in any quadrant
                else
                    return -1;
            }
                // East
            else if (valueBox.left >= center.x) {
                // North East
                if (valueBox.getBottom() > center.y)
                    return 1;
                    // South East
                else if (valueBox.top <= center.y)
                    return 3;
                    // Not contained in any quadrant
                else
                    return -1;
            }
                // Not contained in any quadrant
            else
                return -1;
        }

        /**
         * @brief Get the quadrant which the given coordinate should go in in the node box
         * @param nodeBox
         * @param valueCoordinate
         * @return
         */
        int getQuadrant(const Box &nodeBox, const Coordinate &valueCoordinate) const {
            auto center = nodeBox.getCenter();

            //If the value is the same as the center, it is not contained in any quadrant
            if (center == valueCoordinate)
                return 4;

            // West
            if (valueCoordinate.x < center.x) {
                // North West
                if (valueCoordinate.y > center.y)
                    return 0;
                    // South West
                else if (valueCoordinate.y <= center.y)
                    return 2;
                    // Not contained in any quadrant
                else
                    return -1;
            }
                // East
            else if (valueCoordinate.x >= center.x) {
                // North East
                if (valueCoordinate.y > center.y)
                    return 1;
                    // South East
                else if (valueCoordinate.y <= center.y)
                    return 3;
                    // Not contained in any quadrant
                else
                    return -1;
            }
                // Not contained in any quadrant
            else
                return -1;
        }

        /**
         * @brief Add a value to the quadtree
         * @param cell
         * @param box
         * @param value
         */
        Box add(Cell *cell, const Box &box, const QuadNode &value, double currentTimeS, bool ownObservation) {
            assert(cell != nullptr);
            if (!box.contains(value.coordinate)) { //TODO: If value added outside box, expand box.
                return Box();
            }
            assert(box.contains(value.coordinate));

            assert(value.occupancy != ANY && "Added occupancy should not be ANY");
            assert(value.occupancy != UNKNOWN && "Added occupancy should not be UNKNOWN");
            Box returnBox = Box{MAXFLOAT, MAXFLOAT, 0};
            if (isLeaf(cell)) {
                // Insert the value in this cell if possible

                //If the box size is the minimum size we allow (corresponding to finest mapping level),
                // then we only contain a single QuadNode. Update the occupancy of this cell to the most important occupancy.
                if (std::abs(box.size - RESOLUTION) < 1e-6) { //If the box is the smallest resolution (account for floating point / rounding errors)
                    QuadNode newNode = QuadNode();
//                    newNode.coordinate = value.coordinate;
                    newNode.coordinate = box.getCenter();
                    if (cell->quadNode.visitedAtS == -1) { //If the cell is empty
                        newNode.occupancy = value.occupancy;
                        newNode.visitedAtS = value.visitedAtS;
                        newNode.LConfidence = value.LConfidence;
                        this->numberOfLeafNodes++;
                    } else {
                        //OCCUPIED always takes precedence over FREE
                        //Update with the most precedent or up-to-date information.
//                        if (cell->quadNode.occupancy == OCCUPIED && value.occupancy == OCCUPIED) {
//                            newNode.occupancy = OCCUPIED;
//                            newNode.visitedAtS = std::max(cell->quadNode.visitedAtS, value.visitedAtS);
//                        } else if (cell->quadNode.occupancy == OCCUPIED) {
//                            newNode.occupancy = OCCUPIED;
//                            newNode.visitedAtS = cell->quadNode.visitedAtS;
//                        } else if (value.occupancy == OCCUPIED) {
//                            newNode.occupancy = OCCUPIED;
//                            newNode.visitedAtS = value.visitedAtS;
//                        } else if (cell->quadNode.occupancy == FREE && value.occupancy == FREE) {
//                            newNode.occupancy = FREE;
//                            newNode.visitedAtS = std::max(cell->quadNode.visitedAtS, value.visitedAtS);
//                        } else if (cell->quadNode.occupancy == FREE) {
//                            newNode.occupancy = FREE;
//                            newNode.visitedAtS = cell->quadNode.visitedAtS;
//                        } else if (value.occupancy == FREE) {
//                            newNode.occupancy = FREE;
//                            newNode.visitedAtS = value.visitedAtS;
//                        } else {
//                            assert(-1 &&
//                                   "Shouldn't get here, as neither current cell or added cell are OCCUPIED or FREE");
//                        }
                        if (value.visitedAtS > cell->quadNode.visitedAtS) { //Only combine if the newly received value is more recent
                            if (ownObservation) {
                                //First calculate old pheromone, and use that to calculate new LConfidence. We do this to incorporate decay when adding.
                                double  oldPheromone = calculatePheromone(cell->quadNode.visitedAtS, P(cell->quadNode.LConfidence), currentTimeS);
                                newNode.LConfidence = calculateOccupancyProbability(value.LConfidence, oldPheromone);
                                newNode.visitedAtS = value.visitedAtS; //Is current time since observations are immediately updated
                            } else {
//                                auto valuePheromone = calculatePheromone(value.visitedAtS, P(value.LConfidence), currentTimeS);
//                                auto cellPheromone = calculatePheromone(cell->quadNode.visitedAtS, P(cell->quadNode.LConfidence),
//                                                                        currentTimeS);
//                                auto weightedAveragePheromone = ALPHA_RECEIVE * valuePheromone + (1 - ALPHA_RECEIVE) * cellPheromone;
//                                newNode.LConfidence = L(weightedAveragePheromone);
                                //First calculate old pheromone, and use that to calculate new LConfidence. We do this to incorporate decay when adding.
                                auto P_value = P(value.LConfidence);
                                auto valuePheromone = calculatePheromone(value.visitedAtS, P_value, currentTimeS);
                                auto alpha_compensated = valuePheromone > 0.5 ? (valuePheromone - 0.5) * ALPHA_RECEIVE + 0.5 : 0.5 - (0.5-valuePheromone) * (ALPHA_RECEIVE);
                                double  oldPheromone = calculatePheromone(cell->quadNode.visitedAtS, P(cell->quadNode.LConfidence), currentTimeS);
                                auto combinedL = calculateOccupancyProbability(L(oldPheromone), alpha_compensated);
                                auto combinedP = P(combinedL);
                                newNode.visitedAtS = std::max(cell->quadNode.visitedAtS, value.visitedAtS);
                                //Reverse calculate LConfidence from combined P, so we have the correct pheromone value at the set visited time.
                                auto Pconfidenceval = reversePheromone(newNode.visitedAtS, combinedP, currentTimeS);
                                //Cap the LConfidence value between 0 and 1, as it is a probability.
                                newNode.LConfidence = L(std::min(std::max((Pconfidenceval), 0.000001), 0.999999));

                            }
                            double pheromone = calculatePheromone(newNode.visitedAtS, P(newNode.LConfidence),
                                                                  currentTimeS);
                            Occupancy occ = AMBIGUOUS;
                            if (pheromone >= P_FREE_THRESHOLD) {
                                occ = FREE;
                            } else if (pheromone <= P_OCCUPIED_THRESHOLD) { //Most probably occupied
                                occ = OCCUPIED;
                            }
                            newNode.occupancy = occ;
                        } else {
                            newNode = cell->quadNode;
                        }
                    }

                    assert(newNode.occupancy == FREE ||
                           newNode.occupancy == OCCUPIED ||
                           newNode.occupancy == AMBIGUOUS && "new cell occupancy should be FREE or OCCUPIED or AMBIGUOUS");
                    // Make the only value the 'merged cell'
                    cell->quadNode = newNode;
                    if (cell->quadNode.occupancy == OCCUPIED) { //Occupied
                        //Set neighbors
                        cell->add_occupied_neighbors(box.size);
                    } else {
                        //Remove neighbors; Make sure to remove neighbors if the cell is not occupied, or we will have pointer issues.
                        cell->remove_neighbors();
                    }
                    returnBox = box;


                }
                    // Otherwise, we split and we try again
                else {
//                    //If the to be added occupancy is the same as the parent, and the visited time is not too far apart, we can skip adding.
                    // WRONG: should always add, as we are adding log like
////                    float pv = P(value.LConfidence);
////                    float pc = P(cell->quadNode.LConfidence);
//                    double pv = calculatePheromone(value.visitedAtS, P(value.LConfidence), currentTimeS);
//                    double pc = calculatePheromone(cell->quadNode.visitedAtS, P(cell->quadNode.LConfidence), currentTimeS);
//                    if (cell->quadNode.visitedAtS == -1 || !(std::abs(pv - pc) <= MERGE_MAX_P_CONFIDENCE_DIFF
////                                                            &&
////                                                             value.visitedAtS - cell->quadNode.visitedAtS <=
////                                                             MERGE_MAX_VISITED_TIME_DIFF
//                                                             )) {
                    split(cell, box, currentTimeS);
                    returnBox = add(cell, box, value, currentTimeS, ownObservation);
//                    }
                }
            } else {
                // If the cell is not a leaf
                // And if the box center is the same as the value coordinate (meaning this value information is the same for all children of this cell),
                // then we only contain a single QuadNode. Update the occupancy of this cell to the most important occupancy.
                // This should only happen when adding nodes received from other agents.
                //Round to 5 decimals to counter precision lost in messages
                double roundFactor = 100000.0; //Round to 5 decimals, should be enough for realistic resolution values
                if (std::round(box.getCenter().x*roundFactor) == std::round(value.coordinate.x* roundFactor) && std::round(box.getCenter().y*roundFactor) == std::round(value.coordinate.y* roundFactor) && !ownObservation) {

                    QuadNode newNode = QuadNode();
                    newNode.coordinate = value.coordinate;
                    if (cell->quadNode.visitedAtS == -1) {
                        newNode.occupancy = value.occupancy;
                        newNode.visitedAtS = value.visitedAtS;
                        newNode.LConfidence = value.LConfidence;
                        this->numberOfLeafNodes++;
                    } else {
                        //If cell has occupancy FREE or OCCUPIED, it entails all its children are also of that value, so we just update this parent.
//                        if (cell->quadNode.occupancy == FREE) {
//                            if (value.occupancy == OCCUPIED) {
//                                newNode.occupancy = OCCUPIED;
//                                newNode.visitedAtS = value.visitedAtS;
//                            } else {
//                                newNode.occupancy = FREE;
//                                newNode.visitedAtS = std::max(cell->quadNode.visitedAtS, value.visitedAtS);
//                            }
//                            cell->quadNode = newNode;
//                            returnBox = box;
//                        } else if (cell->quadNode.occupancy == OCCUPIED) {
//                            if (value.occupancy == FREE) {
//                                newNode.occupancy = OCCUPIED;
//                                newNode.visitedAtS = cell->quadNode.visitedAtS;
//                            } else {
//                                newNode.occupancy = OCCUPIED;
//                                newNode.visitedAtS = std::max(cell->quadNode.visitedAtS, value.visitedAtS);
//                            }
//                            cell->quadNode = newNode;
//                            returnBox = box;
                        //I don't think we ever enter this case, as a non-leaf node will have occupancy ANY.
                        if (cell->quadNode.occupancy == FREE || cell->quadNode.occupancy == OCCUPIED || cell->quadNode.occupancy == AMBIGUOUS) {
                            if (value.visitedAtS > cell->quadNode.visitedAtS) { //Only combine if the newly received value is more recent
                                double  oldPheromone = calculatePheromone(cell->quadNode.visitedAtS, P(cell->quadNode.LConfidence), currentTimeS);
                                auto valuePheromone = calculatePheromone(value.visitedAtS, P(value.LConfidence), currentTimeS);

                                auto combinedL = calculateOccupancyProbability(L(valuePheromone), oldPheromone); //Calculate the new confidence, use old pheromone value so we take into account time decay
                                newNode.visitedAtS = std::max(value.visitedAtS, cell->quadNode.visitedAtS);
                                auto PConfidenceval = reversePheromone(newNode.visitedAtS, P(combinedL), currentTimeS);
                                //Cap the confidence value between 0.000001 and 0.999999, as the L function is not defined outside of this range.
                                newNode.LConfidence = L(std::min(std::max((PConfidenceval), 0.000001), 0.999999));

                                double pheromone = calculatePheromone(newNode.visitedAtS, P(newNode.LConfidence),
                                                                      currentTimeS);
                                Occupancy occ = AMBIGUOUS;
                                if (pheromone >= P_FREE_THRESHOLD) {
                                    occ = FREE;
                                } else if (pheromone <= P_OCCUPIED_THRESHOLD) {
                                    occ = OCCUPIED;
                                }
                                newNode.occupancy = occ;
                                cell->quadNode = newNode;
                            }
                            //Else if cell has occupancy ANY or UNKNOWN, we add to the children
                        } else {
                            //When adding a new coordinate and occupancy to that parent, we should set the remaining (yet unset) children to the value occupancy.
                            //What will happen:
                            //Children without a value will be set with information entailed in the received value
                            //Children with a value will compare the occupancy and visited times for precedence and most recent and update accordingly.
                            //If after updating a child, all children have the same occupancy and visited times are within range, the parent will entail the details and the children will be deleted.
                            for (int i = 0; i < cell->children.size(); i++) {
                                Box childBox = computeBox(box, i);
                                Coordinate childBoxCenter = childBox.getCenter();

                                // For each child, create a cell with corresponding center coordinate
                                QuadNode newChildNode = QuadNode();
                                newChildNode.visitedAtS = value.visitedAtS;
                                newChildNode.coordinate = childBoxCenter;
                                newChildNode.occupancy = value.occupancy;
                                newChildNode.LConfidence = value.LConfidence;

                                //Add to current cell, so that it will be placed in the proper child and checked for optimization later.
                                returnBox = add(cell, box, newChildNode, currentTimeS, ownObservation);

                            }
                        }

                    }
//
                    // Else we add the value to the appropriate child
                } else {
                    auto i = box.getQuadrant(value.coordinate);
                    // Add the value in a child if the value is entirely contained in it
                    assert(i != -1 && "A value should be contained in a quadrant");
                    assert(i != 4 && "A value should not be the same as the center of the box");
                    returnBox = add(cell->children.at(static_cast<std::size_t>(i)).get(), computeBox(box, i), value, currentTimeS, ownObservation);
//                Check if all children have the same occupancy
//                    Occupancy firstOccupancy = UNKNOWN;
                    bool confidencesTooFarApart = false;
                    double minConfidence = MAXFLOAT;
                    double maxConfidence = -1;
                    bool visitedTimesTooFarApart = false;
                    double minVisitedTime = MAXFLOAT;
                    double maxVisitedTime = -1;

                    std::vector<double> children_pheromones;
                    std::vector<double> children_l_confidences;

                    if (cell->children.at(0)->quadNode.visitedAtS == -1)
                        confidencesTooFarApart = true; //If the first child is empty, it is not the same occupancy as the others
                    else {
                        //Get the occupancy of the first child (if it is empty, it is not the same occupancy as the others)
//                        firstOccupancy = cell->children.at(0)->quadNode.occupancy;
                        for (const auto &child: cell->children) {

                            //If a child is empty, it is not the same occupancy as the others
                            //If a child has a different occupancy than the first child, it is not the same occupancy as all others
                            //If a child has occupancy ANY, further nested nodes will have a different occupancy.
                            if (child->quadNode.visitedAtS == -1 || child->quadNode.occupancy == ANY) {
                                confidencesTooFarApart = true;
                                break;
                            }

                            //We can just look at the pheromones, as the evaporation curves follow the same shape, even when they have different stargint points.
                            //This means merging them will not change the shape of the curve, and thus the evaporation will be the same.

                            //If the confidence of the child is too far apart from the first child, it is not the same occupancy as the others
                            auto pheromone = calculatePheromone(child->quadNode.visitedAtS, P(child->quadNode.LConfidence), currentTimeS);
                            children_pheromones.push_back(pheromone);
                            children_l_confidences.push_back(child->quadNode.LConfidence);
                            if (pheromone < minConfidence)
                                minConfidence = pheromone;
                            if (pheromone > maxConfidence)
                                maxConfidence = pheromone;

//                            //If the visited time of the child is too far apart from the first child, it is not the same occupancy as the others
                            if (child->quadNode.visitedAtS < minVisitedTime)
                                minVisitedTime = child->quadNode.visitedAtS;
                            if (child->quadNode.visitedAtS > maxVisitedTime)
                                maxVisitedTime = child->quadNode.visitedAtS;
                        }
                        //If one of the children is occupied, all children should be kept
                        //If the occupancies are too far apart, the children should be kept
                        // Only merge free cells if the confidence is not too far apart
                        // Don't merge occupied cells, as we need them separated for the neighbors
                        //Ambiguous cells can become free or occupied, so we don't merge them yet, as they are important regions.
                        if (minConfidence < P_OCCUPIED_THRESHOLD || maxConfidence - minConfidence > MERGE_MAX_P_CONFIDENCE_DIFF)
                            confidencesTooFarApart = true;
//                        //If the visited times are too far apart, the children should be kept
//                        if (maxVisitedTime - minVisitedTime > MERGE_MAX_VISITED_TIME_DIFF)
//                            visitedTimesTooFarApart = true;
                    }
//                assert(!cell->quadNode.visitedAtS == -1 && "A non-leaf cell should have a value");

                    // If all children have the same occupancy, and their visited times are not too far apart, we can delete the children and the parents will have all info
                    if (!confidencesTooFarApart && !visitedTimesTooFarApart) {
                        //Unitialize the children nodes as the parent now contains their information.
                        double PheromoneSum = 0.0;

                        for (auto &pheromone: children_pheromones) {
                            PheromoneSum += pheromone;
                        }

                        auto averagePheromone = PheromoneSum / 4.0;
                        auto PConfidenceAtTimeAndPheromone = reversePheromone(maxVisitedTime, averagePheromone, currentTimeS);
                        //Cap at l_max and l_min. Shouldn't be necessary since it is average and any child's pheromone should be within bounds.
                        cell->quadNode.LConfidence = std::max(std::min(L(PConfidenceAtTimeAndPheromone), l_max), l_min);
//                        cell->quadNode.LConfidence = (L(PConfidenceAtTimeAndPheromone));
//                        double pheromone = calculatePheromone(maxVisitedTime, P(cell->quadNode.LConfidence), currentTimeS);
                        for (auto &child: cell->children) {
//                            PConfidenceSum += P(child->quadNode.LConfidence);
                            child.reset();
                        }
                        numberOfLeafNodes -= 3; //We remove 4 children and the parent is now a leaf node.
                        assert(isLeaf(cell) && "The cell should be a leaf again now");
                        //If all children have the same occupancy, give the parent that occupancy
                        Occupancy occ = AMBIGUOUS;
                        if (averagePheromone >= P_FREE_THRESHOLD){
                            occ = FREE;
                        } else if (averagePheromone <= P_OCCUPIED_THRESHOLD){
                            occ = OCCUPIED;
                        }
                        cell->quadNode.occupancy = occ;
                        assert(cell->quadNode.occupancy != UNKNOWN && cell->quadNode.occupancy != ANY &&
                               "A non-leaf cell should not have UNKNOWN or ANY occupancy at this point");
                        cell->quadNode.visitedAtS = maxVisitedTime;


                    } else {
                        //If the children have different occupancies, or the visited times are too far apart, the parent should have occupancy ANY
                        if(cell->quadNode.occupancy != ANY && cell->quadNode.occupancy != UNKNOWN) this->numberOfLeafNodes--; //We are changing it to ANY, so this is not a leafnode anymore
                        cell->quadNode.occupancy = ANY;
                        cell->quadNode.LConfidence = 0.0; //Ambiguous

                    }
                }

            }
            return returnBox;
        }

        /**
         * @brief Split a leaf cell into four children
         * @param cell
         * @param box
         */
        void split(Cell *cell, const Box &box, double currentTimeS) {
            assert(cell != nullptr);
            assert(isLeaf(cell) && "Only leaves can be split");
            // Create children
            for (auto &child: cell->children){
                child = std::make_unique<Cell>(this->numberOfCells);
                child->parent = cell;
            }

            assert(!isLeaf(cell) && "A cell should not be a leaf after splitting");

            //If cell has occupancy FREE or OCCUPIED or AMBIGUOUS, it entails all its children are also of that value.
            //When adding a new coordinate and occupancy to that parent, we should set the three remaining children to the parent occupancy.
            //So when splitting we set all children to the parent occupancy and visited time.
            if (cell->quadNode.visitedAtS != -1 &&
                (cell->quadNode.occupancy != ANY && cell->quadNode.occupancy != UNKNOWN)) {
                for (int i = 0; i < cell->children.size(); i++) {
                    Box childBox = computeBox(box, i);
                    Coordinate childBoxCenter = childBox.getCenter();

                    auto quadNode = QuadNode{childBoxCenter, cell->quadNode.occupancy,
                                             cell->quadNode.visitedAtS, cell->quadNode.LConfidence};
                    auto childrenPConfidence = P(cell->quadNode.LConfidence);
                    add(cell->children.at(static_cast<std::size_t>(i)).get(), childBox, quadNode, currentTimeS, false);
                }
                this->numberOfLeafNodes--;
            }
            cell->quadNode = QuadNode{box.getCenter(), ANY, 0, 0.0};
        }

        /**
         * @brief Remove a value from the quadtree
         * @param node
         * @param box
         * @param value
         * @return
         */
        void remove(Cell *node, const Box &box, const QuadNode &value) {
            assert(node != nullptr);
            assert(box.contains(value.coordinate));
            if (isLeaf(node)) {
                // Remove the value from node
                removeValue(node, value);
                this->numberOfLeafNodes--;
            } else {
                // Remove the value in a child if the value is entirely contained in it
                auto i = box.getQuadrant(value.coordinate);
                if (i ==
                    4) { // If the value is the same as the center of the box, we remove the value from the current node
                    removeValue(node, value);
                    this->numberOfLeafNodes--;
                    for (auto &child: node->children){
                        if(child->quadNode.visitedAtS != -1) this->numberOfLeafNodes--; //If the child is not empty, we are removing a node
                        child.reset();
                    }

                } else {
                    assert(i != -1);
                    remove(node->children.at(static_cast<std::size_t>(i)).get(), computeBox(box, i), value);
//                    node->children.at(static_cast<std::size_t>(i)].reset();
                    node->quadNode.occupancy = ANY;
                    // Check if there are any children, if not, remove the value from the parent
                    bool atLeastOneChildWithValue = false;
                    for (auto &child: node->children)
                        if (child->quadNode.visitedAtS != -1) atLeastOneChildWithValue = true;

                    if (!atLeastOneChildWithValue) {
                        removeValue(node, node->quadNode);
                        //Node count has been decreased for every child with a value already, and this cell is also removed (so not a leaf) so we don't need to decrease it here.
                        for (auto &child: node->children)
                            child.reset();
                        assert(isLeaf(node));
                    }

                }
            }
        }

        void removeValue(Cell *node, const QuadNode &value) const {
            assert(node->quadNode == value); //We can only remove a value from a node which has that value
            node->quadNode = QuadNode{Coordinate{0, 0}, UNKNOWN, -1}; //visitedAtS -1 means it is empty
        }

        /**
         * @brief Query the quadtree for QuadNodes that intersect with or are contained by the given box
         * @param node
         * @param box
         * @param queryBox
         * @param values
         * @param occupancy
         */
        void query(Cell *node, const Box &box, const Box &queryBox, std::vector<QuadNode> &values,
                   Occupancy occupancy) const {
            assert(node != nullptr);
            assert(queryBox.intersects_or_contains(box));
            if ((occupancy == ANY || node->quadNode.occupancy == occupancy) &&
                (queryBox.contains(node->quadNode.coordinate) || queryBox.intersects_or_contains(box)))
                values.push_back(node->quadNode);

            //Only check further if the occupancy of the non-leaf node is not all the same for its children, so UNKNOWN.
            if (!isLeaf(node) && (node->quadNode.visitedAtS == -1 || node->quadNode.occupancy == ANY ||
                                  node->quadNode.occupancy == UNKNOWN)) {
                for (auto i = std::size_t(0); i < node->children.size(); ++i) {
                    auto childBox = computeBox(box, static_cast<int>(i));
                    if (queryBox.intersects_or_contains(childBox))
                        query(node->children.at(i).get(), childBox, queryBox, values, occupancy);
                }
            }
        }

        /**
         * @brief Query the quadtree for boxes that intersect with or are contained by the given box that have a given occupancy
         * @param cell the current cell being looked in
         * @param box the box the current cell belongs in
         * @param queryBox the search space
         * @param boxes the list of boxes in the search space with the correct occupancy
         * @param occupancy the occupancy to look for
         * @param currentTimeS the current experiment time of the agent
         * @param values_to_be_removed the list of values that need to be removed from the quadtree as they are expired
         */
        void queryBoxes(Cell *cell, const Box &box, const Box &queryBox, std::vector<Box> &boxes,
                        std::vector<Occupancy> occupancies, double currentTimeS, std::vector<QuadNode> &values_to_be_removed) {
            assert(cell != nullptr);
            assert(queryBox.intersects_or_contains(box));
            //Check if pheromone is expired, if so, set the occupancy to unknown and remove it later
            if (cell->quadNode.occupancy != Occupancy::ANY && cell->quadNode.occupancy != UNKNOWN) {
                double pheromone = calculatePheromone(cell->quadNode.visitedAtS, P(cell->quadNode.LConfidence), currentTimeS);
                if (pheromone >= P_FREE_THRESHOLD) {
                    cell->quadNode.occupancy = Occupancy::FREE;
                } else if (pheromone <= P_OCCUPIED_THRESHOLD) {
                    cell->quadNode.occupancy = Occupancy::OCCUPIED;
                } else {
                    cell->quadNode.occupancy = Occupancy::AMBIGUOUS;
                }
                if (cell->quadNode.occupancy != OCCUPIED) cell->remove_neighbors();
//                cell->quadNode.occupancy = Occupancy::UNKNOWN;
                //Keep a list of the to be removed values, as we cant delete them now due to concurrency issues. These will be deleted after the querying is done.
//                values_to_be_removed.push_back(cell->quadNode);
            }


            if (std::find(occupancies.begin(), occupancies.end(), cell->quadNode.occupancy) != occupancies.end() &&
                (queryBox.contains(cell->quadNode.coordinate) || queryBox.intersects_or_contains(box)))
                boxes.push_back(box);


            //Only check further if the occupancy of the non-leaf cell is not all the same for its children, so ANY.
            if (!isLeaf(cell) && (cell->quadNode.visitedAtS == -1 || cell->quadNode.occupancy == ANY ||
                                  cell->quadNode.occupancy == UNKNOWN)) {
                for (int d = 0; d < cell->children.size(); d++) {
                    auto childBox = computeBox(box, static_cast<int>(d));
                    if (queryBox.intersects_or_contains(childBox)) {
                        queryBoxes(cell->children.at(d).get(), childBox, queryBox, boxes, occupancies, currentTimeS,
                                   values_to_be_removed);
                    }
                }
            }
        }

        /**
         * @brief Query the quadtree for boxesAndPheromones that intersect with or are contained by the given box that have a given occupancy, also return the pheromone value
         * @param cell the current cell being looked in
         * @param box the box the current cell belongs in
         * @param queryBox the search space
         * @param boxesAndPheromones the list of boxesAndPheromones in the search space with the correct occupancy
         * @param occupancy the occupancy to look for
         * @param currentTimeS the current experiment time of the agent
         * @param values_to_be_removed the list of values that need to be removed from the quadtree as they are expired
         */
        void queryBoxesAndPheromones(Cell *cell, const Box &box, const Box &queryBox, std::vector<std::pair<Box, double>> &boxesAndPheromones,
                        std::vector<Occupancy> occupancies, double currentTimeS, std::vector<QuadNode> &values_to_be_removed) {
            assert(cell != nullptr);
            assert(queryBox.intersects_or_contains(box));
            //Check if pheromone is expired, if so, set the occupancy to unknown and remove it later
            if (cell->quadNode.occupancy != Occupancy::ANY && cell->quadNode.occupancy != UNKNOWN) {
                double pheromone = calculatePheromone(cell->quadNode.visitedAtS, P(cell->quadNode.LConfidence), currentTimeS);
                if (pheromone >= P_FREE_THRESHOLD) {
                    cell->quadNode.occupancy = Occupancy::FREE;
                } else if (pheromone <= P_OCCUPIED_THRESHOLD) {
                    cell->quadNode.occupancy = Occupancy::OCCUPIED;
                } else {
                    cell->quadNode.occupancy = Occupancy::AMBIGUOUS;
                }
                if (cell->quadNode.occupancy != OCCUPIED) cell->remove_neighbors();
//                cell->quadNode.occupancy = Occupancy::UNKNOWN;
                //Keep a list of the to be removed values, as we cant delete them now due to concurrency issues. These will be deleted after the querying is done.
//                values_to_be_removed.push_back(cell->quadNode);
            }


            if (std::find(occupancies.begin(), occupancies.end(), cell->quadNode.occupancy) != occupancies.end() &&
                (queryBox.contains(cell->quadNode.coordinate) || queryBox.intersects_or_contains(box)))
                boxesAndPheromones.emplace_back(box, calculatePheromone(cell->quadNode.visitedAtS, P(cell->quadNode.LConfidence), currentTimeS));


            //Only check further if the occupancy of the non-leaf cell is not all the same for its children, so ANY.
            if (!isLeaf(cell) && (cell->quadNode.visitedAtS == -1 || cell->quadNode.occupancy == ANY ||
                                  cell->quadNode.occupancy == UNKNOWN)) {
                for (int d = 0; d < cell->children.size(); d++) {
                    auto childBox = computeBox(box, static_cast<int>(d));
                    if (queryBox.intersects_or_contains(childBox)) {
                        queryBoxesAndPheromones(cell->children.at(d).get(), childBox, queryBox, boxesAndPheromones, occupancies, currentTimeS,
                                                values_to_be_removed);
                    }
                }
            }
        }

        double calculatePheromone(double visitedTime, double PConfidence, double currentTime) const {
            //If the visited time is the same as the current time, the time factor will be 1, so the pheromone will be the same as the sensor confidence.
            //If the visited time is greater than the current time, an agent's steps went overtime or oscillator drift occurred as a cell cannot have a visited time that is in the future.
            // So we just take time factor 1 here.
            if (visitedTime >= currentTime) return PConfidence;
//            double timeProbability = 1.0 - std::min((currentTime - visitedTime) / EVAPORATION_TIME_S, (1.0 - EVAPORATED_PHEROMONE_FACTOR));
            double lambda = - std::log(this->EVAPORATED_PHEROMONE_FACTOR) / this->EVAPORATION_TIME_S; //Evaporate to EVAPORATED_PHEROMONE_FACTOR after EVAPORATION_TIME_S
            double timeProbability = exp(-lambda * (currentTime - visitedTime)); //Exponential decay
            double pheromone = timeProbability * (PConfidence - 0.5) + 0.5;
            //This makes sure that a value once set to occupied or free, will not be changed to ambiguous again due to evaporation.
            //So we assume that if a cell is occupied, it will stay that way, albeit with a lower confidence.
            //So the asymptote is P_OCCUPIED_THRESHOLD instead of 0.5;
            if (PConfidence <= P_OCCUPIED_THRESHOLD) pheromone = timeProbability * (PConfidence - P_OCCUPIED_THRESHOLD) + P_OCCUPIED_THRESHOLD;
            //But if a cell is free, it can become ambiguous again, as new obstacles can appear.
//            if (PConfidence >= P_FREE) pheromone = timeProbability * (PConfidence - P_FREE) + P_FREE;
            assert(pheromone >= 0.0 && pheromone <= 1.0 && "Pheromone should be between 0 and 1");
            return pheromone;
        }

        /**
         * Calculates p confidence from pheromone and time
         * @param visitedTime
         * @param pheromone
         * @param currentTime
         * @return
         */
        double reversePheromone(double visitedTime, double pheromone, double currentTime) const {
//            double timeProbability = 1.0 - std::min((currentTime - visitedTime) / EVAPORATION_TIME_S, (1.0 - EVAPORATED_PHEROMONE_FACTOR));
            double lambda = - std::log(this->EVAPORATED_PHEROMONE_FACTOR) / this->EVAPORATION_TIME_S; //Evaporate to EVAPORATED_PHEROMONE_FACTOR after EVAPORATION_TIME_S
            double timeDifference = currentTime - visitedTime;
//            double PConfidence = pheromone * exp(lambda*timeDifference);
            double PConfidence = (pheromone-0.5) * exp(lambda*timeDifference) + 0.5;
            return PConfidence;
        }

//        /**
//         * @brief Update the reachability confidence of the quadtree for QuadNodes that intersect with or are contained by the given box
//         * @param cell the current cell being looked in
//         * @param box the box the current cell belongs in
//         * @param queryBox the search space
//         * @param confidenceIncrease the increase in confidence
//         * @param currentTimeS the current experiment time of the agent
//         */
//        void updateConfidence(Cell *cell, const Box &box, const Box &functionSpace, float Pn_zt, double currentTimeS) {
//            assert(cell != nullptr);
//            assert(functionSpace.intersects_or_contains(box));
//
//            //Only update the confidence if we haven't seen this FREE cell in a while
//            if (cell->quadNode.occupancy != ANY && cell->quadNode.occupancy != UNKNOWN &&
////                    cell->quadNode.occupancy == FREE &&
//                (functionSpace.contains(cell->quadNode.coordinate) || functionSpace.intersects_or_contains(box))
////                &&
////                currentTimeS - cell->quadNode.visitedAtS > MERGE_MAX_VISITED_TIME_DIFF
//                ) {
////                cell->quadNode.LConfidence = std::max(100.0, cell->quadNode.LConfidence + confidenceIncrease);
//
//                cell->quadNode.LConfidence = calculateOccupancyProbability(cell->quadNode.LConfidence, Pn_zt);
//                double pheromone = calculatePheromone(cell->quadNode.visitedAtS, P(cell->quadNode.LConfidence), currentTimeS);
//                if (pheromone <= P_OCCUPIED_THRESHOLD) {
//                    cell->quadNode.occupancy = OCCUPIED;
//                    if (box.getSize() > RESOLUTION){
//                        split(cell, box, currentTimeS);
//                    } else {
//                        //Set neighbors
//                        cell->add_occupied_neighbors(box.getSize());
//                    }
//                } else {
//                    if (pheromone >= P_FREE_THRESHOLD) {
//                        cell->quadNode.occupancy = FREE;
//                    } else {
//                        cell->quadNode.occupancy = AMBIGUOUS;
//                    }
//                    //Remove neighbors; Make sure to remove neighbors if the cell is not occupied, or we will have pointer issues.
//                    cell->remove_neighbors();
//                }
//            }
//
//            //Only check further if the occupancy of the non-leaf cell is not all the same for its children, so ANY.
//            if (!isLeaf(cell) && (cell->quadNode.visitedAtS == -1 || cell->quadNode.occupancy == ANY ||
//                                  cell->quadNode.occupancy == UNKNOWN)) {
//                for (int d = 0; d < cell->children.size(); d++) {
//                    auto childBox = computeBox(box, static_cast<int>(d));
//                    if (functionSpace.intersects_or_contains(childBox)) {
//                        updateConfidence(cell->children.at(d).get(), childBox, functionSpace,
//                                         Pn_zt, currentTimeS);
//                    }
//                }
//            }
//        }
        //According to http://www.arminhornung.de/Research/pub/hornung13auro.pdf
        //Dynamic probability
        double calculateOccupancyProbability(double Ln_z1tMinus1, double Pn_zt) {
//            float Ln_z1tMinus1 = L(Pn_z1tMinus1); // L(P(n|z1:t-1))
            double Ln_zt = L(Pn_zt); // L(P(n|zt))
            double Ln_z1t = std::max(std::min(Ln_z1tMinus1 + Ln_zt, l_max), l_min); // L(P(n|z1:t)) = max(min(L(P(n|z1:t-1)) + L(P(n|zt)), l_max), l_min)
//            float Pn_z1t = std::exp(Ln_z1t) / (1 + std::exp(Ln_z1t)); // P(n|z1:t) = exp(L(P(n|z1:t))) / (1 + exp(L(P(n|z1:t))))
            return Ln_z1t;
        }

        //According to http://www.arminhornung.de/Research/pub/hornung13auro.pdf
        //Dynamic probability
        double calculateOccupancyProbabilityFromTwoL(double Ln_z1tMinus1, double Ln_zt) {
            double Ln_z1t = std::max(std::min(Ln_z1tMinus1 + Ln_zt, l_max), l_min); // L(P(n|z1:t)) = max(min(L(P(n|z1:t-1)) + L(P(n|zt)), l_max), l_min)
//            float Pn_z1t = std::exp(Ln_z1t) / (1 + std::exp(Ln_z1t)); // P(n|z1:t) = exp(L(P(n|z1:t))) / (1 + exp(L(P(n|z1:t))))
            return Ln_z1t;
        }

        double L(double p) const{
            assert(p >= 0 && p < 1);
            return std::log(p / (1-p));
        }

        double P(double l) const {
            return std::exp(l) / (1 + std::exp(l));
        }


        /**
         * @brief Get the QuadNode that contains the given coordinate
         * @param node
         * @param box
         * @param queryCoordinate
         * */
        QuadNode getQuadNodeFromCoordinate(Cell *node, const Box &box, const Coordinate &queryCoordinate) const {
            assert(node != nullptr);
            assert(box.contains(queryCoordinate));
            //If it is a leaf node, return the QuadNode if it exists. If it does not exist, it means this coordinate is unexplored.
            if (isLeaf(node)) {
                if (node->quadNode.visitedAtS == -1) {
                    return QuadNode{queryCoordinate, UNKNOWN, 0};
                } else {
                    assert(node->quadNode.occupancy != ANY && "leaf occupancy should never be ANY");
                    return node->quadNode;
                }
                // If it is not a leaf node, find the nested nodes, and search them.
            } else {
                //If the node occupancy is ANY or UNKNOWN, there can be nested nodes with different occupancies
                assert(node->quadNode.visitedAtS != -1 && "Cell should have a value");
                if (node->quadNode.occupancy == ANY || node->quadNode.occupancy == UNKNOWN) {
                    auto i = box.getQuadrant(queryCoordinate);
                    //If i=4, so the query coordinate is the exact center, check all children
                    if (i == 4) {
                        for (int j = 0; j < node->children.size(); j++) {
                            auto childBox = computeBox(box, static_cast<int>(j));

                            return getQuadNodeFromCoordinate(node->children.at(j).get(), childBox, queryCoordinate);
                        }
                    } else {
                        auto childBox = computeBox(box, static_cast<int>(i));

                        return getQuadNodeFromCoordinate(node->children.at(i).get(), childBox, queryCoordinate);
                    }
                    //Else the nested nodes have the same occupancy, so parent node can be returned.
                } else {
                    return node->quadNode;
                }
            }
            return QuadNode{queryCoordinate, UNKNOWN, 0};
        }

        /**
         * @brief Get the QuadNode that contains the given coordinate
         * @param node
         * @param box
         * @param queryCoordinate
         * */
        std::pair<Cell*, Box> getCellandBoxFromCoordinate(Cell *node, const Box &box, const Coordinate &queryCoordinate) const {
            assert(node != nullptr);
//            assert(box.contains(queryCoordinate));
            if (!box.contains(queryCoordinate)) {
                return std::make_pair(nullptr, box);
            }
            //If it is a leaf node, return the QuadNode if it exists. If it does not exist, it means this coordinate is unexplored.
            if (isLeaf(node)) {
                if (node->quadNode.visitedAtS == -1) {
                    return std::make_pair(nullptr, box);
                } else {
                    assert(node->quadNode.occupancy != ANY && "leaf occupancy should never be ANY");
                    return std::make_pair(node, box);
                }
                // If it is not a leaf node, find the nested nodes, and search them.
            } else {
                //If the node occupancy is ANY or UNKNOWN, there can be nested nodes with different occupancies
                assert(node->quadNode.visitedAtS != -1 && "Cell should have a value");
                if (node->quadNode.occupancy == ANY || node->quadNode.occupancy == UNKNOWN) {
                    auto i = box.getQuadrant(queryCoordinate);
                    //If i=4, so the query coordinate is the exact center, check all children
                    if (i == 4) {
                        for (int j = 0; j < node->children.size(); j++) {
                            auto childBox = computeBox(box, static_cast<int>(j));

                            return getCellandBoxFromCoordinate(node->children.at(j).get(), childBox, queryCoordinate);
                        }
                    } else {
                        auto childBox = computeBox(box, static_cast<int>(i));

                        return getCellandBoxFromCoordinate(node->children.at(i).get(), childBox, queryCoordinate);
                    }
                    //Else the nested nodes have the same occupancy, so parent node can be returned.
                } else {
                    return std::make_pair(node, box);
                }
            }
            return std::make_pair(nullptr, box);;
        }

        void findAllIntersections(Cell *node, std::vector<std::pair<QuadNode, QuadNode>> &intersections) const {
            // Find intersections between values stored in this node
            // Make sure to not report the same intersection twice
//            for (auto i = std::size_t(0); i < node->values.size(); ++i) {
//                for (auto j = std::size_t(0); j < i; ++j) {
//                    if (node->values[i].getBox().intersects(node->values[j].getBox()))
//                        intersections.emplace_back(node->values[i], node->values[j]);
//                }
//            }
//            if (!isLeaf(node)) {
//                // Values in this node can intersect values in descendants
//                for (const auto &child: node->children) {
//                    for (const auto &value: node->values)
//                        findIntersectionsInDescendants(child.get(), value, intersections);
//                }
//                // Find intersections in children
//                for (const auto &child: node->children)
//                    findAllIntersections(child.get(), intersections);
//            }
        }

        void
        findIntersectionsInDescendants(Cell *node, const QuadNode &value,
                                       std::vector<std::pair<QuadNode, QuadNode>> &intersections) const {
            // Test against the values stored in this node
//            for (const auto &other: node->values) {
//                if (value.getBox().intersects(other.getBox()))
//                    intersections.emplace_back(value, other);
//            }
//            // Test against values stored into descendants of this node
//            if (!isLeaf(node)) {
//                for (const auto &child: node->children)
//                    findIntersectionsInDescendants(child.get(), value, intersections);
//            }
        }
    };

}
