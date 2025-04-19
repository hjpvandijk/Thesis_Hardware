#pragma once

#include "coordinate.h"
// #include <argos3/core/utility/logging/argos_log.h>

//Adapted from https://github.com/pvigier/Quadtree


namespace quadtree {

    class Box {
    public:
        double left;
        double top;
        double size; // Must be positive
//    double height; // Must be positive

        Box() noexcept:
                left(0), top(0), size(0) {

        }

        constexpr Box(double Left, double Top, double Size) noexcept:
                left(Left), top(Top), size(Size) {

        }

        constexpr Box(const Coordinate position, double size) noexcept:
                left(position.x), top(position.y), size(size) {

        }

        constexpr double getRight() const noexcept {
            return left + size;
        }

        constexpr double getBottom() const noexcept {
            return top - size;
        }

        constexpr Coordinate getTopLeft() const noexcept {
            return {left, top};
        }

        constexpr Coordinate getCenter() const noexcept {
            return {left + size / 2, top - size / 2};
        }

        constexpr double getSize() const noexcept {
            return size;
        }

        /**
         * @brief Check if the box contains another box
         * @param box
         * @return
         */
        [[nodiscard]] bool contains(const Box &box) const noexcept {
            bool result = left <= box.left && getRight() >= box.getRight() &&
                          top >= box.top && getBottom() <= box.getBottom();

            return result;

        }

        /**
         * @brief Check if the box contains a coordinate
         * @param coordinate
         * @return
         */
        [[nodiscard]] bool contains(const Coordinate &coordinate) const noexcept {
            bool result = left <= coordinate.x && getRight() >= coordinate.x &&
                          top >= coordinate.y && getBottom() <= coordinate.y;

            return result;

        }

        /**
         * @brief Get the quadrant which the given coordinate should go in in the node box
         * @param nodeBox
         * @param valueCoordinate
         * @return
         */
        int getQuadrant(const Coordinate &valueCoordinate) const {
            auto center = this->getCenter();

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
         * @brief Return a child box of the current box
         * @param i
         * @return
         */
        Box boxFromQuadrant(int i) const {
            switch (i) {
                case 0:
                    return Box(left, top, size / 2);
                case 1:
                    return Box(left + size / 2, top, size / 2);
                case 2:
                    return Box(left, top - size / 2, size / 2);
                case 3:
                    return Box(left + size / 2, top - size / 2, size / 2);
                default:
                    return Box();
            }
        }

        /**
         * @brief Check if the box intersects or contains another box
         * @param box
         * @return
         */
        bool intersects_or_contains(const Box &box) const noexcept {

            Coordinate topLeft = getTopLeft();
            Coordinate bottomRight = {getRight(), getBottom()};
            Coordinate topRight = {getRight(), top};
            Coordinate bottomLeft = {left, getBottom()};

            Coordinate boxTopLeft = box.getTopLeft();
            Coordinate boxBottomRight = {box.getRight(), box.getBottom()};
            Coordinate boxTopRight = {box.getRight(), box.top};
            Coordinate boxBottomLeft = {box.left, box.getBottom()};


            bool result = (box.contains(topLeft) || box.contains(bottomRight) || box.contains(topRight) ||
                     box.contains(bottomLeft) ||
                     contains(boxTopLeft) || contains(boxBottomRight) || contains(boxTopRight) ||
                     contains(boxBottomLeft));

            return result;
        }

        bool operator==(const Box &b) const {
            return this->left == b.left && this->top == b.top && this->size == b.size;
        }

        /**
         * Get the 8-connected moore neighbours of the box
         * @return vector of neighboors
         */
        std::vector<Box> getMooreNeighbours(){
            std::vector<Box> neighbours;
            neighbours.push_back(Box(left-size, top, size));
            neighbours.push_back(Box(left-size, top+size, size));
            neighbours.push_back(Box(left, top+size, size));
            neighbours.push_back(Box(left+size, top+size, size));
            neighbours.push_back(Box(left+size, top, size));
            neighbours.push_back(Box(left+size, top-size, size));
            neighbours.push_back(Box(left, top-size, size));
            neighbours.push_back(Box(left-size, top-size, size));
            return neighbours;
        }
    };

}