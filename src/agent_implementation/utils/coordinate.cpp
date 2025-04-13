//
// Created by hugo on 27-9-24.
//

#include "coordinate.h"



std::string Coordinate::toString() const {
    return std::to_string(this->x) + ";" + std::to_string(this->y);
}

/**
 * @brief Converts a heading from the own coordinate system to the argos coordinate system
 * @param radians
 * @return
 */
argos::CRadians Coordinate::OwnHeadingToArgos(argos::CRadians radians) {
    return argos::CRadians(radians - argos::CRadians::PI_OVER_TWO);
}

/**
 * @brief Converts a heading from the argos coordinate system to the own coordinate system
 * @param radians
 * @return
 */
argos::CRadians Coordinate::ArgosHeadingToOwn(argos::CRadians radians) {
    return argos::CRadians(radians + argos::CRadians::PI_OVER_TWO);
}

/**
 * @brief Converts the coordinate from the own coordinate system to the argos coordinate system
 * @return
 */
Coordinate Coordinate::FromOwnToArgos() const{
    return Coordinate{y, -x};
}

/**
 * @brief Converts the coordinate from the argos coordinate system to the own coordinate system
 * @return
 */
Coordinate Coordinate::FromArgosToOwn() const{
    return Coordinate{-y, x};
}

/**
 * @brief Compares two coordinates for equality
 * @return
 */
bool Coordinate::operator==(const Coordinate &rhs) const  {
    return x == rhs.x &&
           y == rhs.y;
}

/**
 * @brief Compares two coordinates
 * @return
 */
bool Coordinate::operator<(const Coordinate &rhs) const  {
     if (x == rhs.x) return y < rhs.y;
     return x < rhs.x;
}

