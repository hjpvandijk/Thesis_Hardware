//
// Created by hugo on 17-6-24.
//

#ifndef THESIS_ARGOS_COORDINATE_H
#define THESIS_ARGOS_COORDINATE_H

#include <string>
// #include "argos3/core/utility/math/angles.h"
#include "angles.h"

struct Coordinate {
    double x;
    double y;

    [[nodiscard]] std::string toString() const;

    /**
     * @brief Converts a heading from the own coordinate system to the argos coordinate system
     * @param radians
     * @return
     */
    static argos::CRadians OwnHeadingToArgos(argos::CRadians radians);

    /**
     * @brief Converts a heading from the argos coordinate system to the own coordinate system
     * @param radians
     * @return
     */
    static argos::CRadians ArgosHeadingToOwn(argos::CRadians radians);

    /**
     * @brief Converts the coordinate from the own coordinate system to the argos coordinate system
     * @return
     */
    [[nodiscard]] Coordinate FromOwnToArgos() const;

    /**
     * @brief Converts the coordinate from the argos coordinate system to the own coordinate system
     * @return
     */
    [[nodiscard]] Coordinate FromArgosToOwn() const;

    /**
     * @brief Compares two coordinates for equality
     * @return
     */
    bool operator==(const Coordinate &rhs) const;

    /**
     * @brief Compares two coordinates for being smaller
     * @return
     */
    bool operator<(const Coordinate &rhs) const;



};


#endif //THESIS_ARGOS_COORDINATE_H
