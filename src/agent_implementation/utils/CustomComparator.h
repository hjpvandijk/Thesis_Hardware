//
// Created by hugo on 22-10-24.
//

#ifndef IMPLEMENTATION_AND_EXAMPLES_CUSTOMCOMPARATOR_H
#define IMPLEMENTATION_AND_EXAMPLES_CUSTOMCOMPARATOR_H

// #include <argos3/core/utility/math/angles.h>
#include "angles.h"

// Custom comparator to order set for wall following. The set is ordered by the angle difference to the wall following direction
struct CustomComparator {
    int dir;  // dir is either 0, 1 or -1
    double heading;
    double targetAngle;

    CustomComparator(int dir, double heading, double targetAngle) : dir(dir), heading(heading),
                                                                    targetAngle(targetAngle) {}


    // Custom comparator logic
    bool operator()(const argos::CDegrees &a, const argos::CDegrees &b) const {

        auto a_diff = a - argos::CDegrees(this->heading);
        auto b_diff = b - argos::CDegrees(this->heading);

        a_diff.SignedNormalize();
        b_diff.SignedNormalize();

        auto a_diff_val = a_diff.GetValue();
        auto b_diff_val = b_diff.GetValue();
        if (dir == 0) {
            a_diff_val = std::abs(NormalizedDifference(a, argos::CDegrees(this->targetAngle)).GetValue());
            b_diff_val = std::abs(NormalizedDifference(b, argos::CDegrees(this->targetAngle)).GetValue());
            if (a_diff_val == b_diff_val) {
                return a < b;
            }
            return a_diff_val < b_diff_val; //Normal ascending order
        } else if (dir == 1) {
            // Handle the first half: 90 to -180
            if (a_diff_val <= 90 && a_diff_val >= -180 && b_diff_val <= 90 && b_diff_val >= -180) {
                return a_diff_val > b_diff_val;  // Normal descending order
            }

            // Handle the second half: 180 to 91
            if (a_diff_val > 90 && a_diff_val <= 180 && b_diff_val > 90 && b_diff_val <= 180) {
                return a_diff_val > b_diff_val;  // Normal descending order
            }

            // Prioritize the first half (90 to -180) over the second half (180 to 91)
            if ((a_diff_val <= 90 && a_diff_val >= -180) && (b_diff_val > 90 && b_diff_val <= 180)) {
                return true;  // 'a' should come before 'b'
            }
            if ((a_diff_val > 90 && a_diff_val <= 180) && (b_diff_val <= 90 && b_diff_val >= -180)) {
                return false;  // 'b' should come before 'a'
            }
        } else {
            // Handle the first half: -90 to 180
            if (a_diff_val >= -90 && a_diff_val <= 180 && b_diff_val >= -90 && b_diff_val <= 180) {
                return a_diff_val < b_diff_val;  // Normal descending order
            }

            // Handle the second half: -180 to -91
            if (a_diff_val < -90 && a_diff_val >= -180 && b_diff_val < -90 && b_diff_val >= -180) {
                return a_diff_val < b_diff_val;  // Normal descending order
            }

            // Prioritize the first half (-90 to 180) over the second half (-180 to -91)
            if ((a_diff_val >= -90 && a_diff_val <= 180) && (b_diff_val < -90 && b_diff_val >= 180)) {
                return true;  // 'a' should come before 'b'
            }
            if ((a_diff_val < -90 && a_diff_val >= -180) && (b_diff_val >= -90 && b_diff_val <= 180)) {
                return false;  // 'b' should come before 'a'
            }
        }

        return a_diff_val > b_diff_val;  // Default to descending order if somehow unmatched
    }
};

#endif //IMPLEMENTATION_AND_EXAMPLES_CUSTOMCOMPARATOR_H
