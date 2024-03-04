#ifndef UTILS_H
#define UTILS_H

#include <cmath>


constexpr double EPSILON = 0.5 * 0.00001;

std::mt19937 *generator;

inline bool isEqual(double x, double y) {
    return std::abs(x - y) < EPSILON;
}

#endif //UTILS_H
