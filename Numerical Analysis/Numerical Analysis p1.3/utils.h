#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <vector>
#include <sstream>


constexpr double EPSILON = 0.5 * 0.0001;

inline bool isEqual(double x, double y) {
    return std::abs(x - y) < EPSILON;
}

// Calculates the l0 norm of a vector.
double infiniteNorm(const std::vector<double>& v)
{
    double l0 = std::abs(v[0]);
    double temp;

    for (int i = 1; i < v.size(); ++i)
    {
        temp = std::abs(v[i]);
        if (temp > l0) {
            l0 = temp;
        }
    }
    return l0;
}

// Auxiliary function for subtracting vector b from vector a.
std::vector<double> vectorDifference(const std::vector<double>& a, const std::vector<double>& b)
{
    if (a.size() != b.size()) {
        std::cerr << "Cannot subtract vectors of different dimension" << std::endl;
        return {};
    }
    auto N = a.size();
    std::vector<double> v(N);

    for (decltype(N) i = 0; i < N; ++i)
    {
        v[i] = a[i] - b[i];
    }
    return v;
}

// Auxiliary function for printing a vector.
auto printVector(const std::vector<double>& v) {
    std::ostringstream oss;

    oss << std::setprecision(4);
    oss << "{";
    for (int i = 0; i < v.size() - 1; ++i) {
        oss << v[i] << ", ";
    }
    oss << v[v.size() - 1] << "}";

    return oss.str();
}

bool inputFail(std::istream& is)
{
    if (is.fail()) {
        is.clear();
        is.ignore(1000, '\n');
        return true;
    }
    return false;
}

#endif //UTILS_H
