#ifndef ROOTAPPROX_H
#define ROOTAPPROX_H

#include <cmath>
#include <random>
#include <ctime>
#include "utils.h"

// Bisection Method
double getRootBisection(double (*f)(double), double a, double b, int& reps)
{
    double m = (a + b) / 2;
    double m_prev = m - 1;
    double f_m;
    const int total_steps = (int)((log(b - a) - log(EPSILON)) / log(2)) + 1;

    if (f(a) * f(b) >= 0) {
        return std::nan("0");
    }

    for (int i = 0; i < total_steps; ++i)
    {
        m = (a + b) / 2;
        f_m = f(m);

        if (isEqual(f_m, 0.0) && isEqual(m, m_prev)) {
            reps = i + 1;
            return m;
        }

        m_prev = m;

        if (f_m * f(a) < 0) {
            b = m;
        } else {
            a = m;
        }
    }
    reps = total_steps;

    return m;
}

// Newton-Raphson Method
double getRootNewton(double (*f[3])(double), double a, double b, int& reps)
{
    int i;
    double x0;
    double x1;
    double x2;

    if (f[0](a) * f[0](b) >= 0) {
        return std::nan("0");
    }

    std::mt19937 generator(time(nullptr) + 2);
    std::uniform_real_distribution<double> distribution(a, b);

    do {
        x0 = distribution(generator);
    } while (f[0](x0) * f[2](x0) <= 0);

    x1 = x0;
    x2 = x1 + 1;

    for (i = 0; !isEqual(x2, x0) || !isEqual(f[0](x1), 0.0); ++i)
    {
        // The extra variable 'x0' is used solely for the loop condition.
        x0 = x1;
        x2 = x1 - f[0](x0) / f[1](x0);

        if (std::isnan(x1)) {
            reps = i + 1;
            return x0;
        }
        x1 = x2;
    }
    reps = i;

    return x1;
}

// Secant Method
double getRootSecant(double (*f)(double), double a, double b, int& reps)
{
    int i;
    double x0 = a;
    double x1 = b;
    double x2 = (a + b) / 2;

    for (i = 0; !isEqual(x2, x0) || !isEqual(f(x2), 0.0); ++i)
    {
        x2 = x1 - (f(x1) * (x1 - x0)) / (f(x1) - f(x0));

        if (std::isnan(x2)) {
            reps = i + 1;
            return x1;
        }
        x0 = x1;
        x1 = x2;
    }
    reps = i;

    return x2;
}

#endif //ROOTAPPROX_H
