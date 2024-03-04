#ifndef ROOTAPPROX_H
#define ROOTAPPROX_H

#include <cmath>
#include <ctime>
#include <random>
#include <vector>
#include "utils.h"


double modifiedNewton(double (*f[3])(double), double a, double b, int& reps)
{
    double x0 = a;
    double x1 = a;
    double x2 = b;

    double t1;
    double t2;
    int i;

    if (f[0](a) * f[0](b) >= 0) {
        reps = 0;
        return std::nan("0");
    }

    for (i = 0; !isEqual(x1, x0) || !isEqual(f[0](x1), 0.0); ++i)
    {
        x0 = x1;

        t1 = f[1](x1) / f[0](x1);
        t2 = f[2](x1) / (2 * f[1](x1));

        x2 = x1 - 1 / (t1 - t2);

        if (std::isnan(x2))
        {
            reps = i + 1;
            return x1;
        }
        x1 = x2;
    }
    reps = i;

    return x2;
}

double modifiedBisection(double (*f)(double), double a, double b, int& reps)
{
    std::uniform_real_distribution<double> distribution(a, b);

    int i;
    double m = distribution(*generator);
    double f_m = f(m);


    if (isEqual(f(a), 0.0)) return a;
    if (isEqual(f(b), 0.0)) return b;

    if (f(a) * f(b) > 0) {
        reps = 0;
        return std::nan("0");
    }

    for (i = 1; !isEqual(a, b) || !isEqual(f_m, 0.0); ++i)
    {
        if (f_m * f(a) < 0) {
            b = m;
        } else {
            a = m;
        }
        distribution.param(decltype(distribution)::param_type(a, b));
        m = distribution(*generator);
        f_m = f(m);
    }
    reps = i;

    return m;
}

double modifiedSecant(double (*f)(double), double a, double b, int &reps)
{
    if (f(a) * f(b) > 0) {
        reps = 0;
        return std::nan("0");
    }

    double x0 = a;
    double x1 = (a + b) / 2;
    double x2 = b;
    double x3 = x2 + 1;

    int i;
    double q;
    double r;
    double s;


    for (i = 0; !isEqual(x3, x2) || !isEqual(f(x3), 0.0); ++i)
    {
        x2 = x3;

        q = f(x0) / f(x1);
        r = f(x2) / f(x1);
        s = f(x2) / f(x0);

        x3 = x2 - (r * (r - q) * (x2 - x1) + (1 - r) * s * (x2 - x0)) / ((q - 1) * (r - 1) * (s - 1));

        if (std::isnan(x3)) {
            reps = i + 1;
            return x2;
        }
        x0 = x1;
        x1 = x2;
    }
    reps = i;

    return x3;
}

#endif //ROOTAPPROX_H
