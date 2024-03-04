#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <cmath>


double function(double x)
{
    double res = 0;
    res += 14 * x * exp(x - 2);
    res -= 12 * exp(x - 2);
    res -= 7 * pow(x, 3);
    res += 20 * x * x;
    res -= 26 * x;
    res += 12;
    return res;
}

double function_prime(double x)
{
    double res = 0;
    res += 2 * exp(x - 2) * (7 * x + 1);
    res -= 21 * x * x;
    res += 40 * x;
    res -= 26;
    return res;
}

double function_secondary(double x)
{
    double res = 0;
    res += 2 * exp(x - 2) * (7 * x + 8);
    res -= 42 * x;
    res += 40;
    return res;
}

#endif //FUNCTIONS_H
