#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <cmath>


double function(double x)
{
    double res = 0;
    res += 54 * pow(x, 6);
    res += 45 * pow(x, 5);
    res -= 102 * pow(x, 4);
    res -= 69 * pow(x, 3);
    res += 35 * x * x;
    res += 16 * x - 4;
    return res;
}

double function_prime(double x)
{
    double res = 0;
    res += 324 * pow(x, 5);
    res += 225 * pow(x, 4);
    res -= 408 * pow(x, 3);
    res -= 207 * x * x;
    res += 70 * x + 16;
    return res;
}

double function_secondary(double x)
{
    double res = 0;
    res += 1620 * pow(x, 4);
    res += 900 * pow(x, 3);
    res -= 1224 * x * x;
    res -= 414 * x + 70;
    return res;
}

#endif //FUNCTIONS_H
