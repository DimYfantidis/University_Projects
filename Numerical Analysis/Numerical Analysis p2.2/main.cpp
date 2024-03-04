#include <iostream>
#include <vector>
#include <utility>
#include <cmath>
#include <iomanip>

#include "exercise6.h"

#ifndef M_PI
#   define M_PI 3.14159265359
#endif


void init_sine_samples_11(std::vector<std::pair<double, double>>& v)
{
    double x;
    for (int i = 0; i < v.size(); ++i) {
        x = i * M_PI / 20;
        v[i] = {x, sin(x)};
    }
}

int main()
{
    using std::vector;
    using std::pair;

    vector<pair<double, double>> SINE_SAMPLES_11(11);

    double error;
    double I;

    // Auxiliary function for preventing the command prompt from closing when running in cmd.
    std::atexit([] () {
        std::cout << '\n' << "Press \"Enter\" to exit ..." << std::endl;
        std::cin.get();
    });

    init_sine_samples_11(SINE_SAMPLES_11);

    std::cout << std::fixed << std::setprecision(8);

    // [1]: Calculation of integral using trapezoids.
    std::cout << "[1]: TRAPEZOIDS METHOD" << std::endl;

    I = integralTrapezoid(SINE_SAMPLES_11);
    std::cout << "Result integral: " << I << std::endl;

    error = std::abs(1 - I);
    std::cout << "Error: " << error << '\n' <<  std::endl;

    // [2]: Calculation of integral using Simpson's method.
    std::cout << "[2]: SIMPSON'S METHOD" << std::endl;

    I = integralSimpson(SINE_SAMPLES_11);
    std::cout << "Result integral: " << I << std::endl;

    error = std::abs(1 - I);
    std::cout << "Error: " << error << '\n' <<  std::endl;

    return 0;
}
