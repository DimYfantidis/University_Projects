#include <iostream>
#include <utility>
#include <vector>
#include <cmath>
#include <cstdlib>

#include "exercise5.h"
#include "utils.h"

#ifndef M_PI
#   define M_PI 3.14159265359
#endif


// Array of (x, sin(x)) pairs, ten samples included.
void init_sine_samples_10(std::vector<std::pair<double, double>>& v)
{
    v[0] = {-M_PI        ,  0.0};
    v[1] = {-3 * M_PI / 4, -0.7071067811865};
    v[2] = {-M_PI / 2    , -1.0};
    v[3] = {-M_PI /4     , -0.7071067811865};
    v[4] = {-M_PI / 6    , -0.5};
    v[5] = {M_PI / 6     ,  0.5};
    v[6] = {M_PI / 4     ,  0.7071067811865};
    v[7] = {M_PI / 2     ,  1.0};
    v[8] = {3 * M_PI / 4 ,  0.7071067811865};
    v[9] = {M_PI         ,  0.0};
}

// Array of (x, sin(x)) pairs, 200 samples included.
void init_sine_samples_200(std::vector<std::pair<double, double>>& v)
{
    double x;

    for (int i = 0; i < v.size(); ++i) {
        x = M_PI * (-1 + (2 * i) / 199.0);
        v[i].first = x;
        v[i].second = sin(x);
    }
}


int main()
{
    using std::pair;
    using std::vector;

    vector<pair<double, double>> SINE_SAMPLES_10(10);
    vector<pair<double, double>> SINE_SAMPLES_200(200);
    vector<pair<double, double>> errors_v;

    double max_error = 0.0;
    double median_error = 0.0;
    double most_flawed_x = 0.0;
    double error_i;
    double phi_x;
    double f_x;

    auto reset_variables = [&] () {
        max_error = 0.0;
        median_error = 0.0;
        error_i = 0.0;
        phi_x = 0.0;
        f_x = 0.0;
    };

    // Auxiliary function for preventing the command prompt from closing when running in cmd.
    std::atexit([] () {
        std::cout << '\n' << "Press \"Enter\" to exit ..." << std::endl;
        std::cin.get();
    });

    init_sine_samples_10(SINE_SAMPLES_10);
    init_sine_samples_200(SINE_SAMPLES_200);

    // ----------- [1]: POLYNOMIAL APPROXIMATION USING LAGRANGE'S ALGORITHM -----------
    std::cout << "[1]: POLYNOMIAL APPROXIMATION" << std::endl;

    // Calculates the infinite norm of φ(x) - f(x) for 200 samples.
    for (const auto& point: SINE_SAMPLES_200)
    {
        phi_x = polynomialLagrange(point.first, SINE_SAMPLES_10);
        f_x = point.second;
        error_i = std::abs(phi_x - f_x);
        median_error += error_i;

        errors_v.emplace_back(point.first, error_i);

        if (error_i > max_error) {
            most_flawed_x = point.first;
            max_error = error_i;
        }
    }
    median_error /= 200.0;

    exportVectorToFile("Polynomial errors.txt", errors_v);
    errors_v.clear();

    // Prints the infinite norm
    std::cout << "Maximum error: " << max_error << " at x = " << most_flawed_x << '\n';
    std::cout << "Median error: " << median_error << '\n';
    std::cout << "(Worst decimal precision: " << precision(max_error) << " digits)\n";
    std::cout << " ---------------------------------------------------------------------------\n" << std::endl;

    reset_variables();



    // ----------- [2]: APPROXIMATION USING SPLINES -----------
    std::cout << "[2]: SPLINES" << std::endl;

    Splines S(SINE_SAMPLES_10);

    // Calculates the infinite norm of φ(x) - f(x) for 200 samples.
    for (const auto& point: SINE_SAMPLES_200)
    {
        phi_x = S.getValue(point.first);
        f_x = point.second;
        error_i = std::abs(phi_x - f_x);
        median_error += error_i;

        errors_v.emplace_back(point.first, error_i);

        if (error_i > max_error) {
            most_flawed_x = point.first;
            max_error = error_i;
        }
    }
    median_error /= 200.0;

    exportVectorToFile("Spline errors.txt", errors_v);
    errors_v.clear();

    // Prints the infinite norm
    std::cout << "Maximum error: " << max_error << " at x = " << most_flawed_x << '\n';
    std::cout << "Median error: " << median_error << '\n';
    std::cout << "(Worst decimal precision: " << precision(max_error) << " digits)\n";
    std::cout << " ---------------------------------------------------------------------------\n" << std::endl;

    reset_variables();



    // ----------- [3]: APPROXIMATION USING METHOD OF THE LEAST SQUARES -----------
    std::cout << "[3]: LEAST SQUARES" << std::endl;

    polynomial *least_squares = leastSquares(3, SINE_SAMPLES_10);

    std::cout << "Approximation of least squares using 3rd degree polynomial: " << '\n';
    std::cout << "y = " << *least_squares << '\n' << std::endl;

    // Calculates the infinite norm of φ(x) - f(x) for 200 samples.
    for (const auto& point: SINE_SAMPLES_200)
    {
        phi_x = least_squares->getValue(point.first);
        f_x = point.second;

        error_i = std::abs(phi_x - f_x);
        median_error += error_i;

        errors_v.emplace_back(point.first, error_i);

        if (error_i > max_error) {
            most_flawed_x = point.first;
            max_error = error_i;
        }
    }
    median_error /= 200.0;

    exportVectorToFile("Least Squares errors.txt", errors_v);
    errors_v.clear();

    // Prints the infinite norm.
    std::cout << "Maximum error: " << max_error << " at x = " << most_flawed_x << '\n';
    std::cout << "Median error: " << median_error << '\n';
    std::cout << "(Worst decimal precision: " << precision(max_error) << " digits)\n" << std::endl;

    reset_variables();

    delete least_squares;



    return EXIT_SUCCESS;
}
