#include <iostream>
#include <iomanip>
#include <random>
#include <ctime>

#include "functions.h"
#include "rootapprox.h"
#include "normalmethods.h"


// edges(i0) is the left edge of the ith root's space and edges(i1) the right one.
const double edges[5][2] = {
        {-1.500, -1.200}, // Space of the first root
        {-1.198, -0.666}, // Space of the second root
        {-0.172, 0.3660}, // Space of the third root
        {0.3670, 0.9000}, // Space of the fourth root
        {0.9800, 1.2000}  // Space of the fifth root
};

int main()
{
    double roots[5];
    double (*f[3])(double);

    int i;
    int j;
    int reps[5];

    generator = new std::mt19937(time(nullptr) + 2);

    std::atexit([]() {
        delete generator;
    });

    f[0] = function;
    f[1] = function_prime;
    f[2] = function_secondary;

    std::cout << std::setprecision(5);

    std::cout << "----- FIRST QUESTION -----\n" << std::endl;

    std::cout << "Modified Newton-Raphson method" << std::endl;
    for (i = 0; i < 5; ++i) {
        // Gets all five roots and the iterations they required
        roots[i] = modifiedNewton(f, edges[i][0], edges[i][1], reps[i]);
        // Prints the results to the console.
        std::cout << "- x" << i + 1 << " = " << roots[i] << " (" << reps[i] << " iterations)" << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Modified Bisection method" << std::endl;
    for (i = 0; i < 5; ++i) {
        roots[i] = modifiedBisection(f[0], edges[i][0], edges[i][1], reps[i]);
        std::cout << "- x" << i + 1 << " = " << roots[i] << " (" << reps[i] << " iterations)" << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Modified Secant method" << std::endl;
    for (i = 0; i < 5; ++i) {
        roots[i] = modifiedSecant(f[0], edges[i][0], edges[i][1], reps[i]);
        std::cout << "- x" << i + 1 << " = " << roots[i] << " (" << reps[i] << " iterations)" << std::endl;
    }
    std::cout << std::endl;


    std::cout << "\n----- SECOND QUESTION -----\n" << std::endl;

    double R[5][10];

    std::cout << "By repeating th modified bisection method 10 times we have:" << std::endl;
    for (j = 0; j < 10; ++j)
    {
        for (i = 0; i < 5; ++i) {
            modifiedBisection(f[0], edges[i][0], edges[i][1], reps[i]);
            R[i][j] = reps[i];
        }
    }

    for (i = 0; i < 5; ++i)
    {
        std::cout << "x" << i + 1 << "'s iterations: {";
        for (j = 0; j < 9; ++j)
        {
            std::cout << R[i][j] << ", ";
        }
        std::cout << R[i][9] << "}" << std::endl;
    }


    std::cout << "\n----- THIRD QUESTION -----\n" << std::endl;

    int Q1[5][3];
    int Q2[5][3];

    for (i = 0; i < 5; ++i)
    {
        getRootBisection(f[0], edges[i][0], edges[i][1], Q1[i][0]);
        getRootNewton(f, edges[i][0], edges[i][1], Q1[i][1]);
        getRootSecant(f[0], edges[i][0], edges[i][1], Q1[i][2]);

        modifiedBisection(f[0], edges[i][0], edges[i][1], Q2[i][0]);
        modifiedNewton(f, edges[i][0], edges[i][1], Q2[i][1]);
        modifiedSecant(f[0], edges[i][0], edges[i][1], Q2[i][2]);
    }

    const char *METHOD[] = {
            "Bisection", "Newton-Raphson", "Secant"
    };

    for (i = 0; i < 5; ++i)
    {
        std::cout << "x" << i + 1 << "'s iterations:" << std::endl;
        for (j = 0; j < 3; ++j) {
            std::cout << "- " << METHOD[j] << ": " << Q1[i][j] << " (";
            std::cout << "Original: " << Q1[i][j] << ", ";
            std::cout << "Modified: " << Q2[i][j] << ", ";
            if (Q2[i][j] != 0) {
                std::cout << "Ratio: " << (double) Q1[i][j] / Q2[i][j] << ")\n";
            } else {
                std::cout << "Ratio: undefined" << ")\n";
            }
        }
        std::cout << std::endl;
    }

    return EXIT_SUCCESS;
}
