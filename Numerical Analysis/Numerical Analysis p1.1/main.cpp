#include <iostream>
#include <iomanip>

#include "rootapprox.h"
#include "functions.h"


void pause() {
    std::cout << "Press \"Enter\" to exit >>> ";
    while (std::cin.get() != '\n');
}

constexpr double MID = 1.156;

int main()
{
    double r1;
    double r2;

    int reps_1;
    int reps_2;

    double (*f[3])(double);

    std::atexit(pause);
    std::cout << std::setprecision(5);

    f[0] = function;
    f[1] = function_prime;
    f[2] = function_secondary;


    r1 = getRootBisection(f[0], 0, MID, reps_1);
    r2 = getRootBisection(f[0], MID, 3, reps_2);
    std::cout << "Roots of function - (Bisection method, range:[0,3]):" << std::endl;
    std::cout << "- x1 = " << r1 << " (reps: " << reps_1 << ")" << std::endl;
    std::cout << "- x2 = " << r2 << " (reps: " << reps_2 << ")" << std::endl;
    std::cout << std::endl;


    r1 = getRootNewton(f, 0, MID, reps_1);
    r2 = getRootNewton(f, MID, 3, reps_2);
    std::cout << "Roots of function - (Newton-Raphson method, range[0, 3]):" << std::endl;
    std::cout << "- x1 = " << r1 << " (reps: " << reps_1 << ")" << std::endl;
    std::cout << "- x2 = " << r2 << " (reps: " << reps_2 << ")" << std::endl;
    std::cout << std::endl;


    r1 = getRootSecant(f[0], 0, MID, reps_1);
    r2 = getRootSecant(f[0], MID, 3, reps_2);
    std::cout << "Roots of function - (Secant method, range[0, 3]):" << std::endl;
    std::cout << "- x1 = " << r1 << " (reps: " << reps_1 << ")" << std::endl;
    std::cout << "- x2 = " << r2 << " (reps: " << reps_2 << ")" << std::endl;
    std::cout << std::endl;


    return EXIT_SUCCESS;
}
