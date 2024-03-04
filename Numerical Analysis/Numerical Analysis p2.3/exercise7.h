#ifndef EXERCISE7_H
#define EXERCISE7_H

#include <vector>
#include "matrix.h"
#include "polynomial.h"
#include "utils.h"


/*
 *  Approximation using least squares method.
 *  - 1st param: degree -> degree of the polynomial φ(x)
 *  - 2nd param: samples -> n-sized list of (x_i, f(x_i)) sample pairs (i = 0, 1, ..., n - 1)
 *               used for creating φ(x).
 *  - returns: φ(x)
 */
polynomial *leastSquares(int degree, const std::vector<std::pair<double, double>>& samples)
{
    auto *phi = new polynomial(degree);
    matrix A((dimension_t) samples.size(), degree + 1);
    matrix b((dimension_t) samples.size(), 1);

    for (dimension_t i = 0; i < A.numOfRows(); ++i) {
        for (dimension_t j = 0; j < A.numOfCols(); ++j) {
            A[i][j] = pow(samples[i].first, j);
        }
        b[i][0] = samples[i].second;
    }

    matrix A2 = A.transpose() * A;
    matrix b2 = A.transpose() * b;

    std::vector<double> terms = gauss(A2, b2.toVector());

    if (terms.size() != phi->getDegree() + 1) {
        std::cerr << "Unexpected error: gauss result size != " << phi->getDegree() + 1 << std::endl;
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i <= phi->getDegree(); ++i) {
        phi->setTerm(i, terms[i]);
    }

    return phi;
}



#endif //EXERCISE7_H
