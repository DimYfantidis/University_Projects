#ifndef LINEARSYS_H
#define LINEARSYS_H

#include "matrix.h"
#include "utils.h"
#include <vector>


std::vector<double> gauss(const matrix& M, const std::vector<double>& b)
{
    if (M.numOfRows() != M.numOfCols()) {
        std::cerr << "Matrix error: square matrix expected" << std::endl;
        return {};
    }
    if (b.size() != M.numOfRows()) {
        std::cerr << "Vector error: parse a proper vector as argument" << std::endl;
        return {};
    }

    dimension_t N = M.numOfRows();

    matrix M_aug(N, N + 1);

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            M_aug[i][j] = M[i][j];
        }
    }
    for (int i = 0; i < N; ++i) {
        M_aug[i][N] = b[i];
    }

    int i;
    int j;
    int k;
    double temp;

    for (j = 0; j < N; ++j)
    {
        for (i = j + 1; i < N; ++i)
        {
            temp = - (M_aug[i][j] / M_aug[j][j]);

            for (k = 0; k < N + 1; ++k)
            {
                M_aug[i][k] = temp * M_aug[j][k] + M_aug[i][k];
            }
        }
    }

    std::vector<double> sol(N);

    double q;
    double s;
    int back;

    for (i = 0; i < N; ++i)
    {
        if (M_aug[N - 1 - i][N - 1 - i] == 0)
        {
            if (M_aug[N - 1 - i][N] == 0) {
                std::cout << "Linear System has infinite solutions" << std::endl;
            }
            else {
                std::cout << "Linear System has no solutions" << std::endl;
            }
            return { std::nan("0") };
        }

        back = N - 1 - i;
        q = M_aug[back][N];

        for (s = j = 0; j < i; ++j) {
            s += M_aug[back][N - 1 - j] * sol[N - 1 - j];
        }
        sol[back] = (q - s) / M_aug[back][back];
    }

    return sol;
}

matrix* cholesky(const matrix& M)
{
    if (M.numOfRows() != M.numOfCols()) {
        std::cerr << "Square matrix expected" << std::endl;
        return nullptr;
    }
    if (!M.isSymmetric()) {
        std::cerr << "Symmetric matrix expected" << std::endl;
        return nullptr;
    }
    dimension_t N = M.numOfRows();

    auto* L = new matrix(N, N);
    L->init(0.0);

    (*L)[0][0] = sqrt(M[0][0]);

    int i;
    int j;
    int k;
    double sum;

    for (i = 0; i < N; i++)
    {
        for (j = 0; j <= i; j++)
        {
            sum = 0;
            for (k = 0; k < j; k++) {
                sum += (*L)[i][k] * (*L)[j][k];
            }
            if (i == j)
                (*L)[i][j] = sqrt(M[i][j] - sum);
            else
                (*L)[i][j] = (M[i][j] - sum) / (*L)[j][j];
        }
    }
    return L;
}


std::vector<double> gaussSeidel(const matrix& M, const std::vector<double>& b, int& reps)
{
    if (M.numOfRows() != M.numOfCols()) {
        std::cerr << "Matrix error: square matrix expected" << std::endl;
        return {};
    }
    if (b.size() != M.numOfRows()) {
        std::cerr << "Vector error: parse a proper vector as argument" << std::endl;
        return {};
    }
    dimension_t N = M.numOfRows();

    std::vector<double> x(N, 0.0);
    std::vector<double> x_old(N, 0.0);
    std::vector<double> x_diff(N, 1.0);

    int i;
    int j;
    int iterations = 0;
    double l0 = 1;
    double sum1;
    double sum2;

    while (l0 >= EPSILON)
    {
        x_old = x;

        for (i = 0; i < N; ++i)
        {
            sum1 = 0;
            for (j = 0; j < i; ++j) {
                sum1 += M[i][j] * x[j];
            }
            sum2 = 0;
            for (j = i + 1; j < N; ++j) {
                sum2 += M[i][j] * x[j];
            }
            x[i] = (b[i] - sum1 - sum2) / M[i][i];
        }
        iterations++;

        x_diff = vectorDifference(x, x_old);
        l0 = infiniteNorm(x_diff);
    }
    reps = iterations;

    return x;
}

void exercise_3_matrix(matrix*& M, int dim)
{
    M = new matrix(dim, dim);
    M->init(0);

    (*M)[0][0] = 5.0;
    (*M)[0][1] = -2.0;

    for (int i = 1; i < dim; ++i)
    {
        (*M)[i][i] = 5.0;
        (*M)[i][i - 1] = -2.0;
        (*M)[i][i + 1] = -2.0;
    }
    (*M)[dim - 1][dim - 1] = 5.0;
    (*M)[dim - 1][dim - 2] = -2.0;
}

void exercise_3_vector(std::vector<double>& v, int dim)
{
    v = std::vector<double>(dim);

    v[0] = 3.0;
    for (int i = 1; i < dim - 1; ++i) {
        v[i] = 1.0;
    }
    v[dim - 1] = 3.0;
}


#endif //LINEARSYS_H
