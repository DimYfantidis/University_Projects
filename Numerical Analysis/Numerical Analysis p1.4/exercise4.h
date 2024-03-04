#ifndef EXERCISE4_H
#define EXERCISE4_H

#include "matrix.h"
#include "utils.h"


static double q = 0.15;

void A_init(matrix& A)
{
    if (A.numOfRows() != 15 || A.numOfCols() != 15) {
        std::cerr << "Wrong Dimension!" << std::endl;
        exit(EXIT_FAILURE);
    }

    A.init(0);

    A[0][1] = A[0][8] = 1;
    A[1][2] = A[1][4] = A[1][6] = 1;
    A[2][1] = A[2][5] = A[2][7] = 1;
    A[3][2] = A[3][11] = 1;
    A[4][0] = A[4][9] = 1;
    A[5][9] = A[5][10] = 1;
    A[6][9] = A[6][10] = 1;
    A[7][3] = A[7][10] = 1;
    A[8][4] = A[8][5] = A[8][9] = 1;
    A[9][12] = 1;
    A[10][14] = 1;
    A[11][6] = A[11][7] = A[11][10] = 1;
    A[12][8] = A[12][13] = 1;
    A[13][9] = A[13][10] = A[13][12] = A[13][14] = 1;
    A[14][11] = A[14][13] = 1;
}

void getGoogle(matrix& G, const matrix& A)
{
    if (G.numOfRows() != 15 || G.numOfCols() != 15) {
        std::cerr << "Wrong Dimension!" << std::endl;
        exit(EXIT_FAILURE);
    }
    dimension_t N = 15;

    double nj;

    for (int i = 0; i < G.numOfRows(); ++i)
    {
        for (int j = 0; j < G.numOfCols(); ++j)
        {
            nj = 0;
            for (int k = 0; k < A.numOfCols(); ++k) {
                nj += A[j][k];
            }
            G[i][j] = (q / N) + (A[j][i] * (1 - q)) / nj;
        }
    }
}

matrix eigenvector(const matrix &G, double& eig, int& reps)
{
    int i;
    int j;
    int k;

    double eigenvalue = 1;
    double eigenvalue_prev = 0;

    matrix p(G.numOfRows(), 1);
    matrix p_old(G.numOfRows(), 1);


    p_old.init(1.0);

    for (i = 0; !isEqual(eigenvalue, eigenvalue_prev); ++i)
    {
        p = G * p_old;
        eigenvalue = p[0][0];
        eigenvalue_prev = p_old[0][0];
        p_old = p;

        for (j = 0; isEqual(p[j][0], 0.0); ++j);

        p /= p[j][0];
    }
    eig = eigenvalue;
    reps = i;

    // Normalizes the vector
    double l1 = 0;
    for (i = 0; i < p.numOfRows(); ++i) {
        l1 += std::abs(p[i][0]);
    }

    p /= l1;

    return p;
}

void changeBounceChance(double q_new)
{
    if (q_new <= 0.0 || q_new >= 1.0)     {
        std::cerr << "Wrong Input!" << std::endl;
        exit(EXIT_FAILURE);
    }
    q = q_new;
}

#endif //EXERCISE4_H
