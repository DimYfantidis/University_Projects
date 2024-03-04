#include <iostream>
#include <iomanip>

#include "matrix.h"
#include "exercise4.h"


int main()
{
    int iterations;
    double sum;
    double eigenvalue;

    matrix A(15, 15);
    matrix G(15, 15);

    matrix p0(15, 1);
    matrix p1(15, 1);
    matrix p2(15, 1);
    matrix p3(15, 1);
    matrix p4(15, 1);


    A_init(A);
    getGoogle(G, A);

    std::cout << "Matrix A:\n" << A << std::endl;
    std::cout << "Google matrix of A:\n" << G << std::endl;

    // 1st Question:

    std::cout << "--- FIRST QUESTION ---" << std::endl;
    std::cout << "We will now proceed to calculate the sum of each column:" << std::endl;
    for (int i = 0; i < G.numOfRows(); ++i)
    {
        sum = 0;

        for (int j = 0; j < G.numOfCols(); ++j) {
            sum += G[j][i];
        }
        std::cout << "Column " << std::setw(2) << std::setfill(' ') << i + 1 << ": " << sum << std::endl;
    }
    std::cout << "Since the sum of each column is equal to one, that means that matrix G is stochastic.\n\n" << std::endl;


    // 2nd Question:

    std::cout << "--- SECOND QUESTION ---" << std::endl;
    std::cout << "We will now apply the power method for finding the eigenvector of matrix G.\n" << std::endl;

    p0 = eigenvector(G, eigenvalue, iterations);

    std::cout << "Eigenvector \"p0\" of matrix G (" << iterations << " iterations):" << std::endl;

    Matrix::scalar_precision(10);
    std::cout << p0 << std::endl;
    Matrix::scalar_precision(DEFAULT_SCALAR_PRECISION);

    std::cout << "Eigenvalue: " << eigenvalue << "\n\n" << std::endl;


    // 3rd Question:

    std::cout << "--- THIRD QUESTION ---" << std::endl;
    std::cout << "Let's suppose that we want to increase the importance of website 11." << std::endl;
    std::cout << "In order to achieve this we will add 4 new links." << std::endl;

    std::cout << "First link:  page(1)  -> page(11)" << std::endl;
    A[0][10] = 1;

    std::cout << "Second link: page(2)  -> page(11)" << std::endl;
    A[1][10] = 1;

    std::cout << "Third link:  page(9)  -> page(11)" << std::endl;
    A[8][10] = 1;

    std::cout << "Fourth link: page(13) -> page(11)" << std::endl;
    A[12][10] = 1;

    std::cout << "We will also remove one link (from page(13) to page(14))\n" << std::endl;
    A[12][13] = 0;

    std::cout << "Matrix A will now be the following:\n" << A << std::endl;


    getGoogle(G, A);

    p1 = eigenvector(G, eigenvalue, iterations);

    std::cout << "The new eigenvector of A's new Google matrix:" << std::endl;
    Matrix::scalar_precision(10);
    std::cout << p1 << std::endl;
    Matrix::scalar_precision(DEFAULT_SCALAR_PRECISION);


    std::cout << "We can see that page(11)'s rank has increased:" << std::endl;
    std::cout << "Previous: " << p0[10][0] << std::endl;
    std::cout << "Current:  " << p1[10][0] << std::endl;
    std::cout << "\n" << std::endl;

    // 4th Question.
    std::cout << "--- FOURTH QUESTION ---" << std::endl;

    Matrix::scalar_precision(10);

    // (a)
    changeBounceChance(0.02);

    getGoogle(G, A);

    p2 = eigenvector(G, eigenvalue, iterations);

    std::cout << "New eigenvector (for q = 0.02):" << std::endl;
    std::cout << p2 << std::endl;

    // (b)
    changeBounceChance(0.6);

    getGoogle(G, A);

    p2 = eigenvector(G, eigenvalue, iterations);

    std::cout << "\nNew eigenvector (for q = 0.02):" << std::endl;
    std::cout << p2 << std::endl;

    Matrix::scalar_precision(DEFAULT_SCALAR_PRECISION);

    std::cout << "It is clear that for lower values of q, the importance of pages tends to become equal. "
              << "That happens because for lower values of q (q -> 0), adding links to another page "
              << "will hardly affect the user's behavior. "
              << "However for higher values of q (q -> 1), adding links greatly affects the user's behavior\n\n"
              << std::endl;

    // IMPORTANT: Changes q back to 0.15
    changeBounceChance(0.15);

    std::cout << "--- FIFTH QUESTION ---" << std::endl;
    std::cout << "We will first revert matrix A back to how it was in the beginning of the exercise." << std::endl;

    A_init(A);

    std::cout << "Then we will assign 3 to A(8, 11) and A(12, 11)" << std::endl;
    A[7][10] = 3;
    A[11][10] = 3;

    // New google matrix.
    getGoogle(G, A);

    p3 = eigenvector(G, eigenvalue, iterations);

    std::cout << "New eigenvector: " << std::endl;

    Matrix::scalar_precision(9);
    std::cout << p3 << std::endl;
    Matrix::scalar_precision(DEFAULT_SCALAR_PRECISION);

    std::cout << "Page(11)'s rank: " << std::endl;
    std::cout << "Initially: " << p1[10][0] << std::endl;
    std::cout << "Current:   " << p3[10][0] << std::endl;

    std::cout << "\nIt is clear that this strategy doesn't improve page(11)'s importance" << std::endl;
    std::cout << "In contrast it lowers its rank.\n\n" << std::endl;


    std::cout << "--- SIXTH QUESTION ---" << std::endl;

    std::cout << "We will revert matrix a back to its initial state." << std::endl;
    A_init(A);

    std::cout << "We will then delete all links to page(10)." << std::endl;
    A[4][9] = 0;
    A[5][9] = 0;
    A[6][9] = 0;
    A[8][9] = 0;
    A[13][9] = 0;

    std::cout << "New matrix A:\n" << A << std::endl;

    getGoogle(G, A);

    p4 = eigenvector(G, eigenvalue, iterations);

    std::cout << "New eigenvector:" << std::endl;
    Matrix::scalar_precision(9);
    std::cout << p4 << std::endl;
    Matrix::scalar_precision(DEFAULT_SCALAR_PRECISION);

    return 0;
}
