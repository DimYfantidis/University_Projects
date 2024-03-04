#include <iostream>
#include <iomanip>

#include "matrix.h"
#include "linearsys.h"


int menu()
{
    int choice;

    std::cout << "Select exercise: " << std::endl;
    std::cout << "1. Linear System Solution\n"
              << "2. Cholesky decomposition\n"
              << "3. Gauss-Seidel algorithm\n"
              << "4. Exit\n";

    do {
        std::cout << ">>> ";
        std::cin >> choice;
    } while (inputFail(std::cin) || choice < 1 || choice > 4);

    std::cin.get();
    return choice;
}

int main()
{
    int exercise;
    int iterations;

    matrix A(3, 3);
    matrix *L  = nullptr;
    matrix *Q = nullptr;

    std::vector<double> b;
    std::vector<double> solution;

    std::ofstream ofs;

    do {
        exercise = menu();
        std::cout << std::endl;

        switch (exercise) {
            case 1:
                A.readFromFile("matrix.txt");

                b = std::vector<double>();


                b.push_back(7);
                b.push_back(8);
                b.push_back(3);

                solution = gauss(A, b);

                std::cout << A << std::endl;


                for (double i: b) {
                    std::cout << "| " << i << " |" << std::endl;
                }
                std::cout << std::endl;

                for (int i = 0; i < solution.size(); ++i) {
                    std::cout << "x" << i + 1 << " = " << solution[i] << std::endl;
                }
                std::cout << "Press Enter to proceed >>>";
                std::cin.get();
                break;

            case 2:
                A.readFromFile("symmetric_matrix.txt");

                L = cholesky(A);

                std::cout << "A:\n" << A << std::endl;

                if (L != nullptr)
                {
                    std::cout << "\nLower Triangular of A (Cholesky decomposition): " << std::endl;
                    std::cout << *L << std::endl;
                    delete L;
                }
                else
                {
                    return EXIT_FAILURE;
                }

                std::cout << "Press Enter to proceed >>>";
                std::cin.get();
                break;

            case 3:
                exercise_3_matrix(Q, 10);
                exercise_3_vector(b, 10);
                solution = gaussSeidel(*Q, b, iterations);

                std::cout << "FOR N = 10" << std::endl;
                std::cout << "A * x = b, where:" << std::endl;

                std::cout << "A:\n" << *Q << std::endl;
                std::cout << "x = " << printVector(solution) << std::endl;
                std::cout << "b = " << printVector(b) << std::endl;

                std::cout << "\n(Total iterations: " << iterations << ")" << std::endl;

                delete Q;
                Q = nullptr;

                std::cout << "\nLoading Data 10.000 x 10.000 linear system ..." << std::endl;
                exercise_3_matrix(Q, 10000);
                exercise_3_vector(b, 10000);

                std::cout << "Calculating solution ..." << std::endl;
                solution = gaussSeidel(*Q, b, iterations);

                ofs.open("solution.txt", std::ios::out);
                if (ofs.is_open()) {
                    ofs << std::setprecision(4);
                    for (double xi: solution) {
                        ofs << xi << std::endl;
                    }
                }
                std::cout << "\nFOR N = 10.000" << std::endl;
                std::cout << "Printed solution in \"solutions.txt\"" << std::endl;
                std::cout << "\n(Total iterations: " << iterations << ")" << std::endl;

                delete Q;
                Q = nullptr;
                std::cout << "Press Enter to proceed >>>";
                std::cin.get();
                break;

            case 4:
                break;

            default:
                return EXIT_FAILURE;
        }
        std::cout << "\n\n";
    } while (exercise != 4);

    return EXIT_SUCCESS;
}
