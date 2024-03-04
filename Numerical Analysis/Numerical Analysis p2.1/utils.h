#ifndef UTILS_H
#define UTILS_H

#include "matrix.h"
#include <vector>
#include <utility>


struct five_tuple {
    double a;
    double b;
    double c;
    double d;
    std::pair<double, double> x;

    five_tuple(double a, double b, double c, double d, std::pair<double, double>x) {
        this->a = a;
        this->b = b;
        this->c = c;
        this->d = d;
        this->x.first = x.first;
        this->x.second = x.second;
    }
};

int precision(double x)
{
    int res;
    x = std::abs(x);

    if (x == 0) {
        return 18;
    }

    for (res = 0; x < 1; ++res) {
        x *= 10;
    }
    return res - 1;
}

std::vector<double> gauss(const matrix& M, const std::vector<double>& b)
{
    if (M.numOfRows() != M.numOfCols()) {
        std::cerr << "Matrix error: square matrix expected" << std::endl;
        return {1, 1};
    }
    if (b.size() != M.numOfRows()) {
        std::cerr << "Vector error: parse a proper vector as argument" << std::endl;
        return {1, 1};
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

    for (i = 0; i < N; ++i)
    {
        q = M_aug[N - 1 - i][N];

        for (s = j = 0; j < i; ++j) {
            s += M_aug[N - 1 - i][N - 1 - j] * sol[N - 1 - j];
        }
        sol[N - 1 - i] = (q - s) / M_aug[N - 1 - i][N - 1 - i];
    }

    return sol;
}

void exportVectorToFile(const char *filename, const std::vector<std::pair<double, double>>& v)
{
    std::ofstream ofs(filename, std::ios::out);

    if (ofs.is_open()) {
        for (const auto& point: v) {
            ofs << point.first << ' ' << point.second << ' ' << std::endl;
        }
    }
    else {
        std::cerr << "File Error: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }
}


#endif //UTILS_H
