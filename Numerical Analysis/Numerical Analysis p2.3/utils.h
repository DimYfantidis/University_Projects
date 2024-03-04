#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include "matrix.h"


void init_closes(const char *filename, std::vector<std::pair<double, double>>& v)
{
    std::ifstream ifs(filename, std::ios::in);
    std::string discard;

    double day;
    double close;

    std::pair<double, double> p;


    if (ifs.is_open())
    {
        std::getline(ifs, discard);

        while (!ifs.eof()) {
            ifs >> day >> close;

            p.first = day;
            p.second = close;

            v.push_back(p);

            std::getline(ifs, discard);
        }
    }
    else
    {
        std::cerr << "File Error: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }
}

void init_next_days(const char *filename, std::vector<double>& v)
{
    std::ifstream ifs(filename, std::ios::in);
    std::string discard;

    double day;

    if (ifs.is_open())
    {
        std::getline(ifs, discard);

        while (!ifs.eof()) {
            ifs >> day;
            v.push_back(day);
            std::getline(ifs, discard);
        }
    }
    else
    {
        std::cerr << "File Error: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }
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

#endif //UTILS_H
