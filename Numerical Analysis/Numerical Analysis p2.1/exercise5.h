#ifndef EXERCISE5_H
#define EXERCISE5_H

#include <vector>
#include "utils.h"
#include "matrix.h"
#include "polynomial.h"


/*
 *  Lagrange polynomial approximation of function.
 *  - 1st param: x -> independent variable
 *  - 2nd param: samples -> n-sized list of (x_i, f(x_i)) sample pairs (i = 0, 1, ..., n - 1)
 *  - returns: φ(x) where φ is the interpolation of f.
 */
double polynomialLagrange(const double x, const std::vector<std::pair<double, double>>& samples)
{
    std::size_t N = samples.size();
    double p_x = 0.0;
    double L_i;
    double x_i;
    double x_j;
    double y_i;

    for (int i = 0; i < N; ++i)
    {
        L_i = 1.0;
        x_i = samples[i].first;
        y_i = samples[i].second;

        for (int j = 0; j < N; ++j) {
            if (j != i) {
                x_j = samples[j].first;
                L_i *= (x - x_j) / (x_i - x_j);
            }
        }
        p_x += y_i * L_i;
    }
    return p_x;
}

/*
 *  Approximation of function using splines.
 *  Instead of a function, this is achieved using the following class.
 */
class Splines {
public:
    std::vector<five_tuple *>* S;

public:
    Splines() = delete;

    /*
     *  Takes an n-sized set of (x, f(x)) samples and
     *  creates (n-1) cubic splines between x_0 and x_(n-1).
     *  The class field 'S' stores the cubic polynomials as
     *  an
     */
    explicit Splines(const std::vector<std::pair<double, double>>& samples)
    {
        std::size_t N = samples.size();
        auto *a = new double[N];

        for (std::size_t i = 0; i < N; ++i) {
            a[i] = samples[i].second;
        }

        auto *b = new double[N - 1];
        auto *d = new double[N - 1];

        auto *h = new double[N - 1];
        for (std::size_t i = 0; i < N - 1; ++i) {
            h[i] = samples[i + 1].first - samples[i].first;
        }

        auto *alpha = new double[N - 1];
        for (std::size_t i = 1; i < N - 1; ++i) {
            alpha[i] = (3 / h[i]) * (a[i + 1] - a[i]) - (3 / h[i - 1]) * (a[i] - a[i - 1]);
        }

        auto *c = new double[N];
        auto *l = new double[N];
        auto *m = new double[N];
        auto *z = new double[N];

        l[0] = 1.0;
        m[0] = 0.0;
        z[0] = 0.0;

        for (std::size_t i = 1; i < N - 1; ++i) {
            l[i] = 2 * (samples[i + 1].first - samples[i - 1].first) - h[i - 1] * m[i - 1];
            m[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }
        l[N - 1] = 1.0;
        z[N - 1] = 0.0;
        c[N - 1] = 0.0;

        for (long long j = N - 2; j >= 0; --j) {
            c[j] = z[j] - m[j] * c[j + 1];
            b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2 * c[j]) / 3;
            d[j] = (c[j + 1] - c[j]) / (3 * h[j]);
        }

        S = new std::vector<five_tuple *>(N - 1);

        for (std::size_t i = 0; i < S->size(); ++i) {
            (*S)[i] = new five_tuple(a[i], b[i], c[i], d[i], {samples[i].first, samples[i + 1].first});
        }
        delete[] a;
        delete[] alpha;
        delete[] b;
        delete[] c;
        delete[] d;
        delete[] l;
        delete[] m;
        delete[] z;
    }

    ~Splines()
    {
        for (five_tuple *spline : *S) {
            delete spline;
        }
        delete S;
    }

    [[nodiscard]] double getValue(double x) const
    {
        double result = 0.0;
        std::size_t last = S->size() - 1;

        if (result < (*S)[0]->x.first || result > (*S)[last]->x.second) {
            std::cerr << "Error: Input out of bounds" << std::endl;
            return std::nan("0");
        }

        for (auto s : *S)
        {
            if (x >= s->x.first && x <= s->x.second)
            {
                result = s->a + s->b * (x - s->x.first) + s->c * pow(x - s->x.first, 2) + s->d * pow(x - s->x.first, 3);
                break;
            }
        }
        return result;
    }
};

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
        exit(-1);
    }

    for (int i = 0; i <= phi->getDegree(); ++i) {
        phi->setTerm(i, terms[i]);
    }

    return phi;
}

#endif //EXERCISE5_H
