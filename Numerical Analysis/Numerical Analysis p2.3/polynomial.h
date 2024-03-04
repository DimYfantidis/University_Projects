#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include <cmath>
#include <iomanip>


// C++ implementation of a polynomial
class polynomial {
private:
    // The degree (n) of the polynomial
    int degree;

    /*
     *  Stores the terms of the polynomial;
     *  if P(x) = a_0 + a_1 * x + a_2 * x^2 + ... + a_n * x^n (n = degree),
     *  then terms = [a_0, a_1, a_2, ..., a_n] - (n+1)-sized array.
     */
    double *terms;
public:
    polynomial() = delete;

    explicit polynomial(int deg) {
        terms = new double[deg + 1];
        degree = deg;
    }

    ~polynomial() {
        delete[] terms;
    }

    // Sets a_i
    bool setTerm(int i, double a_i)
    {
        if (i < 0 || i > degree) {
            return false;
        }
        terms[i] = a_i;
        return true;
    }

    // Returns P(x)
    double getValue(double x) {
        double y = 0.0;

        for (int i = 0; i <= degree; ++i) {
            y += terms[i] * pow(x, i);
        }
        return y;
    }

    [[nodiscard]] int getDegree() const {
        return degree;
    }

    [[nodiscard]] double *getTerms() const {
        return terms;
    }
};

// Output stream operator
std::ostream& operator << (std::ostream &os, const polynomial &arg) {
    int prev_precision = os.precision();

    os << std::fixed << std::setprecision(3);

    if (std::abs(arg.getTerms()[0]) < 0.001 && std::abs(arg.getTerms()[1]) > 0.001) {
        os << arg.getTerms()[1] << " * x";
    }
    else if (std::abs(arg.getTerms()[0]) > 0.001 && std::abs(arg.getTerms()[1]) < 0.001) {
        os << arg.getTerms()[0];
    }
    else if (std::abs(arg.getTerms()[0]) > 0.001 && std::abs(arg.getTerms()[1]) > 0.001){
        os << arg.getTerms()[0]<< " " << " + (" << arg.getTerms()[1] << ") * x";
    }
    for (int i = 2; i <= arg.getDegree() - 1; ++i) {
        if (std::abs(arg.getTerms()[i]) < 0.001) {
            continue;
        }
        os << " + (" << arg.getTerms()[i] << ") * x^" << i;
    }
    os << std::fixed << std::setprecision(prev_precision);
    os << " + (" << arg.getTerms()[arg.getDegree()] << ") * x^" << arg.getDegree();
    return os;
}

#endif //POLYNOMIAL_H
