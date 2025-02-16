#ifndef MATRIX_H
#define MATRIX_H

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

#define DEFAULT_SCALAR_PRECISION 4


static int SCALAR_WIDTH = DEFAULT_SCALAR_PRECISION;

namespace Matrix
{
    void scalar_precision(int n) {
        SCALAR_WIDTH = n;
    }
}

typedef int dimension_t;  // data type for the rows and columns of

class matrix {
    /*  Each square matrix itself is a pseudo-2D array instance.
     *  The scalars are stored in a one-dimensional C-style array.
     *
     *  This offers 2 main advantages compared to a double pointer array:
     *      - slightly higher speed
     *      - optimal locality
     */
protected:    // Class members
    double *m_matrix;            // The one-dimensional array for storing the scalars of the matrix
    dimension_t m_rows;     // Number of rows
    dimension_t m_columns;  // Number of columns

protected:
    // Will be used for the overloading of the "[]" operator
    double *getRow(dimension_t i) const { return m_matrix + (i * m_columns); }

public: // Constructors -- Destructor
    matrix() = delete;

    matrix(dimension_t, dimension_t);

    matrix(const matrix&);

    ~matrix();

public: // Class Methods
    void init(double);

    bool isSymmetric() const;

    void LU(matrix&, matrix&) const;

    matrix transpose();

    void readFromFile(const char *);

    dimension_t numOfRows() const { return m_rows; }

    dimension_t numOfCols() const { return m_columns; }

public: // Operators
    matrix &operator = (const matrix &);

    virtual matrix &operator *= (double);

    virtual matrix &operator /= (double);

    // This  specific definition of the  overloaded "[]" operator
    // makes the expression M[i][j] possible, where M is a matrix
    double *operator [] (dimension_t i) const { return getRow(i); }
};

// Operators
matrix operator + (const matrix &, const matrix &);

matrix operator - (const matrix &, const matrix &);

matrix operator * (const matrix &, const matrix &);

matrix operator * (double, const matrix&);

matrix operator * (const matrix&, double);

bool operator != (const matrix&, const matrix&);

bool operator == (const matrix&, const matrix&);

std::ostream& operator << (std::ostream&, const matrix&);

std::istream& operator >> (std::istream&, const matrix&);


// --- BLUEPRINTS ---

// Constructs a matrix with RxC dimension
matrix::matrix(dimension_t R, dimension_t C)
{
    if (R < 1 || C < 1) {
        std::cerr << "Matrix construction error: non-positive number of rows or columns" << std::endl;
        exit(EXIT_FAILURE);
    }
    m_matrix = new double[(std::size_t) (R * C)];
    m_rows = R;
    m_columns = C;
}

matrix::matrix(const matrix& prototype) {
    m_rows = prototype.m_rows;
    m_columns = prototype.m_columns;
    m_matrix = new double[(unsigned int) (m_rows * m_columns)];

    dimension_t total = m_rows * m_columns;
    for (dimension_t i = 0; i < total; ++i) {
        m_matrix[i] = prototype.m_matrix[i];
    }
}

matrix::~matrix() {
    delete[] this->m_matrix;
}

// --- METHODS ---

// Initialises matrix's cells with the given argument
void matrix::init(double init_arg) {
    dimension_t total = m_rows * m_columns;
    for (dimension_t i = 0; i < total; ++i) {
        m_matrix[i] = init_arg;
    }
}

// Checks whether matrix is symmetric
bool matrix::isSymmetric() const
{
    if (this->numOfCols() != this->numOfRows()) {
        std::cerr << "Error: Non-square matrix cannot be classified regarding symmetry" << std::endl;
        return false;
    }
    dimension_t N = numOfRows();

    for (int i = 1; i < N - 1; ++i)
    {
        for (int j = i; j < N; ++j)
        {
            if ((*this)[i][j] != (*this)[j][i])
                return false;
        }
    }
    return true;
}

// decomposes the matrix in the lower and upper triangular
void matrix::LU(matrix& L, matrix& U) const
{
    if (numOfRows() != numOfCols()) {
        std::cerr << "Error: matrix not square" << std::endl;
        return;
    }
    dimension_t N = numOfRows();

    if (L.numOfRows() != N || L.numOfCols() != N)
        L = matrix(N, N);
    if (U.numOfRows() != N || U.numOfCols() != N)
        U = matrix(N, N);

    int i;
    int j;
    int k;

    for (i = 0; i < N; ++i)
    {
        for (j = 0; j < N; ++j)
        {
            if (j <= i)
            {
                U[j][i] = (*this)[j][i];

                for (k = 0; k < j; ++k) {
                    U[j][i] -= L[j][k] * U[k][i];
                }
                L[j][i] = (i == j ? 1 : 0);
            }
            else
            {
                L[j][i]=(*this)[j][i];

                for(k=0; k<=i-1; k++) {
                    L[j][i] -= L[j][k] * U[k][i];
                }
                L[j][i]/=U[i][i];
                U[j][i]=0;
            }
        }
    }
}

// Returns the transpose matrix
matrix matrix::transpose()
{
    matrix T(m_columns, m_rows);

    for (int i = 0; i < T.numOfRows(); ++i)
    {
        for (int j = 0; j < T.numOfCols(); ++j)
        {
            T[i][j] = (*this)[j][i];
        }
    }
    return T;
}

// Reads a single matrix from file (file should contain only one array)
void matrix::readFromFile(const char *filename)
{
    std::ifstream ifs;
    ifs.open(filename, std::ios::in);

    if (ifs.is_open())
    {
        ifs >> *this;
        ifs.close();
    }
    else
    {
        std::cerr << "File Error!" << std::endl;
        exit(EXIT_FAILURE);
    }
}

// --- OPERATORS ---

// Assignment operator
matrix &matrix::operator = (const matrix& arg) {
    if (this != &arg) {
        m_rows = arg.numOfRows();
        m_columns = arg.numOfCols();

        delete[] m_matrix;
        m_matrix = new double[(std::size_t) (m_rows * m_columns)];

        dimension_t total = m_rows * m_columns;

        for (dimension_t i = 0; i < total; ++i) {
            m_matrix[i] = arg.m_matrix[i];
        }
    }
    return *this;
}

// Times-equals operator with number
matrix& matrix::operator *= (double factor) {
    dimension_t total = m_rows * m_columns;
    for (dimension_t i = 0; i < total; ++i) {
        m_matrix[i] *= factor;
    }
    return *this;
}

// Division-equals operator with number
matrix& matrix::operator /= (double factor)
{
    dimension_t total = m_rows * m_columns;

    if (factor == 0.0) {
        std::cerr << "Cannot divide scalars with zero" << std::endl;
        return *this;
    }

    for (dimension_t i = 0; i < total; ++i) {
        m_matrix[i] /= factor;
    }
    return *this;
}

// Addition operator -- two matrices
matrix operator + (const matrix& one, const matrix& two)
{
    if (one.numOfRows() != two.numOfRows() ||
        one.numOfCols() != two.numOfCols()) {
        std::cerr << "Error: cannot add matrices with different dimensions" << std::endl;
        return {1, 1};
    }
    matrix sum(one.numOfRows(), one.numOfCols());

    for (dimension_t i = 0; i < sum.numOfRows(); ++i) {
        for (dimension_t j = 0; j < sum.numOfCols(); ++j) {
            sum[i][j] = one[i][j] + two[i][j];
        }
    }
    return sum;
}


// Subtraction operator -- two matrices
matrix operator - (const matrix& one, const matrix& two)
{
    if (one.numOfRows() != two.numOfRows() ||
        one.numOfCols() != two.numOfCols()) {
        std::cerr << "Error: cannot subtract matrices with different dimensions" << std::endl;
        return {1, 1};
    }
    matrix diff(one.numOfRows(), one.numOfCols());

    for (dimension_t i = 0; i < diff.numOfRows(); ++i) {
        for (dimension_t j = 0; j < diff.numOfCols(); ++j) {
            diff[i][j] = one[i][j] - two[i][j];
        }
    }
    return diff;
}

// Multiplication operator -- two matrices
matrix operator * (const matrix &one, const matrix &two)
{
    if (one.numOfCols() != two.numOfRows()) {
        std::cerr << "Error: cannot multiply matrices\n"
                  << "Columns and rows of instances do not match"
                  << std::endl;
        return {1, 1};
    }
    matrix prod(one.numOfRows(), two.numOfCols());

    prod.init(0.0);

    for (dimension_t i = 0; i < one.numOfRows(); i++) {
        for (dimension_t j = 0; j < two.numOfCols(); j++) {
            for (dimension_t k = 0; k < one.numOfCols(); k++) {
                prod[i][j] += one[i][k] * two[k][j];
            }
        }
    }
    return prod;
}


// Multiplication operator with number
matrix operator * (double factor, const matrix &arg){
    matrix prod(arg);
    prod *= factor;
    return prod;
}

matrix operator * (const matrix &arg, double factor) {
    return factor * arg;
}

// Equal-to operator
bool operator == (const matrix& one, const matrix& two)
{
    if (one.numOfRows() != two.numOfRows() ||
        two.numOfCols() != two.numOfCols()) {
        return false;
    }
    bool result = true;

    for (dimension_t i = 0; i < one.numOfRows() && result; ++i) {
        for (dimension_t j = 0; j < one.numOfCols() && result; ++j) {
            if (one[i][j] != two[i][j]) {
                result = false;
            }
        }
    }
    return result;
}

// Not-equal-to operator
bool operator != (const matrix &one, const matrix &two) {
    return !(one == two);
}

// Output stream operator
std::ostream& operator << (std::ostream &os, const matrix &arg) {
    for (dimension_t i = 0; i < arg.numOfRows(); ++i) {
        os << "|";
        for (dimension_t j = 0; j < arg.numOfCols(); ++j) {
            os << std::setw(SCALAR_WIDTH) << std::setfill(' ') << arg[i][j] << " ";
        }
        os << "|" << std::endl;
    }
    return os;
}

// Input stream operator
std::istream& operator >> (std::istream &is, const matrix &arg) {
    for (dimension_t i = 0; i < arg.numOfRows(); ++i) {
        for (dimension_t j = 0; j < arg.numOfCols(); ++j) {
            is >> arg[i][j];
        }
    }
    return is;
}

#endif // MATRIX_H