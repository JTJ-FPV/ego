#include <Eigen/Eigen>


// The banded system class is used for solving
// banded linear system Ax=b efficiently.
// A is an N*N band matrix with lower band width lowerBw
// and upper band width upperBw.
// Banded LU factorization has O(N) time complexity.
class BandedSystem
{
public:
// The size of A, as well as the lower/upper
// banded width p/q are needed
inline void create(const int &n, const int &p, const int &q)
{
    // In case of re-creating before destroying
    destroy();
    N = n;
    lowerBw = p;
    upperBw = q;
    int actualSize = N * (lowerBw + upperBw + 1);
    ptrData = new double[actualSize];
    std::fill_n(ptrData, actualSize, 0.0);
    return;
}

inline void destroy()
{
    if (ptrData != nullptr)
    {
        delete[] ptrData;
        ptrData = nullptr;
    }
    return;
}

inline void operator=(const BandedSystem &bs)
{
    ptrData = nullptr;
    create(bs.N, bs.lowerBw, bs.upperBw);
    memcpy(ptrData, bs.ptrData, N * (lowerBw + upperBw + 1) * sizeof(double));
}

private:
int N;
int lowerBw;
int upperBw;
double *ptrData = nullptr;

public:
// Reset the matrix to zero
inline void reset(void)
{
    std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
    return;
}

// The band matrix is stored as suggested in "Matrix Computation"
inline const double &operator()(const int &i, const int &j) const
{
    return ptrData[(i - j + upperBw) * N + j];
}

inline double &operator()(const int &i, const int &j)
{
    return ptrData[(i - j + upperBw) * N + j];
}

// This function conducts banded LU factorization in place
// Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
inline void factorizeLU()
{
    int iM, jM;
    double cVl;
    for (int k = 0; k <= N - 2; k++)
    {
        iM = std::min(k + lowerBw, N - 1);
        cVl = operator()(k, k);
        for (int i = k + 1; i <= iM; i++)
        {
            if (operator()(i, k) != 0.0)
            {
                operator()(i, k) /= cVl;
            }
        }
        jM = std::min(k + upperBw, N - 1);
        for (int j = k + 1; j <= jM; j++)
        {
            cVl = operator()(k, j);
            if (cVl != 0.0)
            {
                for (int i = k + 1; i <= iM; i++)
                {
                    if (operator()(i, k) != 0.0)
                    {
                        operator()(i, j) -= operator()(i, k) * cVl;
                    }
                }
            }
        }
    }
    return;
}

// This function solves Ax=b, then stores x in b
// The input b is required to be N*m, i.e.,
// m vectors to be solved.
inline void solve(const Eigen::MatrixXd &b, Eigen::MatrixXd &res) const
{
    // Copy b into res so the input remains unchanged.
    res = b;

    int iM;
    // Forward substitution through the lower triangular factor L.
    for (int j = 0; j <= N - 1; j++)
    {
        iM = std::min(j + lowerBw, N - 1);
        for (int i = j + 1; i <= iM; i++)
        {
            if (operator()(i, j) != 0.0)
            {
                res.row(i) -= operator()(i, j) * res.row(j);
            }
        }
    }

    // Back substitution through the upper triangular factor U.
    for (int j = N - 1; j >= 0; j--)
    {
        res.row(j) /= operator()(j, j); // Divide by the diagonal entry U[j][j].
        iM = std::max(0, j - upperBw);
        for (int i = iM; i <= j - 1; i++)
        {
            if (operator()(i, j) != 0.0)
            {
                res.row(i) -= operator()(i, j) * res.row(j);
            }
        }
    }
}

// This function solves ATx=b, then stores x in b
// The input b is required to be N*m, i.e.,
// m vectors to be solved.
inline void solveAdj(const Eigen::MatrixXd &b, Eigen::MatrixXd &res) const 
{
    // Copy b into res so the input remains unchanged.
    res = b;

    int iM;
    // Forward substitution through the transposed lower factor L^T.
    for (int j = 0; j <= N - 1; j++) 
    {
        // Solve the diagonal entry.
        res.row(j) /= operator()(j, j);
        
        // Update columns to the right using the transposed upper portion.
        iM = std::min(j + upperBw, N - 1);
        for (int i = j + 1; i <= iM; i++) 
        {
            if (operator()(j, i) != 0.0) 
            {
                res.row(i) -= operator()(j, i) * res.row(j);
            }
        }
    }

    // Back substitution through the transposed upper factor U^T.
    for (int j = N - 1; j >= 0; j--) 
    {
        // Update columns to the left using the transposed lower portion.
        iM = std::max(0, j - lowerBw);
        for (int i = iM; i <= j - 1; i++) 
        {
            if (operator()(j, i) != 0.0) 
            {
                res.row(i) -= operator()(j, i) * res.row(j);
            }
        }
    }
}

EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
