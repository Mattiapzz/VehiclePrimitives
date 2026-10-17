#include "include/linalg.hh"

#include <cmath>
#include <algorithm>
#include <stdexcept>

using namespace vprim;

Matrix::Matrix(size_t rows, size_t cols, double fill)
    : rows_(rows), cols_(cols), data_(rows * cols, fill) {}

Matrix Matrix::identity(size_t n) {
    Matrix I(n, n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        I(i, i) = 1.0;
    }
    return I;
}

Matrix Matrix::zeros(size_t rows, size_t cols) {
    return Matrix(rows, cols, 0.0);
}

double& Matrix::operator()(size_t i, size_t j) {
    return data_[i * cols_ + j];
}

const double& Matrix::operator()(size_t i, size_t j) const {
    return data_[i * cols_ + j];
}

Matrix Matrix::operator*(const Matrix& other) const {
    if (cols_ != other.rows_) {
        throw std::invalid_argument("Matrix::operator*: dimension mismatch");
    }
    Matrix result(rows_, other.cols_, 0.0);
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t k = 0; k < cols_; ++k) {
            const double a_ik = (*this)(i, k);
            if (a_ik == 0.0) continue;
            for (size_t j = 0; j < other.cols_; ++j) {
                result(i, j) += a_ik * other(k, j);
            }
        }
    }
    return result;
}

Matrix Matrix::operator*(double scalar) const {
    Matrix result(rows_, cols_);
    for (size_t idx = 0; idx < data_.size(); ++idx) {
        result.data_[idx] = data_[idx] * scalar;
    }
    return result;
}

Matrix Matrix::operator+(const Matrix& other) const {
    if (rows_ != other.rows_ || cols_ != other.cols_) {
        throw std::invalid_argument("Matrix::operator+: dimension mismatch");
    }
    Matrix result(rows_, cols_);
    for (size_t idx = 0; idx < data_.size(); ++idx) {
        result.data_[idx] = data_[idx] + other.data_[idx];
    }
    return result;
}

Matrix Matrix::operator-(const Matrix& other) const {
    if (rows_ != other.rows_ || cols_ != other.cols_) {
        throw std::invalid_argument("Matrix::operator-: dimension mismatch");
    }
    Matrix result(rows_, cols_);
    for (size_t idx = 0; idx < data_.size(); ++idx) {
        result.data_[idx] = data_[idx] - other.data_[idx];
    }
    return result;
}

Matrix Matrix::transpose() const {
    Matrix result(cols_, rows_);
    for (size_t i = 0; i < rows_; ++i) {
        for (size_t j = 0; j < cols_; ++j) {
            result(j, i) = (*this)(i, j);
        }
    }
    return result;
}

void Matrix::set_block(size_t row0, size_t col0, const Matrix& block) {
    for (size_t i = 0; i < block.rows_; ++i) {
        for (size_t j = 0; j < block.cols_; ++j) {
            (*this)(row0 + i, col0 + j) = block(i, j);
        }
    }
}

Matrix Matrix::get_block(size_t row0, size_t col0, size_t nrows, size_t ncols) const {
    Matrix result(nrows, ncols);
    for (size_t i = 0; i < nrows; ++i) {
        for (size_t j = 0; j < ncols; ++j) {
            result(i, j) = (*this)(row0 + i, col0 + j);
        }
    }
    return result;
}

Matrix Matrix::solve(Matrix A, Matrix B) {
    const size_t n = A.rows_;
    if (A.cols_ != n || B.rows_ != n) {
        throw std::invalid_argument("Matrix::solve: dimension mismatch");
    }
    const size_t m = B.cols_;

    for (size_t col = 0; col < n; ++col) {
        // Partial pivoting
        size_t pivot_row = col;
        double pivot_val = std::abs(A(col, col));
        for (size_t r = col + 1; r < n; ++r) {
            const double v = std::abs(A(r, col));
            if (v > pivot_val) {
                pivot_val = v;
                pivot_row = r;
            }
        }
        if (pivot_val < 1e-14) {
            throw std::runtime_error("Matrix::solve: matrix is singular (or nearly so)");
        }
        if (pivot_row != col) {
            for (size_t j = 0; j < n; ++j) std::swap(A(col, j), A(pivot_row, j));
            for (size_t j = 0; j < m; ++j) std::swap(B(col, j), B(pivot_row, j));
        }

        const double diag = A(col, col);
        for (size_t r = col + 1; r < n; ++r) {
            const double factor = A(r, col) / diag;
            if (factor == 0.0) continue;
            for (size_t j = col; j < n; ++j) A(r, j) -= factor * A(col, j);
            for (size_t j = 0; j < m; ++j) B(r, j) -= factor * B(col, j);
        }
    }

    // Back substitution
    Matrix X(n, m, 0.0);
    for (size_t ii = 0; ii < n; ++ii) {
        const size_t row = n - 1 - ii;
        for (size_t j = 0; j < m; ++j) {
            double acc = B(row, j);
            for (size_t k = row + 1; k < n; ++k) {
                acc -= A(row, k) * X(k, j);
            }
            X(row, j) = acc / A(row, row);
        }
    }
    return X;
}

double Matrix::inf_norm() const {
    double max_row_sum = 0.0;
    for (size_t i = 0; i < rows_; ++i) {
        double row_sum = 0.0;
        for (size_t j = 0; j < cols_; ++j) {
            row_sum += std::abs((*this)(i, j));
        }
        max_row_sum = std::max(max_row_sum, row_sum);
    }
    return max_row_sum;
}

Matrix vprim::expm(const Matrix& A) {
    const size_t n = A.rows();
    const double norm = A.inf_norm();

    // Choose s such that ||A/2^s||_inf <= 0.5, then sum a Taylor series (safe
    // to machine precision for an argument that small) and square back up.
    int s = 0;
    double scale = 1.0;
    while (norm * scale > 0.5) {
        scale *= 0.5;
        ++s;
    }

    const Matrix As = A * scale;

    constexpr int kTaylorTerms = 20;
    Matrix term = Matrix::identity(n);
    Matrix sum = Matrix::identity(n);
    for (int k = 1; k <= kTaylorTerms; ++k) {
        term = (term * As) * (1.0 / static_cast<double>(k));
        sum = sum + term;
    }

    for (int i = 0; i < s; ++i) {
        sum = sum * sum;
    }
    return sum;
}
