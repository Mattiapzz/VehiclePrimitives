#pragma once

#ifndef VPRIM_LINALG_HH
#define VPRIM_LINALG_HH

#include <vector>
#include <cstddef>

namespace vprim {

    // Minimal dense matrix (row-major), sized exactly for the small, fixed
    // linear-algebra needs of the motion-sickness filter's Van Loan construction
    // (state-space matrices of a low-order weighting filter, augmented with a
    // few derivative states -> at most a couple dozen rows/cols). Not a
    // general-purpose linear algebra library.
    class Matrix {
    private:
        size_t rows_{0};
        size_t cols_{0};
        std::vector<double> data_;

    public:
        Matrix() = default;
        Matrix(size_t rows, size_t cols, double fill = 0.0);

        [[nodiscard]] static Matrix identity(size_t n);
        [[nodiscard]] static Matrix zeros(size_t rows, size_t cols);

        [[nodiscard]] size_t rows() const { return rows_; }
        [[nodiscard]] size_t cols() const { return cols_; }

        double& operator()(size_t i, size_t j);
        const double& operator()(size_t i, size_t j) const;

        [[nodiscard]] Matrix operator*(const Matrix& other) const;
        [[nodiscard]] Matrix operator*(double scalar) const;
        [[nodiscard]] Matrix operator+(const Matrix& other) const;
        [[nodiscard]] Matrix operator-(const Matrix& other) const;
        [[nodiscard]] Matrix transpose() const;

        // Writes `block` into this matrix starting at (row0, col0).
        void set_block(size_t row0, size_t col0, const Matrix& block);

        // Reads an (nrows x ncols) sub-matrix starting at (row0, col0).
        [[nodiscard]] Matrix get_block(size_t row0, size_t col0, size_t nrows, size_t ncols) const;

        // Infinity norm (max absolute row sum) -- used to choose the scaling
        // factor in expm().
        [[nodiscard]] double inf_norm() const;

        // Solves A * X = B for X (A square n x n, B is n x m) via Gaussian
        // elimination with partial pivoting. Throws if A is singular.
        [[nodiscard]] static Matrix solve(Matrix A, Matrix B);
    };

    // Matrix exponential via scaling-and-squaring with a scaled Taylor series.
    // Not a Pade-based implementation (see motionsick/motion_sickness_primitive_roadmap.md
    // section 4 for the tradeoff): the matrices this primitive builds (filter
    // state-space matrices augmented with a few polynomial-derivative states)
    // have modest norms by construction, so scaling the matrix down until its
    // infinity norm is <= 0.5 and summing ~20 Taylor terms before squaring back
    // up is accurate to machine precision without needing a Pade linear solve.
    [[nodiscard]] Matrix expm(const Matrix& A);

} // namespace vprim

#endif // VPRIM_LINALG_HH
