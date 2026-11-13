// Tiny fixed-size dense matrices (double) for offline design work: LQR, system
// identification. Never used inside the 1 kHz control step.
#pragma once

#include <cmath>
#include <cstddef>

namespace cubli {

template <int R, int C>
struct Mat {
    double a[R][C] = {};

    static Mat zero() { return Mat{}; }
    static Mat identity() {
        static_assert(R == C, "identity needs a square matrix");
        Mat m;
        for (int i = 0; i < R; ++i) m.a[i][i] = 1.0;
        return m;
    }
    double& operator()(int r, int c) { return a[r][c]; }
    double operator()(int r, int c) const { return a[r][c]; }

    Mat<C, R> t() const {
        Mat<C, R> m;
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < C; ++j) m.a[j][i] = a[i][j];
        return m;
    }
    double max_abs() const {
        double m = 0.0;
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < C; ++j) m = std::fmax(m, std::fabs(a[i][j]));
        return m;
    }
};

template <int R, int C>
Mat<R, C> operator+(const Mat<R, C>& x, const Mat<R, C>& y) {
    Mat<R, C> m;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j) m.a[i][j] = x.a[i][j] + y.a[i][j];
    return m;
}
template <int R, int C>
Mat<R, C> operator-(const Mat<R, C>& x, const Mat<R, C>& y) {
    Mat<R, C> m;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j) m.a[i][j] = x.a[i][j] - y.a[i][j];
    return m;
}
template <int R, int C>
Mat<R, C> operator*(const Mat<R, C>& x, double s) {
    Mat<R, C> m;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j) m.a[i][j] = x.a[i][j] * s;
    return m;
}
template <int R, int K, int C>
Mat<R, C> operator*(const Mat<R, K>& x, const Mat<K, C>& y) {
    Mat<R, C> m;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j) {
            double s = 0.0;
            for (int k = 0; k < K; ++k) s += x.a[i][k] * y.a[k][j];
            m.a[i][j] = s;
        }
    return m;
}

/// Gauss-Jordan inverse with partial pivoting. Returns false if singular.
template <int N>
bool invert(const Mat<N, N>& in, Mat<N, N>& out) {
    Mat<N, N> a = in;
    Mat<N, N> inv = Mat<N, N>::identity();
    for (int col = 0; col < N; ++col) {
        int piv = col;
        for (int r = col + 1; r < N; ++r)
            if (std::fabs(a.a[r][col]) > std::fabs(a.a[piv][col])) piv = r;
        if (std::fabs(a.a[piv][col]) < 1e-300) return false;
        if (piv != col) {
            for (int j = 0; j < N; ++j) {
                double t = a.a[col][j]; a.a[col][j] = a.a[piv][j]; a.a[piv][j] = t;
                t = inv.a[col][j]; inv.a[col][j] = inv.a[piv][j]; inv.a[piv][j] = t;
            }
        }
        const double d = 1.0 / a.a[col][col];
        for (int j = 0; j < N; ++j) {
            a.a[col][j] *= d;
            inv.a[col][j] *= d;
        }
        for (int r = 0; r < N; ++r) {
            if (r == col) continue;
            const double f = a.a[r][col];
            if (f == 0.0) continue;
            for (int j = 0; j < N; ++j) {
                a.a[r][j] -= f * a.a[col][j];
                inv.a[r][j] -= f * inv.a[col][j];
            }
        }
    }
    out = inv;
    return true;
}

/// Matrix exponential by scaling and squaring with a 12-term Taylor series.
template <int N>
Mat<N, N> expm(const Mat<N, N>& m) {
    int squarings = 0;
    double norm = m.max_abs() * N;
    while (norm > 0.5 && squarings < 30) {
        norm *= 0.5;
        ++squarings;
    }
    const Mat<N, N> x = m * std::ldexp(1.0, -squarings);
    Mat<N, N> result = Mat<N, N>::identity();
    Mat<N, N> term = Mat<N, N>::identity();
    for (int k = 1; k <= 12; ++k) {
        term = (term * x) * (1.0 / k);
        result = result + term;
    }
    for (int i = 0; i < squarings; ++i) result = result * result;
    return result;
}

}  // namespace cubli
