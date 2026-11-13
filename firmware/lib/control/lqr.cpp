#include "lqr.h"

#include <cmath>

namespace cubli {

void axis_model_matrices(const AxisModel& m, Mat<4, 4>& A, Mat<4, 1>& B) {
    A = Mat<4, 4>::zero();
    A(0, 1) = 1.0;
    A(1, 0) = m.a;
    A(2, 0) = m.a;
    A(3, 2) = 1.0;
    B = Mat<4, 1>::zero();
    B(1, 0) = m.b;
    B(2, 0) = static_cast<double>(m.b) + static_cast<double>(m.c);
}

void discretize(const Mat<4, 4>& A, const Mat<4, 1>& B, double dt, Mat<4, 4>& Ad, Mat<4, 1>& Bd) {
    // exp([[A, B], [0, 0]] dt) = [[Ad, Bd], [0, 1]]
    Mat<5, 5> M;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) M(i, j) = A(i, j) * dt;
        M(i, 4) = B(i, 0) * dt;
    }
    const Mat<5, 5> E = expm(M);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) Ad(i, j) = E(i, j);
        Bd(i, 0) = E(i, 4);
    }
}

namespace {

double closed_loop_norm(const Mat<4, 4>& Ad, const Mat<4, 1>& Bd, const Mat<1, 4>& K) {
    Mat<4, 4> M = Ad - Bd * K;
    for (int i = 0; i < 20; ++i) {  // (A-BK)^(2^20): ~17 min at 1 kHz
        M = M * M;
        const double n = M.max_abs();
        if (!std::isfinite(n) || n > 1e12) return 1e12;
    }
    return M.max_abs();
}

}  // namespace

bool axis_gain_stabilises(const AxisModel& model, const float k[4], float dt, double* norm_out) {
    Mat<4, 4> A, Ad;
    Mat<4, 1> B, Bd;
    axis_model_matrices(model, A, B);
    discretize(A, B, dt, Ad, Bd);
    Mat<1, 4> K;
    for (int i = 0; i < 4; ++i) K(0, i) = k[i];
    const double n = closed_loop_norm(Ad, Bd, K);
    if (norm_out) *norm_out = n;
    return n < 1e-3;
}

LqrResult design_axis_lqr(const AxisModel& model, const LqrWeights& w, float dt) {
    LqrResult res;
    if (!(model.a > 0.0f && model.b > 0.0f && model.c > 0.0f && dt > 0.0f)) return res;
    if (!(w.theta_max > 0.0f && w.rate_max > 0.0f && w.wheel_max > 0.0f && w.pos_max > 0.0f && w.u_max > 0.0f))
        return res;

    Mat<4, 4> A, Ad;
    Mat<4, 1> B, Bd;
    axis_model_matrices(model, A, B);
    discretize(A, B, dt, Ad, Bd);

    Mat<4, 4> Q;
    const double xmax[4] = {w.theta_max, w.rate_max, w.wheel_max, w.pos_max};
    for (int i = 0; i < 4; ++i) Q(i, i) = 1.0 / (xmax[i] * xmax[i]);
    const double r = 1.0 / (static_cast<double>(w.u_max) * static_cast<double>(w.u_max));

    // Structure-preserving doubling: A_k, G_k = B R^-1 B', H_k -> P.
    Mat<4, 4> Ak = Ad;
    Mat<4, 4> Gk = (Bd * Bd.t()) * (1.0 / r);
    Mat<4, 4> Hk = Q;
    const Mat<4, 4> I = Mat<4, 4>::identity();
    bool converged = false;
    for (int it = 0; it < 60; ++it) {
        Mat<4, 4> Winv;
        if (!invert(I + Gk * Hk, Winv)) return res;
        const Mat<4, 4> AW = Ak * Winv;
        const Mat<4, 4> Hn = Hk + Ak.t() * Hk * Winv * Ak;
        const Mat<4, 4> Gn = Gk + AW * Gk * Ak.t();
        Ak = AW * Ak;
        const double delta = (Hn - Hk).max_abs();
        const double scale = Hn.max_abs();
        Hk = Hn;
        Gk = Gn;
        res.iterations = it + 1;
        if (!std::isfinite(scale)) return res;
        if (delta <= 1e-12 * scale) {
            converged = true;
            break;
        }
    }
    if (!converged) return res;

    const Mat<1, 1> S = Mat<1, 1>{{{r}}} + Bd.t() * Hk * Bd;
    const Mat<1, 4> K = (Bd.t() * Hk * Ad) * (1.0 / S(0, 0));
    for (int i = 0; i < 4; ++i) {
        res.k[i] = static_cast<float>(K(0, i));
        if (!std::isfinite(res.k[i])) return res;
    }
    res.closed_loop_norm = closed_loop_norm(Ad, Bd, K);
    res.ok = res.closed_loop_norm < 1e-3;
    return res;
}

}  // namespace cubli
