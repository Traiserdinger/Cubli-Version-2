#!/usr/bin/env python3
"""Off-line LQR design for one CubliV2 balancing axis (needs numpy only).

Same model and algorithm as firmware/lib/control/lqr.cpp, so gains computed here
match `lqr edge|vertex` on the cube:

    x = [theta, theta_dot, s, p]
    theta_ddot = a*theta + b*tau
    s_dot      = a*theta + (b + c)*tau
    p_dot      = s

Model from physical data (if you know it):
    a = M*g*l/J     b = Km/J     c = Km/Iw
    edge:   l = edge/sqrt(2),   J = M*edge^2*(1/6 + 1/2)     (solid cube approximation)
    vertex: l = edge*sqrt(3)/2, J = M*edge^2*(1/6 + 3/4)

Examples:
    python3 tools/lqr_design.py --a 69 --b 6.7 --c 167
    python3 tools/lqr_design.py --cube 0.15 --mass 1.2 --wheel-inertia 7.2e-4 --stall-torque 0.12
"""
import argparse
import math

import numpy as np


def expm(m):
    n = np.linalg.norm(m, np.inf)
    s = max(0, int(math.ceil(math.log2(n / 0.5))) if n > 0.5 else 0)
    x = m / (2 ** s)
    r = np.eye(m.shape[0])
    t = np.eye(m.shape[0])
    for k in range(1, 13):
        t = t @ x / k
        r = r + t
    for _ in range(s):
        r = r @ r
    return r


def discretize(A, B, dt):
    M = np.zeros((5, 5))
    M[:4, :4] = A * dt
    M[:4, 4:] = B * dt
    E = expm(M)
    return E[:4, :4], E[:4, 4:]


def dlqr(Ad, Bd, Q, R):
    """Structure-preserving doubling algorithm (as in the firmware)."""
    Ak, Gk, Hk = Ad.copy(), Bd @ np.linalg.inv(R) @ Bd.T, Q.copy()
    I = np.eye(4)
    for _ in range(60):
        W = np.linalg.inv(I + Gk @ Hk)
        Hn = Hk + Ak.T @ Hk @ W @ Ak
        Gn = Gk + Ak @ W @ Gk @ Ak.T
        Ak = Ak @ W @ Ak
        done = np.max(np.abs(Hn - Hk)) <= 1e-12 * np.max(np.abs(Hn))
        Hk, Gk = Hn, Gn
        if done:
            break
    K = np.linalg.inv(R + Bd.T @ Hk @ Bd) @ Bd.T @ Hk @ Ad
    return K


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--a", type=float)
    ap.add_argument("--b", type=float)
    ap.add_argument("--c", type=float)
    ap.add_argument("--cube", type=float, help="edge length [m]")
    ap.add_argument("--mass", type=float, help="total mass [kg]")
    ap.add_argument("--wheel-inertia", type=float, help="one wheel, axial [kg m^2]")
    ap.add_argument("--stall-torque", type=float, help="motor torque at full command [N m]")
    ap.add_argument("--vertex", action="store_true", help="vertex geometry instead of edge")
    ap.add_argument("--th", type=float, default=2.0, help="lqr_th [deg]")
    ap.add_argument("--rate", type=float, default=0.5, help="lqr_rate [rad/s]")
    ap.add_argument("--s", type=float, default=400.0, help="lqr_s [rad/s]")
    ap.add_argument("--p", type=float, default=600.0, help="lqr_p [rad]")
    ap.add_argument("--u", type=float, default=0.6, help="lqr_u")
    ap.add_argument("--dt", type=float, default=0.001)
    args = ap.parse_args()

    if args.cube:
        e, m, g = args.cube, args.mass, 9.81
        l, J = (e * math.sqrt(3) / 2, m * e * e * (1 / 6 + 3 / 4)) if args.vertex else (e / math.sqrt(2), m * e * e * (2 / 3))
        a, b, c = m * g * l / J, args.stall_torque / J, args.stall_torque / args.wheel_inertia
    else:
        a, b, c = args.a, args.b, args.c
    if None in (a, b, c):
        ap.error("give --a --b --c or the physical data")

    A = np.zeros((4, 4))
    A[0, 1] = 1
    A[1, 0] = a
    A[2, 0] = a
    A[3, 2] = 1
    B = np.array([[0.0], [b], [b + c], [0.0]])
    Ad, Bd = discretize(A, B, args.dt)
    xmax = [math.radians(args.th), args.rate, args.s, args.p]
    Q = np.diag([1 / v ** 2 for v in xmax])
    R = np.array([[1 / args.u ** 2]])
    K = dlqr(Ad, Bd, Q, R)
    poles = np.linalg.eigvals(Ad - Bd @ K)
    which = "vertex" if args.vertex else "edge"
    pre = "v" if args.vertex else "e"
    print(f"model: a={a:.3f} b={b:.4f} c={c:.2f}")
    print(f"gains: k1={K[0,0]:.5g} k2={K[0,1]:.5g} k3={K[0,2]:.5g} k4={K[0,3]:.5g}")
    print("closed-loop |poles| (discrete):", np.round(np.abs(poles), 6), "stable" if np.all(np.abs(poles) < 1) else "UNSTABLE")
    print("closed-loop time constants [s]:", np.round(-args.dt / np.log(np.abs(poles)), 3))
    print("\nCLI:")
    for i, k in enumerate(K[0], 1):
        print(f"param set {pre}_k{i} {k:.6g}")
    print(f"param set {pre}_a {a:.4g}\nparam set {pre}_b {b:.4g}\nparam set {pre}_c {c:.4g}\nparam save   # {which}")


if __name__ == "__main__":
    main()
