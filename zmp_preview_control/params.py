"""
Preview control parameter computation (port of get_preview_control_parameter.m).

Calculates gain matrices Gi, Gx, Gd via discrete LQR with preview horizon.
"""

import numpy as np
import control as ct

GRAVITY = 9.81


def get_preview_control_parameter(
    zc: float,
    dt: float,
    t_preview: float,
    Qe: float = 1e-4,
    R: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, np.ndarray, np.ndarray]:
    """
    Compute ZMP preview control gains.

    Parameters
    ----------
    zc : float
        LIPM (Linear Inverted Pendulum Model) height [m].
    dt : float
        Time step [s].
    t_preview : float
        Preview horizon duration [s].
    Qe : float
        Weight on ZMP tracking error (default 1e-4).
    R : float
        Weight on jerk input (default 1e-6).

    Returns
    -------
    A_d : (3, 3) ndarray
        Discrete system matrix.
    B_d : (3, 1) ndarray
        Discrete input matrix.
    C_d : (1, 3) ndarray
        Discrete output matrix.
    Gi : float
        Integral gain on ZMP error.
    Gx : (3,) ndarray
        State feedback gain.
    Gd : (N,) ndarray
        Preview feedforward gains for each future step within horizon.
    """
    # Continuous cart-table model
    A = np.array([[0, 1, 0],
                  [0, 0, 1],
                  [0, 0, 0]], dtype=float)
    B = np.array([[0],
                  [0],
                  [1]], dtype=float)
    C = np.array([[1, 0, -zc / GRAVITY]], dtype=float)
    D = 0

    # Continuous → discrete
    sys_c = ct.ss(A, B, C, D)
    sys_d = ct.c2d(sys_c, dt)
    A_d = sys_d.A
    B_d = sys_d.B
    C_d = sys_d.C

    # Augmented system for LQI (state + integrator)
    C_d_dot_A_d = C_d @ A_d
    C_d_dot_B_d = C_d @ B_d

    A_tilde = np.block([
        [1, C_d_dot_A_d],
        [np.zeros((3, 1)), A_d],
    ])
    B_tilde = np.block([
        [C_d_dot_B_d],
        [B_d],
    ])

    # LQR cost matrices
    Q = np.diag([Qe, 0.0, 0.0, 0.0])

    # Solve discrete LQR (dlqr)
    K, _P, _E = ct.dlqr(A_tilde, B_tilde, Q, R)

    Gi = float(K[0, 0])
    Gx = K[0, 1:].flatten()  # (3,)

    # Preview gains
    N = int(t_preview / dt) + 1
    Gd = np.zeros(N)
    Gd[0] = -Gi

    Ac_tilde = A_tilde - B_tilde @ K
    I_tilde = np.array([[1], [0], [0], [0]])
    X_tilde = -Ac_tilde.T @ _P @ I_tilde

    for i in range(1, N):
        val = (R + B_tilde.T @ _P @ B_tilde) ** (-1) @ B_tilde.T @ X_tilde
        Gd[i] = float(val.item()) if hasattr(val, 'item') else float(val)
        X_tilde = Ac_tilde.T @ X_tilde

    return np.asarray(A_d), np.asarray(B_d), np.asarray(C_d), Gi, Gx, Gd


def main():
    """Demo: compute and display gains for typical parameters."""
    zc = 0.22
    dt = 0.01
    t_preview = 1.0
    Qe = 1e-4
    R = 1e-6

    A_d, B_d, C_d, Gi, Gx, Gd = get_preview_control_parameter(zc, dt, t_preview, Qe, R)

    print(f"zc={zc}, dt={dt}, t_preview={t_preview}, Qe={Qe}, R={R}")
    print(f"\nA_d =\n{A_d}")
    print(f"\nB_d =\n{B_d}")
    print(f"\nC_d =\n{C_d}")
    print(f"\nGi = {Gi:.6f}")
    print(f"\nGx = {Gx}")
    print(f"\nGd (first 10) = {Gd[:10]}")
    print(f"\nGd shape = {Gd.shape}")


if __name__ == "__main__":
    main()
