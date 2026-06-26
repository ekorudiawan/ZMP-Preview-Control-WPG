"""
Simulation module — ZMP trajectory generation + preview control.

Port of MATLAB sources:
    create_zmp_trajectory.m
    calc_preview_control.m
    test_calc_preview_control.m
"""

from typing import Optional
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .params import get_preview_control_parameter


def create_zmp_trajectory(
    footsteps: list[list[float]], dt: float, t_step: float
) -> tuple[list[float], list[float]]:
    """
    Build reference ZMP trajectories from footstep plan.

    Each footstep = [x, y, theta] (theta unused for ZMP).
    """
    zmp_x: list[float] = []
    zmp_y: list[float] = []
    steps_per_phase = int(t_step / dt)
    for i, (fx, fy, _) in enumerate(footsteps):
        count = steps_per_phase
        # Last step doesn't repeat
        zmp_x.extend([fx] * count)
        zmp_y.extend([fy] * count)
    return zmp_x, zmp_y


def calc_preview_control(
    zmp_x: list[float],
    zmp_y: list[float],
    dt: float,
    t_preview: float,
    t_calc: float,
    A_d: np.ndarray,
    B_d: np.ndarray,
    C_d: np.ndarray,
    Gi: float,
    Gx: np.ndarray,
    Gd: np.ndarray,
) -> tuple[list[float], list[float]]:
    """
    Run preview control simulation to compute CoM trajectory.

    Returns (com_x, com_y) lists.
    """
    N = int(t_calc / dt)
    preview_steps = int(t_preview / dt)

    x_x = np.zeros((3, 1))
    x_y = np.zeros((3, 1))

    com_x: list[float] = []
    com_y: list[float] = []

    for i in range(N):
        # Output: y = C_d @ x  (current ZMP)
        y_x = float((C_d @ x_x).item())
        y_y = float((C_d @ x_y).item())

        e_x = zmp_x[i] - y_x
        e_y = zmp_y[i] - y_y

        # Preview summation
        preview_x = 0.0
        preview_y = 0.0
        for j in range(preview_steps):
            idx = i + j
            if idx < len(zmp_x):
                preview_x += Gd[j] * zmp_x[idx]
                preview_y += Gd[j] * zmp_y[idx]

        # Control signal (jerk)
        u_x = -Gi * e_x - float(Gx @ x_x.flatten()) - preview_x
        u_y = -Gi * e_y - float(Gx @ x_y.flatten()) - preview_y

        # Update state
        x_x = A_d @ x_x + B_d * u_x
        x_y = A_d @ x_y + B_d * u_y

        com_x.append(float(x_x[0, 0]))
        com_y.append(float(x_y[0, 0]))

    return com_x, com_y


def simulate(
    zc: float = 0.22,
    dt: float = 0.01,
    t_step: float = 0.6,
    t_preview: float = 1.0,
    Qe: float = 1e-4,
    R: float = 1e-6,
    footsteps: Optional[list[list[float]]] = None,
    save_plot: Optional[str] = None,
    show: bool = False,
) -> tuple[list[float], list[float], list[float], list[float]]:
    """
    Full end-to-end simulation.

    Returns (zmp_x, zmp_y, com_x, com_y).
    """
    if footsteps is None:
        footsteps = [
            [0.0, 0.0, 0.0],
            [0.2, 0.06, 0.0],
            [0.4, -0.06, 0.0],
            [0.6, 0.09, 0.0],
            [0.8, -0.03, 0.0],
            [1.3, 0.09, 0.0],
            [1.7, -0.03, 0.0],
            [1.9, 0.09, 0.0],
            [2.0, -0.03, 0.0],
        ]

    # Auto-compute simulation time
    t_calc = len(footsteps) * t_step - t_preview - dt

    # Generate ZMP reference
    zmp_x, zmp_y = create_zmp_trajectory(footsteps, dt, t_step)

    # Compute gains
    A_d, B_d, C_d, Gi, Gx, Gd = get_preview_control_parameter(
        zc, dt, t_preview, Qe, R
    )

    # Run preview control
    com_x, com_y = calc_preview_control(
        zmp_x, zmp_y, dt, t_preview, t_calc, A_d, B_d, C_d, Gi, Gx, Gd
    )

    # Plot
    time_axis = np.arange(len(zmp_x)) * dt
    com_time = np.arange(len(com_x)) * dt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("ZMP Preview Control — Walking Pattern Generation", fontsize=14)

    ax = axes[0, 0]
    ax.plot(time_axis[:len(zmp_x)], zmp_x[:len(zmp_x)], "b-", label="ZMP x", linewidth=1)
    ax.plot(com_time, com_x, "r--", label="CoM x", linewidth=1)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("X [m]")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(time_axis[:len(zmp_y)], zmp_y[:len(zmp_y)], "b-", label="ZMP y", linewidth=1)
    ax.plot(com_time, com_y, "r--", label="CoM y", linewidth=1)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Y [m]")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(zmp_x, zmp_y, "b-", label="ZMP", linewidth=1.5)
    ax.plot(com_x, com_y, "rx", label="CoM", markersize=2, alpha=0.7)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title("ZMP vs CoM Trajectory")
    ax.axis("equal")

    ax = axes[1, 1]
    ax.plot(com_time, com_x, "r-", label="CoM x", linewidth=1)
    ax.plot(com_time, com_y, "g-", label="CoM y", linewidth=1)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Position [m]")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title("CoM Position")

    plt.tight_layout()
    if save_plot:
        fig.savefig(save_plot, dpi=150, bbox_inches="tight")
    if show:
        plt.show()
    plt.close(fig)

    return zmp_x, zmp_y, com_x, com_y


def main():
    """Demo simulation."""
    zmp_x, zmp_y, com_x, com_y = simulate(
        save_plot="images/python_zmp_com.png",
        show=False,
    )
    print(f"Generated {len(com_x)} CoM samples.")
    print("Plot saved to images/python_zmp_com.png")


if __name__ == "__main__":
    main()
