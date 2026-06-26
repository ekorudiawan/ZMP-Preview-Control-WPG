#!/usr/bin/env python3
"""
ZMP Preview Control — CLI.

Usage:
    zmp-preview-ctl params [--zc F] [--dt F] [--tprev F] [--Qe F] [--R F]
    zmp-preview-ctl simulate [--output PATH] [--footsteps JF] ...
    zmp-preview-ctl gif [--output PATH] [--footsteps JF] ...
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from .params import get_preview_control_parameter
from .simulation import simulate


def cmd_params(args: argparse.Namespace) -> None:
    A_d, B_d, C_d, Gi, Gx, Gd = get_preview_control_parameter(
        args.zc, args.dt, args.tprev, args.Qe, args.R
    )
    print(f"Gi = {Gi:.6f}")
    print(f"Gx = {Gx}")
    print(f"Gd (N={len(Gd)}), first 5 = {Gd[:5]}")
    print(f"\nA_d =\n{A_d}")
    print(f"\nB_d =\n{B_d}")
    print(f"\nC_d =\n{C_d}")


def cmd_simulate(args: argparse.Namespace) -> None:
    footsteps = json.loads(args.footsteps) if args.footsteps else None
    _, _, _, _ = simulate(
        zc=args.zc, dt=args.dt, t_step=args.tstep,
        t_preview=args.tprev, Qe=args.Qe, R=args.R,
        footsteps=footsteps,
        save_plot=str(args.output),
    )
    print(f"Plot saved to {args.output}")


def cmd_gif(args: argparse.Namespace) -> None:
    """Generate animated GIF showing walking pattern evolution."""
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt

    footsteps = json.loads(args.footsteps) if args.footsteps else None
    zmp_x, zmp_y, com_x, com_y = simulate(
        zc=args.zc, dt=args.dt, t_step=args.tstep,
        t_preview=args.tprev, Qe=args.Qe, R=args.R,
        footsteps=footsteps,
    )

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.set_xlim(min(zmp_x) - 0.1, max(zmp_x) + 0.1)
    ax.set_ylim(min(zmp_y) - 0.1, max(zmp_y) + 0.1)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("ZMP Preview Control — Walking Pattern")
    ax.grid(True, alpha=0.3)
    ax.axis("equal")

    zmp_line, = ax.plot([], [], "b-", lw=1.5, label="ZMP reference")
    com_scat, = ax.plot([], [], "rx", ms=3, alpha=0.7, label="CoM")
    ax.legend()

    def init():
        zmp_line.set_data([], [])
        com_scat.set_data([], [])
        return zmp_line, com_scat

    n_frames = len(com_x)
    step = max(1, n_frames // 60)  # target ~60 frames

    def update(frame):
        i = min(frame * step, n_frames - 1)
        zmp_line.set_data(zmp_x[:i], zmp_y[:i])
        com_scat.set_data(com_x[:i], com_y[:i])
        return zmp_line, com_scat

    anim = animation.FuncAnimation(
        fig, update, frames=n_frames // step, init_func=init,
        interval=50, blit=True,
    )

    out = Path(args.output)
    anim.save(str(out), writer="pillow", fps=20)
    plt.close(fig)
    print(f"GIF saved to {out}")


def main():
    parser = argparse.ArgumentParser(description="ZMP Preview Control")
    parser.add_argument("--zc", type=float, default=0.22, help="LIPM height [m]")
    parser.add_argument("--dt", type=float, default=0.01, help="Time step [s]")
    parser.add_argument("--tstep", type=float, default=0.6, help="Step duration [s]")
    parser.add_argument("--tprev", type=float, default=1.0, help="Preview horizon [s]")
    parser.add_argument("--Qe", type=float, default=1e-4, help="ZMP error weight")
    parser.add_argument("--R", type=float, default=1e-6, help="Jerk weight")
    parser.add_argument("--footsteps", type=str, default=None,
                        help="JSON list of [x,y,theta] footsteps")
    parser.add_argument("--output", type=str, default="zmp_preview.png",
                        help="Output path")

    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("params", help="Compute preview control gains")
    sub.add_parser("simulate", help="Run simulation and save plot")
    sub.add_parser("gif", help="Generate animated walking GIF")

    args = parser.parse_args()
    if args.command == "params":
        cmd_params(args)
    elif args.command == "simulate":
        cmd_simulate(args)
    elif args.command == "gif":
        cmd_gif(args)


if __name__ == "__main__":
    main()
