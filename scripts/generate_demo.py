#!/usr/bin/env python3
"""
Generate animated GIF of ZMP walking pattern.
"""

import sys
import os

os.environ["MPLBACKEND"] = "Agg"

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from zmp_preview_control.simulation import simulate


def main():
    zmp_x, zmp_y, com_x, com_y = simulate()

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.set_xlim(min(zmp_x) - 0.1, max(zmp_x) + 0.1)
    ax.set_ylim(min(zmp_y) - 0.1, max(zmp_y) + 0.1)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("ZMP Preview Control — Walking Pattern")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")

    (zmp_line,) = ax.plot([], [], "b-", lw=1.5, label="ZMP reference")
    (com_scat,) = ax.plot([], [], "rx", ms=3, alpha=0.7, label="CoM")
    ax.legend()

    n_frames = len(com_x)
    step = max(1, n_frames // 60)

    def init():
        zmp_line.set_data([], [])
        com_scat.set_data([], [])
        return zmp_line, com_scat

    def update(frame):
        i = min(frame * step, n_frames - 1)
        zmp_line.set_data(zmp_x[:i], zmp_y[:i])
        com_scat.set_data(com_x[:i], com_y[:i])
        return zmp_line, com_scat

    anim = animation.FuncAnimation(
        fig, update, frames=n_frames // step,
        init_func=init, interval=50, blit=True,
    )

    os.makedirs("images", exist_ok=True)
    out = "images/zmp_preview_control.gif"
    anim.save(out, writer="pillow", fps=20)
    plt.close(fig)
    print(f"GIF saved to {out}")


if __name__ == "__main__":
    main()
