#!/usr/bin/env python3
"""Visualize a trajectory CSV exported by ego::Ego::saveTrajectoryCsv()."""

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List


REQUIRED_COLUMNS = (
    "t",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "ax",
    "ay",
    "az",
    "jx",
    "jy",
    "jz",
    "occupied",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot a 2D or 3D trajectory CSV exported by the ego library."
    )
    parser.add_argument("csv_file", type=Path, help="trajectory CSV file")
    parser.add_argument(
        "--mode",
        choices=("auto", "2d", "3d"),
        default="auto",
        help="plot mode; auto detects a constant-z trajectory (default: auto)",
    )
    parser.add_argument("--save", type=Path, help="save the figure as PNG/PDF/SVG")
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="do not open an interactive window (useful on a robot or CI server)",
    )
    parser.add_argument("--title", default="EGO trajectory", help="figure title")
    return parser.parse_args()


def load_csv(path: Path) -> Dict[str, List[float]]:
    if not path.is_file():
        raise FileNotFoundError("trajectory CSV does not exist: {}".format(path))

    data = {name: [] for name in REQUIRED_COLUMNS}
    with path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = [name for name in REQUIRED_COLUMNS if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError("CSV is missing columns: {}".format(", ".join(missing)))

        for line_number, row in enumerate(reader, start=2):
            try:
                for name in REQUIRED_COLUMNS:
                    data[name].append(float(row[name]))
            except (TypeError, ValueError) as error:
                raise ValueError(
                    "invalid numeric value at CSV line {}: {}".format(line_number, error)
                ) from error

    if not data["t"]:
        raise ValueError("trajectory CSV contains no samples")
    return data


def vector_norm(x: List[float], y: List[float], z: List[float]) -> List[float]:
    return [math.sqrt(xi * xi + yi * yi + zi * zi) for xi, yi, zi in zip(x, y, z)]


def set_3d_equal_axes(axis, x: List[float], y: List[float], z: List[float]) -> None:
    centers = ((min(x) + max(x)) * 0.5,
               (min(y) + max(y)) * 0.5,
               (min(z) + max(z)) * 0.5)
    radius = max(max(x) - min(x), max(y) - min(y), max(z) - min(z)) * 0.5
    radius = max(radius, 1e-6)
    axis.set_xlim(centers[0] - radius, centers[0] + radius)
    axis.set_ylim(centers[1] - radius, centers[1] + radius)
    axis.set_zlim(centers[2] - radius, centers[2] + radius)


def plot_trajectory(data: Dict[str, List[float]], mode: str, title: str):
    import matplotlib.pyplot as plt

    t = data["t"]
    x, y, z = data["x"], data["y"], data["z"]
    speed = vector_norm(data["vx"], data["vy"], data["vz"])
    acceleration = vector_norm(data["ax"], data["ay"], data["az"])
    jerk = vector_norm(data["jx"], data["jy"], data["jz"])
    occupied_indices = [i for i, value in enumerate(data["occupied"]) if value >= 0.5]

    if mode == "auto":
        mode = "2d" if max(z) - min(z) < 1e-9 else "3d"

    figure = plt.figure(figsize=(13, 8), constrained_layout=True)
    grid = figure.add_gridspec(3, 2, width_ratios=(1.45, 1.0))

    if mode == "3d":
        # Older Matplotlib versions require an explicit import to register "3d".
        import mpl_toolkits.mplot3d  # noqa: F401
        trajectory_axis = figure.add_subplot(grid[:, 0], projection="3d")
        trajectory_axis.plot(x, y, z, color="tab:blue", linewidth=1.5, alpha=0.75)
        colored_path = trajectory_axis.scatter(x, y, z, c=t, cmap="viridis", s=9)
        trajectory_axis.scatter(x[0], y[0], z[0], color="limegreen", s=80,
                                marker="o", label="start")
        trajectory_axis.scatter(x[-1], y[-1], z[-1], color="red", s=90,
                                marker="*", label="goal")
        if occupied_indices:
            trajectory_axis.scatter(
                [x[i] for i in occupied_indices],
                [y[i] for i in occupied_indices],
                [z[i] for i in occupied_indices],
                color="crimson", marker="x", s=40, label="occupied",
            )
        trajectory_axis.set_xlabel("x [m]")
        trajectory_axis.set_ylabel("y [m]")
        trajectory_axis.set_zlabel("z [m]")
        set_3d_equal_axes(trajectory_axis, x, y, z)
    else:
        trajectory_axis = figure.add_subplot(grid[:, 0])
        trajectory_axis.plot(x, y, color="tab:blue", linewidth=1.5, alpha=0.75)
        colored_path = trajectory_axis.scatter(x, y, c=t, cmap="viridis", s=10)
        trajectory_axis.scatter(x[0], y[0], color="limegreen", s=80,
                                marker="o", label="start", zorder=3)
        trajectory_axis.scatter(x[-1], y[-1], color="red", s=90,
                                marker="*", label="goal", zorder=3)
        if occupied_indices:
            trajectory_axis.scatter(
                [x[i] for i in occupied_indices],
                [y[i] for i in occupied_indices],
                color="crimson", marker="x", s=40, label="occupied", zorder=4,
            )
        trajectory_axis.set_xlabel("x [m]")
        trajectory_axis.set_ylabel("y [m]")
        trajectory_axis.set_aspect("equal", adjustable="box")

    trajectory_axis.set_title("{} ({})".format(title, mode.upper()))
    trajectory_axis.grid(True, alpha=0.3)
    trajectory_axis.legend(loc="best")
    colorbar = figure.colorbar(colored_path, ax=trajectory_axis, shrink=0.75, pad=0.08)
    colorbar.set_label("time [s]")

    series = (
        (speed, "speed [m/s]", "tab:green"),
        (acceleration, "acceleration [m/s²]", "tab:orange"),
        (jerk, "jerk [m/s³]", "tab:purple"),
    )
    for row, (values, label, color) in enumerate(series):
        axis = figure.add_subplot(grid[row, 1])
        axis.plot(t, values, color=color, linewidth=1.3)
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.3)
        if row == len(series) - 1:
            axis.set_xlabel("time [s]")

    return figure, mode, len(occupied_indices)


def main() -> int:
    args = parse_args()

    if args.no_show:
        import matplotlib
        matplotlib.use("Agg")

    data = load_csv(args.csv_file)
    figure, detected_mode, occupied_count = plot_trajectory(data, args.mode, args.title)

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(str(args.save), dpi=180, bbox_inches="tight")
        print("saved figure: {}".format(args.save))

    print("mode: {}, samples: {}, occupied samples: {}".format(
        detected_mode, len(data["t"]), occupied_count
    ))

    if not args.no_show:
        import matplotlib.pyplot as plt
        plt.show()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError) as error:
        print("error: {}".format(error))
        raise SystemExit(1)
