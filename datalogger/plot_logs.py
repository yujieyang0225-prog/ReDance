"""Plot the 5 IMUs x 6 axes for every log CSV in logs/, plus per-category
overlay comparisons across movements (hipflexion, trunkflexion, trunkrotation,
sitting vs standing).

Usage:
    python3 plot_logs.py

Outputs (under logs/plots/):
    per_file/<name>_accel.png   per_file/<name>_gyro.png   -- one file's own 5 IMUs
    comparison_accel.png        comparison_gyro.png        -- all files overlaid, colored by movement category
"""
import os
import re
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR = os.path.join(HERE, "logs")
PLOTS_DIR = os.path.join(LOGS_DIR, "plots")
PER_FILE_DIR = os.path.join(PLOTS_DIR, "per_file")

IMUS = [(0, "lumbar"), (1, "clavicleR"), (2, "clavicleL"),
        (3, "thighL"), (4, "thighR")]

ACCEL_AXES = [("ax", "Accel X (g)"), ("ay", "Accel Y (g)"), ("az", "Accel Z (g)")]
GYRO_AXES = [("gx", "Gyro X (deg/s)"), ("gy", "Gyro Y (deg/s)"), ("gz", "Gyro Z (deg/s)")]


def load_csv(path):
    with open(path, newline="") as f:
        rows = [r for r in csv.reader(f) if r and not r[0].startswith("#")]
    header, data_rows = rows[0], rows[1:]
    idx = {name: i for i, name in enumerate(header)}

    t_host = np.array([float(r[idx["t_host_s"]]) for r in data_rows])
    t = t_host - t_host[0]

    cols = {}
    for imu_id, _ in IMUS:
        for axis, _ in ACCEL_AXES + GYRO_AXES:
            key = f"{axis}{imu_id}"
            cols[key] = np.array([float(r[idx[key]]) for r in data_rows])
    return t, cols


def category_from_filename(filename):
    stem = os.path.splitext(filename)[0]
    return re.sub(r"_\d+$", "", stem)


def plot_group(t, cols, axes_spec, title, out_path, per_imu_series=None):
    """axes_spec: list of (axis_prefix, ylabel). per_imu_series: optional
    dict {(imu_id, axis): [(t_arr, y_arr, color, label), ...]} to overlay
    multiple series instead of the single (t, cols) pair."""
    fig, axes = plt.subplots(len(axes_spec), len(IMUS),
                              figsize=(4 * len(IMUS), 3 * len(axes_spec)),
                              sharex=False)
    if len(axes_spec) == 1:
        axes = axes.reshape(1, -1)

    for row, (axis, ylabel) in enumerate(axes_spec):
        for col, (imu_id, imu_name) in enumerate(IMUS):
            ax = axes[row, col]
            if per_imu_series is not None:
                for (st, sy, color, label) in per_imu_series[(imu_id, axis)]:
                    ax.plot(st, sy, linewidth=0.8, color=color, alpha=0.75, label=label)
            else:
                ax.plot(t, cols[f"{axis}{imu_id}"], linewidth=0.9, color="#2b6ea5")
            if row == 0:
                ax.set_title(f"IMU{imu_id} {imu_name}", fontsize=10)
            if col == 0:
                ax.set_ylabel(ylabel, fontsize=9)
            if row == len(axes_spec) - 1:
                ax.set_xlabel("time (s)", fontsize=9)
            ax.tick_params(labelsize=7)

    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    if per_imu_series is not None:
        handles, labels = axes[0, 0].get_legend_handles_labels()
        seen = set()
        uniq_h, uniq_l = [], []
        for h, l in zip(handles, labels):
            if l not in seen:
                seen.add(l)
                uniq_h.append(h)
                uniq_l.append(l)
        fig.legend(uniq_h, uniq_l, loc="lower center", ncol=len(uniq_l),
                   bbox_to_anchor=(0.5, -0.01), fontsize=9)
        fig.tight_layout(rect=(0, 0.06, 1, 0.96))

    fig.savefig(out_path, dpi=130, bbox_inches="tight")
    plt.close(fig)


def main():
    os.makedirs(PER_FILE_DIR, exist_ok=True)
    files = sorted(f for f in os.listdir(LOGS_DIR) if f.endswith(".csv"))

    loaded = {}
    for fname in files:
        loaded[fname] = load_csv(os.path.join(LOGS_DIR, fname))

    # --- per-file detail plots -------------------------------------------
    for fname, (t, cols) in loaded.items():
        stem = os.path.splitext(fname)[0]
        plot_group(t, cols, ACCEL_AXES, f"{fname} - Accelerometers",
                   os.path.join(PER_FILE_DIR, f"{stem}_accel.png"))
        plot_group(t, cols, GYRO_AXES, f"{fname} - Gyroscopes",
                   os.path.join(PER_FILE_DIR, f"{stem}_gyro.png"))

    # --- category overlay comparison --------------------------------------
    categories = sorted({category_from_filename(f) for f in files})
    cmap = matplotlib.colormaps.get_cmap("tab10")
    color_of = {cat: cmap(i) for i, cat in enumerate(categories)}

    def build_series(axes_spec):
        series = {(imu_id, axis): [] for imu_id, _ in IMUS for axis, _ in axes_spec}
        for fname, (t, cols) in loaded.items():
            cat = category_from_filename(fname)
            color = color_of[cat]
            for imu_id, _ in IMUS:
                for axis, _ in axes_spec:
                    series[(imu_id, axis)].append((t, cols[f"{axis}{imu_id}"], color, cat))
        return series

    plot_group(None, None, ACCEL_AXES, "All movements overlaid - Accelerometers",
               os.path.join(PLOTS_DIR, "comparison_accel.png"),
               per_imu_series=build_series(ACCEL_AXES))
    plot_group(None, None, GYRO_AXES, "All movements overlaid - Gyroscopes",
               os.path.join(PLOTS_DIR, "comparison_gyro.png"),
               per_imu_series=build_series(GYRO_AXES))

    print(f"Wrote {2 * len(files)} per-file plots to {PER_FILE_DIR}")
    print(f"Wrote 2 comparison plots to {PLOTS_DIR}")
    print("Categories:", ", ".join(categories))


if __name__ == "__main__":
    main()
