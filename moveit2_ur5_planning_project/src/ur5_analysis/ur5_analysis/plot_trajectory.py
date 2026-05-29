import argparse
import csv
from pathlib import Path


def read_rows(results_dir: Path):
    rows = []
    summary_path = results_dir / "trajectory_summary.csv"
    csv_paths = [summary_path] if summary_path.exists() else sorted(results_dir.glob("*.csv"))
    for csv_path in csv_paths:
        with csv_path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                row.setdefault("source_file", csv_path.name)
                rows.append(row)
    return rows


def to_float(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def print_summary(rows):
    if not rows:
        print("No CSV result files found.")
        return

    total = len(rows)
    success = sum(1 for row in rows if row.get("success") == "1")
    avg_time = sum(to_float(row, "planning_time_ms") for row in rows) / total
    avg_points = sum(to_float(row, "trajectory_points") for row in rows) / total
    avg_length = sum(to_float(row, "joint_space_length") for row in rows) / total

    print("UR5 MoveIt planning result summary")
    print(f"  samples:          {total}")
    print(f"  success:          {success}/{total} = {success / total:.2%}")
    print(f"  avg plan time:    {avg_time:.2f} ms")
    print(f"  avg traj points:  {avg_points:.1f}")
    print(f"  avg joint length: {avg_length:.4f} rad")
    print()
    for row in rows:
        print(
            f"- {row['source_file']} | {row.get('planner_method', 'unknown')} | {row['scenario']} | "
            f"success={row['success']} | time={to_float(row, 'planning_time_ms'):.2f} ms | "
            f"points={row.get('trajectory_points', '0')} | "
            f"length={to_float(row, 'joint_space_length'):.4f}"
        )


def plot(rows, output_path: Path):
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - optional dependency
        print(f"matplotlib unavailable, skipped plot: {exc}")
        return

    labels = [
        f"{row['source_file']}\n{row.get('planner_method', 'unknown')}\n{row['scenario']}"
        for row in rows
    ]
    times = [to_float(row, "planning_time_ms") for row in rows]
    lengths = [to_float(row, "joint_space_length") for row in rows]

    fig, (ax_time, ax_len) = plt.subplots(2, 1, figsize=(12, 7), constrained_layout=True)
    ax_time.bar(range(len(rows)), times, color="#2f6f9f")
    ax_time.set_ylabel("Planning time (ms)")
    ax_time.set_title("UR5 MoveIt planning metrics")
    ax_time.grid(axis="y", alpha=0.3)

    ax_len.bar(range(len(rows)), lengths, color="#5c8a3a")
    ax_len.set_ylabel("Joint-space length (rad)")
    ax_len.set_xticks(range(len(rows)))
    ax_len.set_xticklabels(labels, rotation=35, ha="right")
    ax_len.grid(axis="y", alpha=0.3)

    fig.savefig(output_path, dpi=160)
    print(f"Saved plot: {output_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=Path, default=Path.home() / "moveit2_ur5_planning_project" / "results")
    parser.add_argument("--plot", action="store_true", help="Generate metrics PNG when matplotlib is available.")
    args = parser.parse_args()

    rows = read_rows(args.results)
    print_summary(rows)
    if args.plot and rows:
        plot(rows, args.results / "planning_metrics.png")


if __name__ == "__main__":
    main()
