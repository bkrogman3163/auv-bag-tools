"""Read a ROS bag with Python and plot it — charts, time series, and a track map.

Works on rosbag1 (.bag) and rosbag2 (directory), with no ROS installation.
Topics are matched by message *type*, not name, so this runs unchanged against
different vehicles' bags.

Usage:
    python plot_bag.py <bag> --list
    python plot_bag.py <bag>                      # write all figures
    python plot_bag.py <bag> --out figs/
"""

from __future__ import annotations

import argparse
from collections.abc import Iterator
from functools import lru_cache
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")  # write files, no GUI needed
import matplotlib.pyplot as plt  # noqa: E402

from rosbags.highlevel import AnyReader  # noqa: E402

SCALARS = (int, float, bool, str, np.integer, np.floating, np.bool_)
MAX_DEPTH = 3


def field_names(msg: object) -> list[str]:
    """Field names of a rosbags message.

    rosbags builds messages as dataclasses, so they expose
    ``__dataclass_fields__`` and have no ``__slots__``. The synthetic
    ``__msgtype__`` entry is metadata, not data.
    """
    fields = getattr(msg, "__dataclass_fields__", None)
    if fields is not None:
        return [f for f in fields if not f.startswith("__")]
    return list(getattr(msg, "__slots__", []) or [])


def is_message(val: object) -> bool:
    return hasattr(val, "__dataclass_fields__") or hasattr(val, "__slots__")


def flatten(msg: object, prefix: str = "", depth: int = 0) -> Iterator[tuple[str, object]]:
    """Yield (dotted_name, scalar_value) for every scalar field in a message.

    Nested sub-messages are recursed into; large numeric arrays are summarised
    rather than exploded, so a 6000-point sonar scan doesn't become 6000 columns.
    """
    for field in field_names(msg):
        try:
            val = getattr(msg, field)
        except AttributeError:
            continue
        name = f"{prefix}{field}"
        if isinstance(val, SCALARS):
            yield name, val
        elif isinstance(val, np.ndarray):
            if val.size == 0 or val.dtype.kind not in "fiub":
                continue
            if val.size <= 4:                       # e.g. a 4-beam DVL range
                for i, v in enumerate(val.tolist()):
                    yield f"{name}[{i}]", v
            else:                                   # covariance, sonar bins, ...
                yield f"{name}.mean", float(np.nanmean(val))
                yield f"{name}.n", int(val.size)
        elif is_message(val) and depth < MAX_DEPTH:
            yield from flatten(val, f"{name}.", depth + 1)


@lru_cache(maxsize=32)
def to_frame(bag: Path, topic: str, limit: int | None = None) -> pd.DataFrame:
    """Deserialize one topic into a tidy DataFrame indexed by mission seconds.

    Cached: a 377 MB rosbag1 is slow to scan, and several panels want the same
    topic.
    """
    rows: list[dict] = []
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            return pd.DataFrame()
        t0 = reader.start_time
        for i, (conn, stamp, raw) in enumerate(reader.messages(connections=conns)):
            if limit and i >= limit:
                break
            msg = reader.deserialize(raw, conn.msgtype)
            rec = {"t": (stamp - t0) / 1e9}
            rec.update(dict(flatten(msg)))
            rows.append(rec)
    return pd.DataFrame(rows)


def find_topics(bag: Path) -> dict[str, list[tuple[str, str]]]:
    """Group topics into plot roles by message type."""
    roles: dict[str, list[tuple[str, str]]] = {
        "imu": [], "odom": [], "navsat": [], "pressure": [],
        "battery": [], "dvl": [], "depth": [], "other": [],
    }
    with AnyReader([bag]) as reader:
        for name, info in reader.topics.items():
            mt = info.msgtype.lower()
            entry = (name, info.msgtype)
            if "imu" in mt:
                roles["imu"].append(entry)
            elif "odometry" in mt:
                roles["odom"].append(entry)
            elif "navsatfix" in mt:
                roles["navsat"].append(entry)
            elif "fluidpressure" in mt or "pressure" in mt:
                roles["pressure"].append(entry)
            elif "batterystate" in mt:
                roles["battery"].append(entry)
            elif "dvl" in mt or "dvl" in name.lower():
                roles["dvl"].append(entry)
            elif "depth" in mt or "depth" in name.lower():
                roles["depth"].append(entry)
            else:
                roles["other"].append(entry)
    return roles


def pick_xy(df: pd.DataFrame) -> tuple[str, str] | None:
    """Find the best pair of columns to use as a horizontal track."""
    for x, y in (
        ("pose.pose.position.x", "pose.pose.position.y"),
        ("position.x", "position.y"),
        ("longitude", "latitude"),
        ("x", "y"),
    ):
        if x in df.columns and y in df.columns:
            return x, y
    return None


def first_col(df: pd.DataFrame, *candidates: str) -> str | None:
    for c in candidates:
        if c in df.columns and df[c].notna().any():
            return c
    return None


# ------------------------------------------------------------------ figures


def fig_overview(bag: Path, roles: dict, out: Path) -> None:
    """Depth profile, track map, speed, and topic rates in one sheet."""
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(f"Mission overview — {bag.name}", fontsize=15, fontweight="bold")

    # --- depth / pressure over time
    ax = axes[0][0]
    plotted = False
    for role, conv, label in (
        ("depth", lambda s: s, "depth (m)"),
        ("pressure", lambda s: (s - 101325.0) / 9804.5, "depth from pressure (m)"),
    ):
        for topic, _ in roles[role]:
            df = to_frame(bag, topic)
            col = first_col(df, "depth", "fluid_pressure", "pressure", "data", "z")
            if col is None or df.empty:
                continue
            ax.plot(df["t"], conv(df[col]), lw=1.0, label=f"{topic} · {label}")
            plotted = True
            break
        if plotted:
            break
    if not plotted:                       # fall back to odometry z
        for topic, _ in roles["odom"]:
            df = to_frame(bag, topic)
            col = first_col(df, "pose.pose.position.z")
            if col:
                ax.plot(df["t"], -df[col], lw=1.0, label=f"{topic} · depth (m)")
                plotted = True
                break
    ax.invert_yaxis()
    ax.set_xlabel("mission time (s)")
    ax.set_ylabel("depth (m)  — deeper is down")
    ax.set_title("Depth profile")
    ax.grid(alpha=0.3)
    if plotted:
        ax.legend(fontsize=8)

    # --- horizontal track, coloured by time
    ax = axes[0][1]
    for role in ("odom", "navsat"):
        for topic, _ in roles[role]:
            df = to_frame(bag, topic)
            xy = pick_xy(df)
            if not xy or df.empty:
                continue
            x, y = xy
            sc = ax.scatter(df[x], df[y], c=df["t"], cmap="viridis", s=4)
            ax.plot(df[x], df[y], lw=0.4, alpha=0.4, color="grey")
            plt.colorbar(sc, ax=ax, label="mission time (s)")
            geo = x == "longitude"
            ax.set_xlabel("longitude (deg)" if geo else "x / east (m)")
            ax.set_ylabel("latitude (deg)" if geo else "y / north (m)")
            ax.set_title(f"Track — {topic}")
            ax.set_aspect("equal", adjustable="datalim")
            break
        else:
            continue
        break
    ax.grid(alpha=0.3)

    # --- speed / velocity
    ax = axes[1][0]
    for role in ("dvl", "odom"):
        for topic, _ in roles[role]:
            df = to_frame(bag, topic)
            if df.empty:
                continue
            col = first_col(
                df, "speed_gnd", "velocity.x",
                "twist.twist.linear.x", "vx", "bi_x_axis",
                # Girona's LinkquestDvl exposes body/earth velocity vectors
                "velocityInst[0]", "velocityEarth[0]",
            )
            if col is None:
                continue
            ax.plot(df["t"], df[col], lw=0.8, label=f"{topic}.{col}")
            break
    ax.set_xlabel("mission time (s)")
    ax.set_ylabel("speed (m/s)")
    ax.set_title("Forward speed")
    ax.grid(alpha=0.3)
    if ax.get_legend_handles_labels()[0]:
        ax.legend(fontsize=8)

    # --- message rate per topic
    ax = axes[1][1]
    with AnyReader([bag]) as reader:
        dur = max((reader.end_time - reader.start_time) / 1e9, 1e-9)
        rates = sorted(
            ((n, i.msgcount / dur) for n, i in reader.topics.items()),
            key=lambda kv: kv[1],
        )
    names = [n if len(n) <= 26 else n[:23] + "..." for n, _ in rates]
    ax.barh(names, [r for _, r in rates], color="steelblue")
    ax.set_xscale("log")
    ax.set_xlabel("messages / second (log scale)")
    ax.set_title("Topic rates — the fast ones dominate storage")
    ax.grid(alpha=0.3, axis="x")
    ax.tick_params(labelsize=8)

    fig.tight_layout()
    dest = out / "01_overview.png"
    fig.savefig(dest, dpi=130)
    plt.close(fig)
    print(f"  wrote {dest}")


def fig_imu(bag: Path, roles: dict, out: Path) -> None:
    if not roles["imu"]:
        return
    # Where a vendor publishes both its own type and a standard sensor_msgs twin,
    # prefer the standard one — its field names are predictable.
    candidates = sorted(
        roles["imu"], key=lambda e: e[1] != "sensor_msgs/msg/Imu"
    )
    topic, msgtype = candidates[0]
    df = to_frame(bag, topic, limit=60000)
    if df.empty:
        return
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle(f"IMU — {topic}  ({msgtype})", fontsize=14, fontweight="bold")

    # Each group lists alternative field namings: the sensor_msgs convention
    # first, then the flat vendor style (gx/ax/roll) some custom types use.
    groups = (
        (
            "angular velocity (rad/s)",
            [("angular_velocity.x", "angular_velocity.y", "angular_velocity.z"),
             ("gx", "gy", "gz")],
        ),
        (
            "linear acceleration (m/s²)",
            [("linear_acceleration.x", "linear_acceleration.y",
              "linear_acceleration.z"),
             ("ax", "ay", "az")],
        ),
        (
            "orientation",
            [("orientation.x", "orientation.y", "orientation.z", "orientation.w"),
             ("roll", "pitch", "yaw")],
        ),
    )
    for ax, (label, alternatives) in zip(axes, groups):
        for cols in alternatives:
            if all(c in df.columns for c in cols):
                for c in cols:
                    ax.plot(df["t"], df[c], lw=0.5, label=c)
                ax.legend(fontsize=8, ncol=4, loc="upper right")
                break
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("mission time (s)")
    fig.tight_layout()
    dest = out / "02_imu.png"
    fig.savefig(dest, dpi=130)
    plt.close(fig)
    print(f"  wrote {dest}")


def fig_dvl(bag: Path, roles: dict, out: Path) -> None:
    if not roles["dvl"]:
        return
    topic = roles["dvl"][0][0]
    df = to_frame(bag, topic)
    if df.empty:
        return
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    fig.suptitle(f"DVL — {topic}", fontsize=14, fontweight="bold")

    alt = first_col(df, "altitude", "bd_range", "range[0]")
    if alt:
        axes[0].plot(df["t"], df[alt], lw=0.8, color="darkgreen")
        axes[0].set_ylabel(f"{alt} (m)")
        axes[0].set_title("Altitude above seafloor")
    axes[0].grid(alpha=0.3)

    # Per-beam data, whichever convention this vendor used.
    for stem, ylabel in (
        ("range[", "beam range (m)"),
        ("altitudeBeam[", "per-beam altitude (m)"),
        ("bottomVelocityBeam[", "per-beam bottom velocity (m/s)"),
        ("velocityInst[", "instrument-frame velocity (m/s)"),
        ("velocity.", "velocity (m/s)"),
    ):
        beams = [c for c in df.columns if c.startswith(stem)]
        if beams:
            for c in beams:
                axes[1].plot(df["t"], df[c], lw=0.6, label=c)
            axes[1].set_ylabel(ylabel)
            axes[1].set_title(f"Per-beam data — {stem.rstrip('[.')}")
            axes[1].legend(fontsize=8, ncol=4)
            break
    axes[1].set_xlabel("mission time (s)")
    axes[1].grid(alpha=0.3)

    fig.tight_layout()
    dest = out / "03_dvl.png"
    fig.savefig(dest, dpi=130)
    plt.close(fig)
    print(f"  wrote {dest}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--list", action="store_true", help="show discovered roles only")
    args = ap.parse_args()

    roles = find_topics(args.bag)
    print(f"Discovered topics in {args.bag.name}:")
    for role, entries in roles.items():
        for name, mt in entries:
            print(f"  {role:9s} {name:34s} {mt}")
    if args.list:
        return

    out = args.out or (args.bag.parent / f"{args.bag.stem}_figs")
    out.mkdir(parents=True, exist_ok=True)
    print(f"\nWriting figures to {out}")
    fig_overview(args.bag, roles, out)
    fig_imu(args.bag, roles, out)
    fig_dvl(args.bag, roles, out)
    print("done")


if __name__ == "__main__":
    main()
