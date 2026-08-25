"""Profile any ROS bag — rosbag1 (.bag) or rosbag2 (directory) — with pure Python.

No ROS installation required. `rosbags`' AnyReader detects the format and, for
rosbag1, reads the message definitions embedded in the bag's own connection
headers, so vendor/custom message types work without their source packages.

Usage:
    python inspect_bag.py <path-to-bag>
    python inspect_bag.py <path-to-bag> --peek /topic_name
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd
from rosbags.highlevel import AnyReader


def profile(path: Path) -> pd.DataFrame:
    with AnyReader([path]) as reader:
        dur = (reader.end_time - reader.start_time) / 1e9
        print("=" * 78)
        print(f"BAG:      {path}")
        print(f"format:   {'rosbag1 (.bag)' if path.suffix == '.bag' else 'rosbag2'}")
        print(f"duration: {dur:,.1f} s  ({dur / 60:.1f} min)")
        print(f"messages: {reader.message_count:,}")
        print(f"topics:   {len(reader.topics)}")
        print("=" * 78)

        rows = []
        for name, info in reader.topics.items():
            count = info.msgcount
            rows.append(
                {
                    "topic": name,
                    "type": info.msgtype,
                    "count": count,
                    "hz": round(count / dur, 2) if dur > 0 else 0.0,
                }
            )

    df = pd.DataFrame(rows).sort_values("count", ascending=False)

    # Rate tiering is the thing that matters for storage planning: a handful of
    # fast topics usually dominate the byte budget.
    total = df["count"].sum()
    df["pct"] = (df["count"] / total * 100).round(1)

    with pd.option_context("display.max_colwidth", 46, "display.width", 200):
        print(df.to_string(index=False))

    unknown = df[df["type"].str.contains("cirs|custom", case=False, na=False)]
    if not unknown.empty:
        print(f"\nnote: {len(unknown)} topic(s) use package-specific message types")
    return df


def peek(path: Path, topic: str, n: int = 3) -> None:
    """Print the first n messages on a topic, field by field."""
    with AnyReader([path]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            print(f"topic {topic!r} not in bag. Available:")
            for t in sorted(reader.topics):
                print("   ", t)
            return
        print(f"\n--- first {n} messages on {topic} ({conns[0].msgtype}) ---")
        for i, (conn, stamp, raw) in enumerate(reader.messages(connections=conns)):
            if i >= n:
                break
            msg = reader.deserialize(raw, conn.msgtype)
            print(f"\n[{i}] t={stamp / 1e9:.3f}")
            # rosbags messages are dataclasses; __msgtype__ is metadata.
            fields = [
                f for f in getattr(msg, "__dataclass_fields__", {})
                if not f.startswith("__")
            ] or list(getattr(msg, "__slots__", []) or [])
            for field in fields:
                val = getattr(msg, field)
                text = repr(val)
                print(f"    {field:28s} {text[:88]}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--peek", metavar="TOPIC")
    ap.add_argument("-n", type=int, default=3)
    args = ap.parse_args()

    profile(args.bag)
    if args.peek:
        peek(args.bag, args.peek, args.n)


if __name__ == "__main__":
    main()
