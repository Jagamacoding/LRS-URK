#!/usr/bin/env python3
# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT
"""Plot recorded MAVROS commands/poses and report measured tracking errors.

Run after sourcing ROS: python3 scripts/analyze_bag.py BAG --output results/run
Metrics compare each received pose with the most recent recorded command; they
include sensor/transport delay and are not ground-truth collision measurements.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"samples": 0}
    return {"samples": len(values), "rms": float(np.sqrt(np.mean(values ** 2))),
            "max_abs": float(np.max(np.abs(values))),
            "p95_abs": float(np.percentile(np.abs(values), 95))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--output", required=True, help="Output file prefix")
    parser.add_argument("--poi-east", type=float, default=0.0)
    parser.add_argument("--poi-north", type=float, default=0.0)
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    names = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {"/mavros/local_position/pose", "/mavros/setpoint_raw/local",
              "/mission_control/state", "/mavros/global_position/compass_hdg",
              "/mavros/global_position/global", "/mavros/state"}
    types = {name: get_message(typ) for name, typ in names.items() if name in wanted}
    target = None
    compass = math.nan
    stage = "WAIT"
    origin = None
    states = []
    rows = []
    start = None
    armed = False
    final_mode = ""
    fix = None
    command_times = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(data, types[topic])
        if start is None:
            start = stamp
        t = (stamp - start) / 1e9
        if topic == "/mission_control/state":
            stage = msg.data
            states.append({"seconds": t, "stage": stage})
        elif topic == "/mavros/setpoint_raw/local":
            target = msg
            command_times.append(t)
        elif topic == "/mavros/global_position/compass_hdg":
            compass = msg.data
        elif topic == "/mavros/global_position/global":
            fix = msg
        elif topic == "/mavros/state":
            armed, final_mode = msg.armed, msg.mode
        elif topic == "/mavros/local_position/pose":
            p, q = msg.pose.position, msg.pose.orientation
            if origin is None:
                origin = (p.x, p.y, p.z)
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            xyz = [p.x-origin[0], p.y-origin[1], p.z-origin[2]]
            if target is None:
                command = [math.nan]*3
                target_yaw = math.nan
            else:
                command = [target.position.x-origin[0], target.position.y-origin[1],
                           target.position.z-origin[2]]
                target_yaw = target.yaw
            bearing = math.atan2(args.poi_north-xyz[1], args.poi_east-xyz[0])
            rows.append([t, stage, *xyz, *command, math.degrees(yaw),
                         math.degrees(target_yaw), compass,
                         math.degrees(angle(yaw-target_yaw)),
                         math.degrees(angle(yaw-math.radians(90-compass))),
                         math.degrees(angle(yaw-bearing)),
                         fix.latitude if fix else math.nan,
                         fix.longitude if fix else math.nan,
                         fix.altitude if fix else math.nan])
    if not rows:
        raise SystemExit("No pose messages in bag")
    header = ["seconds", "stage", "x_m", "y_m", "z_m", "cmd_x_m", "cmd_y_m",
              "cmd_z_m", "yaw_enu_deg", "cmd_yaw_enu_deg", "compass_deg",
              "yaw_tracking_error_deg", "compass_pose_error_deg", "poi_bearing_error_deg",
              "latitude_deg", "longitude_deg", "gps_altitude_m"]
    with output.with_suffix(".csv").open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(rows)
    values = np.asarray([[r[0], *r[2:]] for r in rows], dtype=float)
    phases = np.asarray([r[1] for r in rows])
    distance = np.linalg.norm(values[:, 1:4]-values[:, 4:7], axis=1)
    summary = {"bag": args.bag, "duration_seconds": float(values[-1, 0]),
               "complete": any(s["stage"] == "DONE" for s in states),
               "final_armed": armed, "final_mode": final_mode, "state_sequence": states,
               "compass_pose_error_deg": stats(values[:, 11]),
               "max_height_above_first_pose_m": float(np.max(values[:, 3])),
               "max_radius_from_first_pose_m": float(np.max(np.hypot(values[:, 1], values[:, 2]))),
               "per_stage": {}}
    if len(command_times) > 1:
        intervals = np.diff(command_times)
        summary["command_timing"] = {"count": len(command_times),
            "median_hz": float(1/np.median(intervals)),
            "largest_gap_seconds": float(np.max(intervals))}
    for phase in sorted(set(phases)):
        selected = phases == phase
        summary["per_stage"][phase] = {"position_error_m": stats(distance[selected]),
            "yaw_tracking_error_deg": stats(values[selected, 10])}
    figure = phases == "FIGURE"
    if np.any(figure):
        summary["figure_extent_m"] = {
            "flown_x": float(np.ptp(values[figure, 1])),
            "flown_y": float(np.ptp(values[figure, 2])),
            "commanded_x": float(np.ptp(values[figure, 4])),
            "commanded_y": float(np.ptp(values[figure, 5]))}
    square = ((phases == "NAVIGATE") &
              (np.hypot(values[:, 1]-args.poi_east, values[:, 2]-args.poi_north) > 1.0))
    summary["navigation_poi_error_outside_1m_deg"] = stats(values[square, 12])
    output.with_suffix(".json").write_text(json.dumps(summary, indent=2)+"\n")
    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    axes[0].plot(values[:, 4], values[:, 5], "--", color="tab:orange", label="Commanded")
    axes[0].plot(values[:, 1], values[:, 2], color="tab:blue", label="Flown")
    axes[0].scatter([0], [0], marker="s", color="black", label="Launch")
    axes[0].set(xlabel="East from first pose [m]", ylabel="North from first pose [m]")
    axes[0].axis("equal")
    axes[0].legend()
    axes[1].plot(values[:, 0], values[:, 6], "--", label="Commanded")
    axes[1].plot(values[:, 0], values[:, 3], label="Flown")
    axes[1].set(xlabel="Recorded time [s]", ylabel="Height from first pose [m]")
    axes[1].legend()
    axes[2].plot(values[:, 0], values[:, 10], label="Pose minus command")
    axes[2].plot(values[:, 0], values[:, 11], label="Pose minus compass ENU", alpha=.6)
    axes[2].set(xlabel="Recorded time [s]", ylabel="Wrapped yaw error [deg]")
    axes[2].legend()
    for axis in axes:
        axis.grid(alpha=.3)
    fig.suptitle(Path(args.bag).name)
    fig.tight_layout()
    fig.savefig(output.with_suffix(".png"), dpi=160)
    plt.close(fig)
    print(json.dumps({k: v for k, v in summary.items() if k != "state_sequence"}, indent=2))


if __name__ == "__main__":
    main()
