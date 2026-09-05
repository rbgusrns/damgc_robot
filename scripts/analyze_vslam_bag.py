#!/usr/bin/env python3
"""Create a compact accuracy report from a VSLAM mapping rosbag."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import mean, pstdev
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ODOMETRY_TOPICS = (
    "/leader/odom/raw",
    "/leader/odometry/local",
    "/leader/odometry/global",
    "/visual_slam/tracking/odometry",
    "/visual_slam/vis/slam_odometry",
    "/visual_slam/slam_odometry_with_covariance",
)
STATUS_TOPIC = "/visual_slam/status"
CMD_TOPIC = "/leader/cmd_vel"
TF_TOPICS = ("/tf", "/tf_static")


def yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(end: float, start: float) -> float:
    return math.atan2(math.sin(end - start), math.cos(end - start))


def sample_summary(samples: list[tuple[float, float, float, float]]) -> dict[str, Any]:
    if not samples:
        return {}

    duration = max(0.0, samples[-1][0] - samples[0][0])
    path = sum(
        math.hypot(current[1] - previous[1], current[2] - previous[2])
        for previous, current in zip(samples, samples[1:])
    )
    accumulated_yaw = sum(
        angle_delta(current[3], previous[3])
        for previous, current in zip(samples, samples[1:])
    )
    start = samples[0]
    end = samples[-1]
    xs = [sample[1] for sample in samples]
    ys = [sample[2] for sample in samples]
    yaws = [sample[3] for sample in samples]

    def window_jitter(window: list[tuple[float, float, float, float]]) -> dict[str, float]:
        if not window:
            return {}
        wx = [sample[1] for sample in window]
        wy = [sample[2] for sample in window]
        base_yaw = window[0][3]
        wyaw = [angle_delta(sample[3], base_yaw) for sample in window]
        return {
            "position_range_m": math.hypot(max(wx) - min(wx), max(wy) - min(wy)),
            "yaw_range_deg": math.degrees(max(wyaw) - min(wyaw)),
        }

    head = [sample for sample in samples if sample[0] <= start[0] + 5.0]
    tail = [sample for sample in samples if sample[0] >= end[0] - 5.0]
    return {
        "samples": len(samples),
        "duration_s": duration,
        "average_rate_hz": (len(samples) - 1) / duration if duration > 0 else 0.0,
        "max_message_gap_ms": max(
            (current[0] - previous[0]) * 1000.0
            for previous, current in zip(samples, samples[1:])
        )
        if len(samples) > 1
        else 0.0,
        "start": {"x_m": start[1], "y_m": start[2], "yaw_deg": math.degrees(start[3])},
        "end": {"x_m": end[1], "y_m": end[2], "yaw_deg": math.degrees(end[3])},
        "net_displacement_m": math.hypot(end[1] - start[1], end[2] - start[2]),
        "net_yaw_deg": math.degrees(angle_delta(end[3], start[3])),
        "accumulated_yaw_deg": math.degrees(accumulated_yaw),
        "accumulated_path_m": path,
        "x_range_m": max(xs) - min(xs),
        "y_range_m": max(ys) - min(ys),
        "yaw_range_deg": math.degrees(max(angle_delta(yaw, yaws[0]) for yaw in yaws) - min(angle_delta(yaw, yaws[0]) for yaw in yaws)),
        "first_5s_jitter": window_jitter(head),
        "last_5s_jitter": window_jitter(tail),
    }


def finite_stats(values: list[float], scale: float = 1.0) -> dict[str, float]:
    values = [value * scale for value in values if math.isfinite(value)]
    if not values:
        return {}
    return {
        "mean": mean(values),
        "stddev": pstdev(values),
        "max": max(values),
        "min": min(values),
    }


def imu_window_summary(
    samples: list[tuple[float, float, float, float, float]], start: float, end: float
) -> dict[str, Any]:
    window = [sample for sample in samples if start <= sample[0] <= end]
    if not window:
        return {}
    return {
        "samples": len(window),
        "gyro_z_radps": finite_stats([sample[1] for sample in window]),
        "acceleration_norm_mps2": finite_stats(
            [math.sqrt(sample[2] ** 2 + sample[3] ** 2 + sample[4] ** 2) for sample in window]
        ),
    }


def analyze(bag_dir: Path, true_distance: float | None, true_yaw_deg: float | None) -> dict[str, Any]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    message_types = {topic: get_message(type_name) for topic, type_name in topic_types.items()}

    odometry: dict[str, list[tuple[float, float, float, float]]] = {
        topic: [] for topic in ODOMETRY_TOPICS if topic in topic_types
    }
    map_to_odom: list[tuple[float, float, float, float]] = []
    imu_samples: list[tuple[float, float, float, float, float]] = []
    status_states: list[int] = []
    callback_times: list[float] = []
    tracking_times: list[float] = []
    cmd_samples = 0
    cmd_active_samples = 0
    max_linear = 0.0
    max_angular = 0.0
    first_time: float | None = None
    last_time: float | None = None
    counts: dict[str, int] = {}

    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        timestamp = timestamp_ns / 1e9
        first_time = timestamp if first_time is None else min(first_time, timestamp)
        last_time = timestamp if last_time is None else max(last_time, timestamp)
        counts[topic] = counts.get(topic, 0) + 1
        if topic not in message_types:
            continue
        message = deserialize_message(raw, message_types[topic])

        if topic in odometry:
            pose = message.pose.pose
            odometry[topic].append(
                (timestamp, pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
            )
        elif topic == STATUS_TOPIC:
            status_states.append(int(message.vo_state))
            callback_times.append(float(message.node_callback_execution_time))
            tracking_times.append(float(message.track_execution_time))
        elif topic == CMD_TOPIC:
            cmd_samples += 1
            linear = math.hypot(float(message.linear.x), float(message.linear.y))
            angular = abs(float(message.angular.z))
            max_linear = max(max_linear, linear)
            max_angular = max(max_angular, angular)
            if linear > 1e-4 or angular > 1e-4:
                cmd_active_samples += 1
        elif topic == "/leader/imu/data_raw":
            imu_samples.append(
                (
                    timestamp,
                    float(message.angular_velocity.z),
                    float(message.linear_acceleration.x),
                    float(message.linear_acceleration.y),
                    float(message.linear_acceleration.z),
                )
            )
        elif topic in TF_TOPICS:
            for transform in message.transforms:
                parent = transform.header.frame_id.lstrip("/")
                child = transform.child_frame_id.lstrip("/")
                if parent == "map" and child == "odom":
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    map_to_odom.append(
                        (timestamp, translation.x, translation.y, yaw_from_quaternion(rotation))
                    )

    odometry_summary = {
        topic: sample_summary(samples) for topic, samples in odometry.items() if samples
    }
    status_counts = {
        "unknown": status_states.count(0),
        "success": status_states.count(1),
        "failed": status_states.count(2),
    }
    known_status = status_counts["success"] + status_counts["failed"]
    status_summary: dict[str, Any] = {
        "samples": len(status_states),
        **status_counts,
        "success_rate_percent": 100.0 * status_counts["success"] / known_status if known_status else None,
        "callback_execution_ms": finite_stats(callback_times, 1000.0),
        "tracking_execution_ms": finite_stats(tracking_times, 1000.0),
    }

    comparisons: dict[str, Any] = {}
    wheel = odometry_summary.get("/leader/odom/raw")
    vslam = odometry_summary.get("/visual_slam/tracking/odometry")
    if wheel and vslam:
        wheel_path = wheel["accumulated_path_m"]
        vslam_path = vslam["accumulated_path_m"]
        comparisons["wheel_vs_vslam"] = {
            "path_difference_m": wheel_path - vslam_path,
            "path_ratio_wheel_over_vslam": wheel_path / vslam_path if vslam_path > 1e-9 else None,
            "net_displacement_difference_m": wheel["net_displacement_m"] - vslam["net_displacement_m"],
            "accumulated_yaw_difference_deg": wheel["accumulated_yaw_deg"]
            - vslam["accumulated_yaw_deg"],
        }

    truth: dict[str, Any] = {}
    if true_distance is not None:
        truth["distance_m"] = true_distance
        truth["distance_errors"] = {
            topic: {
                "error_m": summary["accumulated_path_m"] - true_distance,
                "error_percent": 100.0 * (summary["accumulated_path_m"] - true_distance) / true_distance
                if true_distance > 0
                else None,
            }
            for topic, summary in odometry_summary.items()
        }
    if true_yaw_deg is not None:
        truth["yaw_deg"] = true_yaw_deg
        truth["yaw_errors"] = {
            topic: summary["accumulated_yaw_deg"] - true_yaw_deg
            for topic, summary in odometry_summary.items()
        }

    imu_summary: dict[str, Any] = {}
    if imu_samples:
        imu_duration = max(0.0, imu_samples[-1][0] - imu_samples[0][0])
        imu_summary = {
            "samples": len(imu_samples),
            "average_rate_hz": (len(imu_samples) - 1) / imu_duration if imu_duration > 0 else 0.0,
            "max_message_gap_ms": max(
                (current[0] - previous[0]) * 1000.0
                for previous, current in zip(imu_samples, imu_samples[1:])
            )
            if len(imu_samples) > 1
            else 0.0,
            "first_5s_stationary": imu_window_summary(
                imu_samples, imu_samples[0][0], imu_samples[0][0] + 5.0
            ),
            "last_5s_stationary": imu_window_summary(
                imu_samples, imu_samples[-1][0] - 5.0, imu_samples[-1][0]
            ),
        }

    return {
        "bag": str(bag_dir),
        "duration_s": (last_time - first_time) if first_time is not None and last_time is not None else 0.0,
        "topic_message_counts": counts,
        "odometry": odometry_summary,
        "map_to_odom": sample_summary(map_to_odom),
        "imu": imu_summary,
        "visual_slam_status": status_summary,
        "commands": {
            "samples": cmd_samples,
            "active_samples": cmd_active_samples,
            "active_sample_percent": 100.0 * cmd_active_samples / cmd_samples if cmd_samples else None,
            "max_linear_mps": max_linear,
            "max_angular_radps": max_angular,
        },
        "comparisons": comparisons,
        "ground_truth": truth,
    }


def value(value: Any, digits: int = 3) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def markdown_report(report: dict[str, Any]) -> str:
    lines = [
        "# VSLAM mapping analysis",
        "",
        f"- Bag: `{report['bag']}`",
        f"- Duration: {value(report['duration_s'], 1)} s",
        "",
        "## Odometry",
        "",
        "| Topic | Samples | Rate (Hz) | Max gap (ms) | Path (m) | Net displacement (m) | Accum. yaw (deg) | Last 5 s jitter (m / deg) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for topic, summary in report["odometry"].items():
        jitter = summary.get("last_5s_jitter", {})
        jitter_text = f"{value(jitter.get('position_range_m'))} / {value(jitter.get('yaw_range_deg'), 2)}"
        lines.append(
            f"| `{topic}` | {summary['samples']} | {value(summary['average_rate_hz'], 1)} | "
            f"{value(summary['max_message_gap_ms'], 1)} | "
            f"{value(summary['accumulated_path_m'])} | {value(summary['net_displacement_m'])} | "
            f"{value(summary['accumulated_yaw_deg'], 2)} | {jitter_text} |"
        )

    status = report["visual_slam_status"]
    imu = report["imu"]
    if imu:
        first_imu = imu["first_5s_stationary"]
        last_imu = imu["last_5s_stationary"]
        lines.extend(
            [
                "",
                "## IMU",
                "",
                f"- Rate / max gap: {value(imu['average_rate_hz'], 1)} Hz / "
                f"{value(imu['max_message_gap_ms'], 1)} ms",
                f"- First 5 s gyro Z mean / stddev: "
                f"{value(first_imu.get('gyro_z_radps', {}).get('mean'), 5)} / "
                f"{value(first_imu.get('gyro_z_radps', {}).get('stddev'), 5)} rad/s",
                f"- Last 5 s gyro Z mean / stddev: "
                f"{value(last_imu.get('gyro_z_radps', {}).get('mean'), 5)} / "
                f"{value(last_imu.get('gyro_z_radps', {}).get('stddev'), 5)} rad/s",
            ]
        )
    lines.extend(
        [
            "",
            "## Visual SLAM status",
            "",
            f"- Success / failed / unknown: {status['success']} / {status['failed']} / {status['unknown']}",
            f"- Tracking success rate: {value(status['success_rate_percent'], 1)}%",
            f"- Tracking execution mean / max: "
            f"{value(status['tracking_execution_ms'].get('mean'), 2)} / "
            f"{value(status['tracking_execution_ms'].get('max'), 2)} ms",
            "",
            "## Comparisons",
            "",
        ]
    )
    wheel_vslam = report["comparisons"].get("wheel_vs_vslam")
    if wheel_vslam:
        lines.extend(
            [
                f"- Wheel − VSLAM path: {value(wheel_vslam['path_difference_m'])} m",
                f"- Wheel / VSLAM path ratio: {value(wheel_vslam['path_ratio_wheel_over_vslam'])}",
                f"- Wheel − VSLAM accumulated yaw: "
                f"{value(wheel_vslam['accumulated_yaw_difference_deg'], 2)} deg",
            ]
        )
    else:
        lines.append("- Wheel/VSLAM comparison unavailable (one or both topics are missing).")

    truth = report["ground_truth"]
    if truth:
        lines.extend(["", "## Ground truth", ""])
        if "distance_m" in truth:
            lines.append(f"- Entered distance: {value(truth['distance_m'])} m")
            for topic, error in truth["distance_errors"].items():
                lines.append(
                    f"  - `{topic}`: {value(error['error_m'])} m ({value(error['error_percent'], 1)}%)"
                )
        if "yaw_deg" in truth:
            lines.append(f"- Entered yaw: {value(truth['yaw_deg'], 2)} deg")
            for topic, error in truth["yaw_errors"].items():
                lines.append(f"  - `{topic}`: {value(error, 2)} deg")
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--true-distance-m", type=float)
    parser.add_argument(
        "--true-yaw-deg",
        type=float,
        help="signed measured rotation: counter-clockwise positive, clockwise negative",
    )
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    bag_dir = args.bag_dir.resolve()
    if not bag_dir.is_dir():
        parser.error(f"bag directory does not exist: {bag_dir}")
    output_dir = (args.output_dir or bag_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    report = analyze(bag_dir, args.true_distance_m, args.true_yaw_deg)
    json_path = output_dir / "analysis.json"
    markdown_path = output_dir / "analysis.md"
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    markdown_path.write_text(markdown_report(report), encoding="utf-8")
    print(f"Analysis JSON: {json_path}")
    print(f"Analysis report: {markdown_path}")


if __name__ == "__main__":
    main()
