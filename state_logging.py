#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import sys
import time
from pathlib import Path

from piper_sdk import C_PiperInterface_V2


REFRESH_INTERVAL_S = 0.05
RANGE_FILE = Path(__file__).with_name("state_logging_ranges.json")

FIELD_SPECS = (
    ("eef_x", "EEF X", "0.001 mm"),
    ("eef_y", "EEF Y", "0.001 mm"),
    ("eef_z", "EEF Z", "0.001 mm"),
    ("eef_rx", "EEF RX", "0.001 deg"),
    ("eef_ry", "EEF RY", "0.001 deg"),
    ("eef_rz", "EEF RZ", "0.001 deg"),
    ("joint_1", "Joint 1", "0.001 deg"),
    ("joint_2", "Joint 2", "0.001 deg"),
    ("joint_3", "Joint 3", "0.001 deg"),
    ("joint_4", "Joint 4", "0.001 deg"),
    ("joint_5", "Joint 5", "0.001 deg"),
    ("joint_6", "Joint 6", "0.001 deg"),
    ("gripper_distance", "Gripper Distance", "0.001 mm"),
    ("gripper_effort", "Gripper Effort", "0.001 N·m"),
)

RESET = "\x1b[0m"
BOLD = "\x1b[1m"
DIM = "\x1b[2m"
CYAN = "\x1b[36m"
GREEN = "\x1b[32m"
YELLOW = "\x1b[33m"
MAGENTA = "\x1b[35m"
WHITE = "\x1b[37m"

CTRL_MODE_LABELS = {
    0x00: "待机模式",
    0x01: "CAN指令控制模式",
    0x02: "示教模式",
}

ARM_STATUS_LABELS = {
    0x00: "正常",
    0x01: "急停",
    0x02: "无解",
    0x03: "奇异点",
    0x04: "目标角度超过限",
    0x05: "关节通信异常",
    0x06: "关节抱闸未打开",
    0x07: "机械臂发生碰撞",
    0x08: "拖动示教时超速",
    0x09: "关节状态异常",
    0x0A: "其它异常",
    0x0B: "示教记录",
    0x0C: "示教执行",
    0x0D: "示教暂停",
    0x0E: "主控NTC过温",
    0x0F: "释放电阻NTC过温",
}

MODE_FEED_LABELS = {
    0x00: "MOVE P",
    0x01: "MOVE J",
    0x02: "MOVE L",
    0x03: "MOVE C",
    0x04: "MOVE M",
    0x05: "MOVE_CPV",
}

MOTION_STATUS_LABELS = {
    0x00: "Arrived",
    0x01: "Moving",
}


def _get_value(obj, name, default=None):
    return getattr(obj, name, default)


def _unwrap_message(obj, candidates):
    for name in candidates:
        inner = _get_value(obj, name)
        if inner is not None and not isinstance(inner, (int, float, str, bool)):
            return inner
    return obj


def _format_value(value, unit):
    if value is None:
        return "N/A"
    return f"{value} {unit}"


def _format_raw(value):
    if value is None:
        return "N/A"
    return str(value)


def _format_hz(value):
    if value is None:
        return "N/A"
    try:
        return f"{float(value):.2f} Hz"
    except (TypeError, ValueError):
        return _format_raw(value)


def _colorize(text, color):
    return f"{color}{text}{RESET}"


def _pad_value(value, unit, width=16):
    rendered = _format_value(value, unit)
    if rendered == "N/A":
        return f"{rendered:>{width}}"
    return f"{rendered:>{width}}"


def _format_code(code, labels):
    if code is None:
        return "N/A"
    label = labels.get(code, "Unknown")
    return f"0x{int(code):02X} {label}"


def _status_dot(is_ok):
    if is_ok is None:
        return _colorize("?", YELLOW)
    return _colorize("●", GREEN if is_ok else "\x1b[31m")


def _status_grid(title, names, getter):
    cells = []
    for index, name in enumerate(names, start=1):
        value = getter(name)
        cells.append(f"J{index}:{_status_dot(value is False if value is not None else None)}")
    return f"{_colorize(title, WHITE):<19} " + "  ".join(cells)


def _format_range_line(stats, key, unit):
    entry = stats.get(key, {})
    min_text = _pad_value(entry.get("min"), unit)
    max_text = _pad_value(entry.get("max"), unit)
    return f"{_colorize('min', GREEN)} {min_text}   {_colorize('max', YELLOW)} {max_text}"


def _collect_values(end_pose, joint_state, gripper_state):
    end_pose_data = _unwrap_message(end_pose, ("end_pose", "pose"))
    joint_state_data = _unwrap_message(joint_state, ("joint_state", "joints", "joint_msgs"))
    gripper_data = _unwrap_message(gripper_state, ("gripper_state", "gripper", "grippers"))

    return {
        "eef_x": _get_value(end_pose_data, "X_axis"),
        "eef_y": _get_value(end_pose_data, "Y_axis"),
        "eef_z": _get_value(end_pose_data, "Z_axis"),
        "eef_rx": _get_value(end_pose_data, "RX_axis"),
        "eef_ry": _get_value(end_pose_data, "RY_axis"),
        "eef_rz": _get_value(end_pose_data, "RZ_axis"),
        "joint_1": _get_value(joint_state_data, "joint_1"),
        "joint_2": _get_value(joint_state_data, "joint_2"),
        "joint_3": _get_value(joint_state_data, "joint_3"),
        "joint_4": _get_value(joint_state_data, "joint_4"),
        "joint_5": _get_value(joint_state_data, "joint_5"),
        "joint_6": _get_value(joint_state_data, "joint_6"),
        "gripper_distance": _get_value(gripper_data, "grippers_angle"),
        "gripper_effort": _get_value(gripper_data, "grippers_effort"),
    }


def _load_ranges():
    if not RANGE_FILE.exists():
        return {key: {"min": None, "max": None} for key, _, _ in FIELD_SPECS}

    try:
        data = json.loads(RANGE_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return {key: {"min": None, "max": None} for key, _, _ in FIELD_SPECS}

    stats = {}
    for key, _, _ in FIELD_SPECS:
        entry = data.get(key, {})
        stats[key] = {
            "min": entry.get("min"),
            "max": entry.get("max"),
        }
    return stats


def _save_ranges(stats):
    serializable = {}
    for key, _, _ in FIELD_SPECS:
        entry = stats.get(key, {})
        serializable[key] = {
            "min": entry.get("min"),
            "max": entry.get("max"),
        }
    RANGE_FILE.write_text(
        json.dumps(serializable, ensure_ascii=True, indent=2),
        encoding="utf-8",
    )


def _update_ranges(stats, values):
    changed = False
    for key, _, _ in FIELD_SPECS:
        value = values.get(key)
        if value is None:
            continue

        entry = stats.setdefault(key, {"min": None, "max": None})
        if entry["min"] is None or value < entry["min"]:
            entry["min"] = value
            changed = True
        if entry["max"] is None or value > entry["max"]:
            entry["max"] = value
            changed = True
    return changed


def _section_header(title, color):
    line = "=" * 74
    return [
        _colorize(line, DIM),
        _colorize(f"{BOLD}{title}{RESET}", color),
        _colorize(line, DIM),
    ]


def _metric_line(label, value, unit, stats, key):
    current = _pad_value(value, unit)
    left = _colorize(f"{label:<10}", WHITE)
    current_text = f"{_colorize('now', CYAN)} {current}"
    return f"{left} {current_text}   {_format_range_line(stats, key, unit)}"


def _info_line(label, value, color=CYAN):
    return f"{_colorize(f'{label:<19}', WHITE)} {_colorize(value, color)}"


def _render_status(end_pose, joint_state, gripper_state, arm_status_msg, stats, can_name):
    end_pose_data = _unwrap_message(end_pose, ("end_pose", "pose"))
    joint_state_data = _unwrap_message(joint_state, ("joint_state", "joints", "joint_msgs"))
    gripper_data = _unwrap_message(gripper_state, ("gripper_state", "gripper", "grippers"))
    arm_status_data = _unwrap_message(arm_status_msg, ("arm_status", "status"))
    err_status_data = _unwrap_message(_get_value(arm_status_data, "err_status"), ("err_status", "status"))

    lines = [
        _colorize(f"{BOLD}Piper State Logger{RESET}", MAGENTA),
        f"{_colorize('CAN', WHITE)} {_colorize(can_name, CYAN)}   "
        f"{_colorize('Range File', WHITE)} {_colorize(RANGE_FILE.name, CYAN)}   "
        f"{_colorize('Exit', WHITE)} {_colorize('Ctrl+C', CYAN)}",
        "",
    ]

    lines.extend(_section_header("EEF Pose [raw units: 0.001 mm / 0.001 deg]", MAGENTA))
    lines.extend(
        [
            _metric_line("X", _get_value(end_pose_data, "X_axis"), "0.001 mm", stats, "eef_x"),
            _metric_line("Y", _get_value(end_pose_data, "Y_axis"), "0.001 mm", stats, "eef_y"),
            _metric_line("Z", _get_value(end_pose_data, "Z_axis"), "0.001 mm", stats, "eef_z"),
            _metric_line("RX", _get_value(end_pose_data, "RX_axis"), "0.001 deg", stats, "eef_rx"),
            _metric_line("RY", _get_value(end_pose_data, "RY_axis"), "0.001 deg", stats, "eef_ry"),
            _metric_line("RZ", _get_value(end_pose_data, "RZ_axis"), "0.001 deg", stats, "eef_rz"),
            f"{_colorize('Feedback', WHITE):<19} {_colorize(_format_hz(_get_value(end_pose, 'Hz')), CYAN)}",
            "",
        ]
    )

    lines.extend(_section_header("Joint State [raw units: 0.001 deg]", MAGENTA))
    lines.extend(
        [
            _metric_line("J1", _get_value(joint_state_data, "joint_1"), "0.001 deg", stats, "joint_1"),
            _metric_line("J2", _get_value(joint_state_data, "joint_2"), "0.001 deg", stats, "joint_2"),
            _metric_line("J3", _get_value(joint_state_data, "joint_3"), "0.001 deg", stats, "joint_3"),
            _metric_line("J4", _get_value(joint_state_data, "joint_4"), "0.001 deg", stats, "joint_4"),
            _metric_line("J5", _get_value(joint_state_data, "joint_5"), "0.001 deg", stats, "joint_5"),
            _metric_line("J6", _get_value(joint_state_data, "joint_6"), "0.001 deg", stats, "joint_6"),
            f"{_colorize('Feedback', WHITE):<19} {_colorize(_format_hz(_get_value(joint_state, 'Hz')), CYAN)}",
            "",
        ]
    )

    lines.extend(_section_header("Gripper [raw units]", MAGENTA))
    lines.extend(
        [
            _metric_line("Distance", _get_value(gripper_data, "grippers_angle"), "0.001 mm", stats, "gripper_distance"),
            _metric_line("Effort", _get_value(gripper_data, "grippers_effort"), "0.001 N·m", stats, "gripper_effort"),
            f"{_colorize('Feedback', WHITE):<19} {_colorize(_format_hz(_get_value(gripper_state, 'Hz')), CYAN)}",
            "",
        ]
    )

    lines.extend(_section_header("Arm Status [live only]", MAGENTA))
    lines.extend(
        [
            _info_line("Ctrl Mode", _format_code(_get_value(arm_status_data, "ctrl_mode"), CTRL_MODE_LABELS)),
            _info_line("Arm Status", _format_code(_get_value(arm_status_data, "arm_status"), ARM_STATUS_LABELS)),
            _info_line("Mode Feed", _format_code(_get_value(arm_status_data, "mode_feed"), MODE_FEED_LABELS)),
            _info_line("Teach Status", _format_raw(_get_value(arm_status_data, "teach_status"))),
            _info_line("Motion Status", _format_code(_get_value(arm_status_data, "motion_status"), MOTION_STATUS_LABELS)),
            _info_line("Trajectory No", _format_raw(_get_value(arm_status_data, "trajectory_num"))),
            _status_grid(
                "Angle Limit",
                (
                    "joint_1_angle_limit",
                    "joint_2_angle_limit",
                    "joint_3_angle_limit",
                    "joint_4_angle_limit",
                    "joint_5_angle_limit",
                    "joint_6_angle_limit",
                ),
                lambda name: _get_value(err_status_data, name),
            ),
            _status_grid(
                "Joint Comm",
                (
                    "communication_status_joint_1",
                    "communication_status_joint_2",
                    "communication_status_joint_3",
                    "communication_status_joint_4",
                    "communication_status_joint_5",
                    "communication_status_joint_6",
                ),
                lambda name: _get_value(err_status_data, name),
            ),
            _info_line("Feedback", _format_hz(_get_value(arm_status_msg, "Hz"))),
        ]
    )
    return "\n".join(lines)


def main():
    can_name = sys.argv[1] if len(sys.argv) > 1 else "can0"
    piper = C_PiperInterface_V2(can_name)
    piper.ConnectPort()
    stats = _load_ranges()

    while not piper.EnablePiper():
        time.sleep(0.01)

    sys.stdout.write("\x1b[2J\x1b[H")
    sys.stdout.flush()

    try:
        while True:
            end_pose = piper.GetArmEndPoseMsgs()
            joint_state = piper.GetArmJointMsgs()
            gripper_state = piper.GetArmGripperMsgs()
            arm_status_msg = piper.GetArmStatus()
            values = _collect_values(end_pose, joint_state, gripper_state)
            if _update_ranges(stats, values):
                _save_ranges(stats)

            status_text = _render_status(end_pose, joint_state, gripper_state, arm_status_msg, stats, can_name)
            sys.stdout.write("\x1b[H\x1b[2J")
            sys.stdout.write(status_text)
            sys.stdout.flush()
            time.sleep(REFRESH_INTERVAL_S)
    except KeyboardInterrupt:
        sys.stdout.write("\nStopped.\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
