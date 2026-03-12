#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import time
import sys
import re

import h5py
import numpy as np

from piper_sdk import C_PiperInterface_V2


# -----------------------------
# HDF5 expression parsing
# -----------------------------
_TERM_RE = re.compile(r"""
^\s*
(?P<path>/[^\[\]\+\s]+)     # /abc/def style path, no + or [] or spaces
\s*
(?:
  \[
    \s*(?P<sel>[^\]]+)\s*   # selector inside [...]
  \]
)?
\s*$
""", re.VERBOSE)

def _split_expr(expr: str) -> list[str]:
    """Split by '+' but ignore plus signs inside brackets."""
    parts = []
    buf = []
    depth = 0
    for ch in expr:
        if ch == '[':
            depth += 1
            buf.append(ch)
        elif ch == ']':
            depth = max(0, depth - 1)
            buf.append(ch)
        elif ch == '+' and depth == 0:
            part = ''.join(buf).strip()
            if part:
                parts.append(part)
            buf = []
        else:
            buf.append(ch)
    last = ''.join(buf).strip()
    if last:
        parts.append(last)
    return parts

def _parse_selector(sel: str):
    """
    Supported selectors (applied on axis=1 columns):
      - "3" or "-1" (single column)
      - ":6", "2:", "1:7", ":-1", etc (slice)
    """
    sel = sel.strip()
    if ':' in sel:
        # slice
        a, b = sel.split(':', 1)
        a = a.strip()
        b = b.strip()
        start = int(a) if a != '' else None
        stop = int(b) if b != '' else None
        return slice(start, stop)
    else:
        # single index
        return int(sel)

def _ensure_2d_len_first(arr: np.ndarray) -> np.ndarray:
    """Make sure array is (len, k)."""
    if arr.ndim == 1:
        return arr.reshape(-1, 1)
    if arr.ndim == 2:
        return arr
    raise ValueError(f"Dataset must be 1D or 2D for expression slicing/concat, got shape {arr.shape}")

def _select_cols(arr2d: np.ndarray, sel):
    """Select columns from (len,k) with sel as int or slice. Always returns 2D."""
    if isinstance(sel, slice):
        out = arr2d[:, sel]
        if out.ndim != 2:
            out = np.atleast_2d(out)
        return out
    else:
        # int index -> keep as (len,1)
        out = arr2d[:, sel]
        return out.reshape(-1, 1)

def eval_h5_expr(h5_path: str, expr: str) -> np.ndarray:
    """
    Evaluate expression like:
      /abc[:6] + /def[-1]
    Read datasets, slice columns (axis=1), then concat along last axis.
    Result must be (len, 7) for your use case (checked later).
    """
    terms = _split_expr(expr)
    if not terms:
        raise ValueError("Empty expression")

    arrays = []
    with h5py.File(h5_path, "r") as f:
        for term in terms:
            m = _TERM_RE.match(term)
            if not m:
                raise ValueError(
                    f"Bad term syntax: '{term}'. Expected like /path, /path[:6], /path[-1], /path[1:7]"
                )
            path = m.group("path")
            sel_str = m.group("sel")

            if path not in f:
                raise KeyError(f"Key not found in HDF5: {path}")

            arr = np.array(f[path])
            arr = arr.astype(np.float64, copy=False)

            if not np.isfinite(arr).all():
                raise ValueError(f"Dataset {path} contains NaN/Inf")

            arr2d = _ensure_2d_len_first(arr)

            if sel_str is not None and sel_str.strip() != "":
                sel = _parse_selector(sel_str)
                arr2d = _select_cols(arr2d, sel)

            arrays.append(arr2d)

    # check same len
    lens = [a.shape[0] for a in arrays]
    if len(set(lens)) != 1:
        raise ValueError(f"Length mismatch across terms: {[(t, a.shape) for t, a in zip(terms, arrays)]}")

    out = np.concatenate(arrays, axis=1)
    return out


# -----------------------------
# Piper replay
# -----------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description="Replay Piper arm end-pose trajectories from HDF5 using an expression (concat + slicing)."
    )
    p.add_argument("--h5", required=True, help="Path to .hdf5/.h5 file")
    p.add_argument(
        "--key",
        required=True,
        help=("Expression over HDF5 datasets, e.g. "
              "'/abc[:6]+/def[-1]'. Each term may be /path, /path[:6], /path[-1], /path[1:7]."),
    )
    p.add_argument("--can", default="can0", help="CAN interface name (default: can0)")
    p.add_argument("--hz", type=float, default=30.0, help="Replay frequency (default: 30Hz)")
    p.add_argument("--start", type=int, default=0, help="Start index (inclusive)")
    p.add_argument("--end", type=int, default=None, help="End index (exclusive). Default: until end")
    p.add_argument("--loop", action="store_true", help="Loop playback")

    # factors split
    p.add_argument("--factor-xyz", type=float, default=1000000.0, help="Scale factor for X,Y,Z (default: 1000000)")
    p.add_argument("--factor-rpy", type=float, default=57295.7795, help="Scale factor for RX,RY,RZ (default:  57295.7795)")
    p.add_argument("--factor-grip", type=float, default=1000000.0, help="Scale factor for gripper/joint_6 (default: 1000000)")
    p.add_argument("--factor-joint", type=float, default=57295.7795, help="Scale factor for joints 0-5 in JointCtrl mode (default: 57295.7795)")

    p.add_argument(
        "--enable-timeout",
        type=float,
        default=10.0,
        help="Max seconds to wait for EnablePiper() (default: 10s)",
    )
    p.add_argument("--motion-speed", type=int, default=100, help="MotionCtrl_2 speed parameter (default: 100)")
    p.add_argument("--grip-speed", type=int, default=1000, help="GripperCtrl speed parameter (default: 1000)")
    p.add_argument("--grip-mode", type=lambda x: int(x, 0), default=0x01, help="GripperCtrl mode (default: 0x01)")

    # 控制器模式选择
    ctrl_group = p.add_mutually_exclusive_group(required=False)
    ctrl_group.add_argument(
        "--end-posectrl",
        action="store_true",
        default=True,
        help="Use EndPoseCtrl (末端位置控制，默认)",
    )
    ctrl_group.add_argument(
        "--joint-ctrl",
        action="store_true",
        help="Use JointCtrl (关节位置控制)",
    )

    p.add_argument("--dry-run", action="store_true", help="Do not send commands; only print")
    p.add_argument("--print-every", type=int, default=50, help="Print status every N frames (default: 50)")
    return p.parse_args()


def connect_and_enable(can_iface: str, timeout_s: float, dry_run: bool):
    if dry_run:
        return None

    piper = C_PiperInterface_V2(can_iface)
    piper.ConnectPort()

    t0 = time.time()
    while not piper.EnablePiper():
        if time.time() - t0 > timeout_s:
            raise TimeoutError(f"EnablePiper() timeout after {timeout_s} seconds")
        time.sleep(0.01)
    return piper


def send_frame_endpose(
    piper,
    frame7: np.ndarray,
    factor_xyz: float,
    factor_rpy: float,
    factor_grip: float,
    motion_speed: int,
    grip_speed: int,
    grip_mode: int,
    dry_run: bool,
):
    # frame7: [X, Y, Z, RX, RY, RZ, joint_6]
    X  = int(round(frame7[0] * factor_xyz))
    Y  = int(round(frame7[1] * factor_xyz))
    Z  = int(round(frame7[2] * factor_xyz))
    RX = int(round(frame7[3] * factor_rpy))
    RY = int(round(frame7[4] * factor_rpy))
    RZ = int(round(frame7[5] * factor_rpy))
    joint_6 = int(round(frame7[6] * factor_grip))

    if dry_run:
        print(f"[DRY] EndPoseCtrl({X},{Y},{Z},{RX},{RY},{RZ})  GripperCtrl({abs(joint_6)},{grip_speed},{hex(grip_mode)},0)")
        return

    # 末端位置控制：ctrl_mode=0x01 (CAN指令控制), move_mode=0x00 (MOVE P - 位置模式)
    piper.MotionCtrl_2(0x01, 0x00, motion_speed, 0x00)
    piper.EndPoseCtrl(X, Y, Z, RX, RY, RZ)
    piper.GripperCtrl(abs(joint_6), grip_speed, grip_mode, 0)


def send_frame_joint(
    piper,
    frame7: np.ndarray,
    factor_joint: float,
    factor_grip: float,
    motion_speed: int,
    grip_speed: int,
    grip_mode: int,
    dry_run: bool,
):
    # frame7: [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    # 使用 factor_joint 作为前6个关节的scale因子 (等同于示例中的 factor = 57295.7795)
    joint_0 = int(round(frame7[0] * factor_joint))
    joint_1 = int(round(frame7[1] * factor_joint))
    joint_2 = int(round(frame7[2] * factor_joint))
    joint_3 = int(round(frame7[3] * factor_joint))
    joint_4 = int(round(frame7[4] * factor_joint))
    joint_5 = int(round(frame7[5] * factor_joint))
    joint_6 = int(round(frame7[6] * factor_grip))

    if dry_run:
        print(f"[DRY] JointCtrl({joint_0},{joint_1},{joint_2},{joint_3},{joint_4},{joint_5})  GripperCtrl({abs(joint_6)},{grip_speed},{hex(grip_mode)},0)")
        return

    # 关节控制模式: ctrl_mode=0x01, move_mode=0x01 (MOVE J - 关节运动)
    piper.MotionCtrl_2(0x01, 0x01, motion_speed, 0x00)
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
    piper.GripperCtrl(abs(joint_6), grip_speed, grip_mode, 0)


def main():
    args = parse_args()

    traj = eval_h5_expr(args.h5, args.key)

    if traj.ndim != 2 or traj.shape[1] != 7:
        raise ValueError(f"Expression result must have shape (len, 7), got {traj.shape}")

    n = traj.shape[0]
    start = max(0, args.start)
    end = n if args.end is None else min(n, args.end)
    if start >= end:
        raise ValueError(f"Invalid slice: start={start}, end={end}, len={n}")

    dt_target = 1.0 / args.hz if args.hz > 0 else 0.0

    piper = connect_and_enable(args.can, args.enable_timeout, args.dry_run)
    if piper is not None:
        piper.GripperCtrl(0, args.grip_speed, args.grip_mode, 0)

    print(f"Loaded expr: {args.key}")
    ctrl_mode = "JointCtrl" if args.joint_ctrl else "EndPoseCtrl"
    print(f"Result shape={traj.shape}, replay [{start}:{end}) at {args.hz} Hz, mode={ctrl_mode}, loop={args.loop}, dry_run={args.dry_run}")

    idx = start
    frame_count = 0

    try:
        while True:
            t0 = time.perf_counter()

            frame = traj[idx]
            if args.joint_ctrl:
                send_frame_joint(
                    piper=piper,
                    frame7=frame,
                    factor_joint=args.factor_joint,
                    factor_grip=args.factor_grip,
                    motion_speed=args.motion_speed,
                    grip_speed=args.grip_speed,
                    grip_mode=args.grip_mode,
                    dry_run=args.dry_run,
                )
            else:
                send_frame_endpose(
                    piper=piper,
                    frame7=frame,
                    factor_xyz=args.factor_xyz,
                    factor_rpy=args.factor_rpy,
                    factor_grip=args.factor_grip,
                    motion_speed=args.motion_speed,
                    grip_speed=args.grip_speed,
                    grip_mode=args.grip_mode,
                    dry_run=args.dry_run,
                )

            frame_count += 1
            if args.print_every > 0 and (frame_count % args.print_every == 0):
                if piper is not None:
                    try:
                        print(f"[{frame_count}] idx={idx} pose={frame.tolist()}  arm={piper.GetArmEndPoseMsgs()}")
                    except Exception:
                        print(f"[{frame_count}] idx={idx} pose={frame.tolist()}  (GetArmEndPoseMsgs() failed)")
                else:
                    print(f"[{frame_count}] idx={idx} pose={frame.tolist()}")

            idx += 1
            if idx >= end:
                if args.loop:
                    idx = start
                else:
                    break

            if dt_target > 0:
                elapsed = time.perf_counter() - t0
                to_sleep = dt_target - elapsed
                if to_sleep > 0:
                    time.sleep(to_sleep)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        pass

    print("Done.")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
