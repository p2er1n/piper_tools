#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import time

from piper_sdk import C_PiperInterface_V2


def _get_attr(obj, name, default=None):
    return getattr(obj, name, default)


def _unwrap(obj, candidates):
    for name in candidates:
        inner = _get_attr(obj, name)
        if inner is not None and not isinstance(inner, (int, float, str, bool)):
            return inner
    return obj


def parse_args():
    parser = argparse.ArgumentParser(
        description="Move Piper arm by delta end pose relative to current pose."
    )

    parser.add_argument(
        "delta",
        nargs=7,
        type=float,
        metavar="dV",
        help="Delta pose in move_once units: dX dY dZ dRX dRY dRZ dGRIP",
    )

    parser.add_argument("--can", default="can0", help="CAN interface (default: can0)")

    parser.add_argument("--factor-xyz", type=float, default=1000000.0)
    parser.add_argument("--factor-rpy", type=float, default=57295.7795)
    parser.add_argument("--factor-grip", type=float, default=1000000.0)

    parser.add_argument("--motion-speed", type=int, default=100)
    parser.add_argument("--grip-speed", type=int, default=1000)

    parser.add_argument("--motion-mode", type=lambda x: int(x, 0), default=0x01)
    parser.add_argument("--motion-submode", type=lambda x: int(x, 0), default=0x00)
    parser.add_argument("--grip-mode", type=lambda x: int(x, 0), default=0x01)

    parser.add_argument(
        "--enable-timeout",
        type=float,
        default=10.0,
        help="Max seconds to wait for EnablePiper() (default: 10s)",
    )
    parser.add_argument(
        "--state-timeout",
        type=float,
        default=2.0,
        help="Max seconds to wait for valid current arm state (default: 2s)",
    )
    parser.add_argument(
        "--wait",
        type=float,
        default=0.2,
        help="Seconds to wait after sending command (default: 0.2s)",
    )
    return parser.parse_args()


def get_current_state_raw(piper, timeout_s: float):
    t0 = time.time()
    while True:
        end_pose_msg = piper.GetArmEndPoseMsgs()
        joint_msg = piper.GetArmJointMsgs()

        end_pose = _unwrap(end_pose_msg, ("end_pose", "pose"))
        joints = _unwrap(joint_msg, ("joint_state", "joints", "joint_msgs"))

        x = _get_attr(end_pose, "X_axis")
        y = _get_attr(end_pose, "Y_axis")
        z = _get_attr(end_pose, "Z_axis")
        rx = _get_attr(end_pose, "RX_axis")
        ry = _get_attr(end_pose, "RY_axis")
        rz = _get_attr(end_pose, "RZ_axis")
        grip = _get_attr(joints, "joint_6")

        if None not in (x, y, z, rx, ry, rz, grip):
            return int(x), int(y), int(z), int(rx), int(ry), int(rz), int(grip)

        if time.time() - t0 > timeout_s:
            raise TimeoutError(
                "Failed to read valid current pose/joint_6 from feedback in time."
            )
        time.sleep(0.01)


def main():
    args = parse_args()

    dX, dY, dZ, dRX, dRY, dRZ, dGRIP = args.delta

    dX_raw = int(round(dX * args.factor_xyz))
    dY_raw = int(round(dY * args.factor_xyz))
    dZ_raw = int(round(dZ * args.factor_xyz))
    dRX_raw = int(round(dRX * args.factor_rpy))
    dRY_raw = int(round(dRY * args.factor_rpy))
    dRZ_raw = int(round(dRZ * args.factor_rpy))
    dGRIP_raw = int(round(dGRIP * args.factor_grip))

    print("Connecting to Piper...")
    piper = C_PiperInterface_V2(args.can)
    piper.ConnectPort()

    print("Enabling arm...")
    t0 = time.time()
    while not piper.EnablePiper():
        if time.time() - t0 > args.enable_timeout:
            print(f"Enable timeout after {args.enable_timeout}s.")
            sys.exit(1)
        time.sleep(0.01)

    cur = get_current_state_raw(piper, args.state_timeout)
    curX, curY, curZ, curRX, curRY, curRZ, curGRIP = cur

    tgtX = curX + dX_raw
    tgtY = curY + dY_raw
    tgtZ = curZ + dZ_raw
    tgtRX = curRX + dRX_raw
    tgtRY = curRY + dRY_raw
    tgtRZ = curRZ + dRZ_raw
    tgtGRIP = curGRIP + dGRIP_raw

    print("Current(raw):", cur)
    print("Delta(raw):  ", (dX_raw, dY_raw, dZ_raw, dRX_raw, dRY_raw, dRZ_raw, dGRIP_raw))
    print("Target(raw): ", (tgtX, tgtY, tgtZ, tgtRX, tgtRY, tgtRZ, tgtGRIP))

    piper.MotionCtrl_2(args.motion_mode, args.motion_submode, args.motion_speed, 0x00)
    piper.EndPoseCtrl(tgtX, tgtY, tgtZ, tgtRX, tgtRY, tgtRZ)
    piper.GripperCtrl(abs(tgtGRIP), args.grip_speed, args.grip_mode, 0)

    time.sleep(args.wait)

    try:
        print("Current arm state:", piper.GetArmEndPoseMsgs())
    except Exception:
        pass

    print("Done.")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
