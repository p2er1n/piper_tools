#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import time
import sys

from piper_sdk import C_PiperInterface_V2


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send one 7D end-pose command to Piper arm."
    )

    parser.add_argument(
        "pose",
        nargs=7,
        type=float,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ", "GRIP"),
        help="7 control parameters: X Y Z RX RY RZ GRIPPER",
    )

    parser.add_argument("--can", default="can0", help="CAN interface (default: can0)")

    # 三个 factor
    parser.add_argument("--factor-xyz", type=float, default=1000000.0)
    parser.add_argument("--factor-rpy", type=float, default=57295.7795)
    parser.add_argument("--factor-grip", type=float, default=1000000.0)

    parser.add_argument("--motion-speed", type=int, default=100)
    parser.add_argument("--grip-speed", type=int, default=1000)

    parser.add_argument("--motion-mode", type=lambda x: int(x, 0), default=0x01)
    parser.add_argument("--motion-submode", type=lambda x: int(x, 0), default=0x00)
    parser.add_argument("--grip-mode", type=lambda x: int(x, 0), default=0x01)

    parser.add_argument("--wait", type=float, default=0.2,
                        help="Seconds to wait after sending command (default 0.2s)")

    return parser.parse_args()


def main():
    args = parse_args()

    X, Y, Z, RX, RY, RZ, GRIP = args.pose

    # scale
    X  = int(round(X  * args.factor_xyz))
    Y  = int(round(Y  * args.factor_xyz))
    Z  = int(round(Z  * args.factor_xyz))

    RX = int(round(RX * args.factor_rpy))
    RY = int(round(RY * args.factor_rpy))
    RZ = int(round(RZ * args.factor_rpy))

    GRIP = int(round(GRIP * args.factor_grip))

    print("Connecting to Piper...")
    piper = C_PiperInterface_V2(args.can)
    piper.ConnectPort()

    print("Enabling arm...")
    t0 = time.time()
    while not piper.EnablePiper():
        if time.time() - t0 > 10:
            print("Enable timeout.")
            sys.exit(1)
        time.sleep(0.01)

    print("Sending command:")
    print(f"EndPoseCtrl({X},{Y},{Z},{RX},{RY},{RZ})")
    print(f"GripperCtrl({abs(GRIP)},{args.grip_speed},{hex(args.grip_mode)},0)")

    piper.MotionCtrl_2(args.motion_mode,
                       args.motion_submode,
                       args.motion_speed,
                       0x00)

    piper.EndPoseCtrl(X, Y, Z, RX, RY, RZ)
    piper.GripperCtrl(abs(GRIP), args.grip_speed, args.grip_mode, 0)

    time.sleep(args.wait)

    try:
        print("Current arm state:", piper.GetArmEndPoseMsgs())
    except Exception:
        pass

    print("Done.")


if __name__ == "__main__":
    main()
