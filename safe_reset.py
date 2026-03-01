#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys

from piper_sdk import C_PiperInterface_V2


def parse_args():
    parser = argparse.ArgumentParser(
        description="Reset Piper after a single confirmation prompt."
    )
    parser.add_argument("--can", default="can0", help="CAN interface (default: can0)")
    parser.add_argument(
        "--force",
        action="store_true",
        help="跳过确认，直接执行重置（高风险）",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    print(f"连接机械臂: {args.can}")
    piper = C_PiperInterface_V2(args.can)
    piper.ConnectPort()

    if not args.force:
        print("警告: ResetPiper() 会让机械臂立刻失电落下，并清除所有错误和内部标志位。")
        answer = input("确认执行重置？输入 yes 继续: ").strip().lower()
        if answer != "yes":
            print("已取消重置。")
            sys.exit(1)

    print("执行 ResetPiper() ...")
    piper.ResetPiper()
    print("已发送重置指令。")


if __name__ == "__main__":
    main()
