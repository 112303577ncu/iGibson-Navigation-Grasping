#!/usr/bin/env python3
"""Supervised Rosmaster servo smoke test; dry-run unless ``--real`` is given."""
from __future__ import annotations

import argparse
import time


OPEN_HOME = [90, 90, 90, 90, 90, 30]


def main() -> int:
    parser = argparse.ArgumentParser(description="X3Plus servo smoke test")
    parser.add_argument("--port", default="/dev/myserial")
    parser.add_argument("--real", action="store_true", help="actually move S1 and the arm")
    args = parser.parse_args()

    if not args.real:
        print("[DRY] S1: 90 -> 100 deg over 2000ms")
        print(f"[DRY] open home: {OPEN_HOME} over 2000ms")
        return 0

    from Rosmaster_Lib import Rosmaster

    bot = Rosmaster(com=args.port)
    try:
        bot.create_receive_threading()
        time.sleep(1.0)
        bot.set_uart_servo_ctrl_enable(1)
        time.sleep(0.5)
        print("Moving S1 to 100 deg...")
        bot.set_uart_servo_angle(1, 100, 2000)
        time.sleep(3.0)
        print("Moving all servos to OPEN home...")
        bot.set_uart_servo_angle_array(angle_s=OPEN_HOME, run_time=2000)
        time.sleep(3.0)
    finally:
        try:
            bot.cancel_receive_threading()
        except Exception:
            pass
        serial_obj = getattr(bot, "ser", None)
        if serial_obj is not None:
            try:
                serial_obj.close()
            except Exception:
                pass
    print("Done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
