"""T0: passive CAN feedback rate / jitter check. Sends NOTHING (receive-only).

    python t0_can_jitter.py [--secs 10] [--arms left right]

Reports, per arm, for joint feedback (0x2A5), gripper feedback (0x2A8) and
high-speed feedback (0x251): rate, inter-arrival mean/std/P99/max, and gap count.
"""
import argparse

import numpy as np

from common import ARM_CAN, can_if


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=10.0)
    ap.add_argument("--arms", nargs="+", default=["left", "right"], choices=list(ARM_CAN))
    a = ap.parse_args()

    import can
    import time

    ids = {0x2A5: "joint12 (0x2A5)", 0x2A8: "gripper (0x2A8)", 0x251: "joint1 hs (0x251)"}
    for arm in a.arms:
        ch = can_if(arm)
        try:
            bus = can.interface.Bus(channel=ch, interface="socketcan")
        except TypeError:
            bus = can.interface.Bus(channel=ch, bustype="socketcan")
        ts = {i: [] for i in ids}
        t0 = time.time()
        while time.time() - t0 < a.secs:
            m = bus.recv(0.1)
            if m is not None and m.arbitration_id in ts:
                # hardware/kernel RX timestamp when available (m.timestamp), else wall clock
                ts[m.arbitration_id].append(m.timestamp or time.time())
        bus.shutdown()
        print(f"\n== {arm} ({ch}) {a.secs:.0f}s ==")
        for i, name in ids.items():
            t = np.array(ts[i])
            if len(t) < 10:
                print(f"{name:20s} too few frames ({len(t)})")
                continue
            dt = np.diff(t) * 1e3
            nominal = np.median(dt)
            print(
                f"{name:20s} n={len(t):5d} rate={1e3 / dt.mean():6.1f} Hz  "
                f"dt mean={dt.mean():.2f} std={dt.std():.2f} p99={np.percentile(dt, 99):.2f} "
                f"max={dt.max():.2f} ms  gaps>2x={int((dt > 2 * nominal).sum())}"
            )


if __name__ == "__main__":
    main()
