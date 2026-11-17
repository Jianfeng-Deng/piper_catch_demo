"""Save the arm's CURRENT (real, resting) joint angles as its folded/stow pose. Passive: sends nothing.

    python save_stow_pose.py --arm left       # or: right, both

Put the arm in the pose you want it to rest in (wrist already sagged as it would after power-down),
then run this. stow.py / t1 / t2 fold to this pose and only then disable the joints.
Writes stow_pose.json next to common.py.
"""
import argparse
import json
import struct
import time

import can
import numpy as np

import common as C


def read_q_deg(ch, secs=1.5):
    bus = can.interface.Bus(channel=ch, interface="socketcan")
    rows, last = [], {}
    t0 = time.time()
    while time.time() - t0 < secs:
        m = bus.recv(0.1)
        if m is not None and m.arbitration_id in (0x2A5, 0x2A6, 0x2A7):
            last[m.arbitration_id] = struct.unpack(">ii", bytes(m.data))
            if len(last) == 3 and m.arbitration_id == 0x2A7:
                rows.append([x for i in (0x2A5, 0x2A6, 0x2A7) for x in last[i]])
    bus.shutdown()
    q = np.array(rows) * 1e-3
    if len(q) < 50:
        raise SystemExit(f"{ch}: not enough feedback frames")
    if q.std(axis=0).max() > 0.05:
        raise SystemExit(f"{ch}: arm is moving (std {q.std(axis=0).max():.3f} deg); keep it still")
    return q.mean(axis=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left", choices=["left", "right", "both"])
    a = ap.parse_args()
    arms = ["left", "right"] if a.arm == "both" else [a.arm]
    try:
        data = json.loads(C.STOW_FILE.read_text())
    except Exception:  # noqa: BLE001
        data = {}
    for arm in arms:
        q = read_q_deg(C.can_if(arm))
        data[arm] = {"deg": [round(float(x), 2) for x in q], "saved": time.strftime("%Y-%m-%d %H:%M:%S")}
        print(arm, "stow pose (deg):", data[arm]["deg"])
    C.STOW_FILE.write_text(json.dumps(data, indent=2))
    print("saved", C.STOW_FILE)


if __name__ == "__main__":
    main()
