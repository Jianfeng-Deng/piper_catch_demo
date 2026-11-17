"""Fold ("归拢") an arm slowly, then power its joints down ONLY if it ended up folded.

    python stow.py --arm left            # dry run: prints the path check only
    python stow.py --arm left --execute  # actually folds (10% speed) and disables

Run this after every test so the arm never drops onto the table when control is released.
"""
import argparse
import time

import numpy as np

import common as C


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left", choices=list(C.ARM_SERIAL))
    ap.add_argument("--rate", type=int, default=10)
    ap.add_argument("--no-disable", action="store_true")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--yes", action="store_true")
    a = ap.parse_args()

    ch = C.can_if(a.arm)
    ref = C.stow_q(a.arm)
    print(f"arm={a.arm} ({ch}) stow target (deg) = {np.round(np.degrees(ref), 1)}  [from {C.STOW_FILE.name}]")
    foreign = C.detect_foreign_commands(ch, 1.0)
    if foreign:
        raise SystemExit(f"ABORT: other process is commanding this arm ({[hex(i) for i in sorted(foreign)]}).")
    p = C.connect(a.arm)
    q, _ = C.get_q(p)
    print("current joints (deg):", np.round(np.degrees(q), 1), " stowed already:", C.is_stowed(q, ref))
    path = C.path_tcp(C.load_chain(), q, C.clamp_q(ref, 0.0))
    print(f"TCP z along path: min={path[:, 2].min():.3f} max={path[:, 2].max():.3f} m")
    if not a.execute:
        print("DRY RUN: nothing sent. Add --execute.")
        return
    if not C.confirm("The arm will fold slowly. Clear space around it?", a.yes):
        raise SystemExit("cancelled")
    # hold the current pose while enabling so it cannot jump to a stale target
    for _ in range(30):
        C.send_joints(p, q, 5, margin_deg=0.0)
        time.sleep(0.01)
    if not C.enable_arm(p):
        raise SystemExit("could not enable arm")
    ok = C.stow(p, a.arm, a.rate)
    if ok and not a.no_disable:
        p.DisableArm(7)
        print("arm folded and disabled")
    elif not ok:
        print("arm NOT folded: left enabled and holding so it does not fall")


if __name__ == "__main__":
    main()
