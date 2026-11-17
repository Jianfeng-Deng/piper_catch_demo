"""T1: gripper command -> motion latency (open and close). Arm joints do NOT move.

Dry-run by default. To actually command the gripper:  --execute

    python t1_gripper_latency.py --arm left --open-mm 60 --close-mm 10 --reps 20 --execute

Per rep it reports, for each direction:
  latency   command sent -> first feedback change
  t10/t50/t90   time to 10/50/90 % of travel
  settle    time until within 0.5 mm of the final value
Feedback arrives at ~200 Hz so time resolution is about +/-5 ms.
Use --with-object when a bottle/cup is clamped, to label the CSV (the object must
be placed by you; close-mm should be BELOW the object width so the gripper stalls on it).
Hands clear of the fingers: the gripper can pinch.
"""
import argparse
import csv
import time
from datetime import datetime

import numpy as np

import common as C


def analyse(rows, t_send, target_mm):
    """rows: Recorder array. Returns dict of timings (ms) or None if no motion."""
    t = rows[:, 8]  # gripper sdk timestamp
    g = rows[:, 9]
    tw = rows[:, 0]
    # sanity on clocks: SDK timestamp should be close to wall clock
    skew = float(np.median(tw - t))
    use_t = t if abs(skew) < 0.05 else tw
    pre = (use_t > t_send - 0.15) & (use_t < t_send)
    if pre.sum() < 3:
        return None
    g0 = float(np.median(g[pre]))
    post = use_t >= t_send
    tt, gg = use_t[post] - t_send, g[post]
    if len(tt) < 5:
        return None
    tail = tt > (tt[-1] - 0.3)
    gf = float(np.median(gg[tail]))
    travel = gf - g0
    out = {"g0_mm": g0, "gf_mm": gf, "travel_mm": travel, "clock_skew_ms": skew * 1e3}
    if abs(travel) < 2.0:
        out["note"] = "no_motion"
        return out
    mov = np.abs(gg - g0) > max(0.3, 0.03 * abs(travel))
    out["latency_ms"] = float(tt[np.argmax(mov)] * 1e3) if mov.any() else float("nan")
    for frac in (0.1, 0.5, 0.9):
        hit = np.abs(gg - g0) >= frac * abs(travel)
        out[f"t{int(frac * 100)}_ms"] = float(tt[np.argmax(hit)] * 1e3) if hit.any() else float("nan")
    bad = np.where(np.abs(gg - gf) > 0.5)[0]
    out["settle_ms"] = float(tt[bad[-1]] * 1e3) if len(bad) else 0.0
    return out


def summarize(label, vals):
    v = np.array([x for x in vals if x == x])
    if len(v) == 0:
        print(f"  {label:10s} no data")
        return
    print(
        f"  {label:10s} n={len(v):2d} mean={v.mean():6.1f}  std={v.std():5.1f}  "
        f"p95={np.percentile(v, 95):6.1f}  max={v.max():6.1f} ms"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left", choices=list(C.ARM_CAN))
    ap.add_argument("--open-mm", type=float, default=60.0)
    ap.add_argument("--close-mm", type=float, default=10.0)
    ap.add_argument("--effort", type=float, default=1.0, help="gripper effort N*m (0..5)")
    ap.add_argument("--reps", type=int, default=20)
    ap.add_argument("--settle-s", type=float, default=1.5)
    ap.add_argument("--with-object", default="", help="label, e.g. bottle-65mm")
    ap.add_argument("--enable-arm", action="store_true",
                    help="enable joints + CAN control mode first (holds the current pose at 5%%)")
    ap.add_argument("--disable-after", action="store_true",
                    help="with --enable-arm: disable joints again at the end if at the rest pose")
    ap.add_argument("--execute", action="store_true", help="actually send commands")
    ap.add_argument("--yes", action="store_true", help="skip the YES prompt")
    a = ap.parse_args()

    print(f"arm={a.arm} ({C.can_if(a.arm)}) open={a.open_mm}mm close={a.close_mm}mm "
          f"effort={a.effort}N*m reps={a.reps} object='{a.with_object}'")
    if not a.execute:
        print("DRY RUN: nothing sent. Add --execute to run.")
        return

    foreign = C.detect_foreign_commands(C.can_if(a.arm), 1.0)
    if foreign:
        raise SystemExit(f"ABORT: other process is commanding this arm (CAN IDs {[hex(i) for i in sorted(foreign)]}).")
    if not C.confirm("Gripper will open/close repeatedly; keep hands clear.", a.yes):
        raise SystemExit("cancelled")

    p = C.connect(a.arm)
    q_start, _ = C.get_q(p)
    print("joints now (deg):", np.round(np.degrees(q_start), 1), " gripper now (mm): %.1f" % C.get_gripper_mm(p)[0])
    if a.enable_arm:
        # Hold the CURRENT pose before/while enabling so the arm cannot jump to a stale target.
        for _ in range(30):
            C.send_joints(p, q_start, 5, margin_deg=0.0)
            time.sleep(0.01)
        if not C.enable_arm(p):
            raise SystemExit("could not enable arm")
        for _ in range(30):
            C.send_joints(p, q_start, 5, margin_deg=0.0)
            time.sleep(0.01)
        print("arm enabled (CAN control mode, holding current pose); ctrl status =", C.arm_status_code(p))
    rec = C.Recorder(p).start()
    tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    results = []
    try:
        # enable gripper + go to open and wait
        C.send_gripper(p, a.open_mm, a.effort)
        time.sleep(2.0)
        for rep in range(a.reps):
            for direction, target in (("close", a.close_mm), ("open", a.open_mm)):
                mark = len(rec.rows)
                time.sleep(0.4)  # quiet baseline before the step
                t_send = time.time()
                C.send_gripper(p, target, a.effort)
                t_end = t_send + a.settle_s
                while time.time() < t_end:  # re-send same target to be robust to a lost frame
                    time.sleep(0.02)
                    C.send_gripper(p, target, a.effort)
                rows = np.array(rec.rows[max(0, mark - 300):], float)
                r = analyse(rows, t_send, target)
                if r is None:
                    print(f"rep {rep} {direction}: not enough data")
                    continue
                r.update({"rep": rep, "dir": direction, "target_mm": target})
                results.append(r)
                print(f"rep {rep:2d} {direction:5s} lat={r.get('latency_ms', float('nan')):6.1f} "
                      f"t50={r.get('t50_ms', float('nan')):6.1f} t90={r.get('t90_ms', float('nan')):6.1f} "
                      f"settle={r.get('settle_ms', float('nan')):6.1f} ms  travel={r['travel_mm']:.1f} mm"
                      + ("  [NO MOTION]" if r.get("note") else ""))
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        rec.stop()
        # leave the gripper open (safe, releases any object)
        C.send_gripper(p, a.open_mm, a.effort)
        if a.enable_arm and a.disable_after:
            q_now, _ = C.get_q(p)
            if C.is_stowed(q_now, C.stow_q(a.arm)):  # at the saved folded pose: safe to power the joints down
                p.DisableArm(7)
                print("arm disabled again (it was at the saved folded pose)")
            else:
                print("arm NOT disabled (not at the saved folded pose, it would drop); it stays enabled and holding")

    path =C.HERE / "data" / f"t1_gripper_{a.arm}_{tag}.csv"
    if results:
        keys = sorted({k for r in results for k in r})
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(results)
        print(f"\nsaved {path}")
    print(f"\n=== summary ({a.arm}, object='{a.with_object}') ===")
    for d in ("close", "open"):
        sub = [r for r in results if r["dir"] == d and not r.get("note")]
        print(f"{d}:")
        for k in ("latency_ms", "t10_ms", "t50_ms", "t90_ms", "settle_ms"):
            summarize(k, [r.get(k, float("nan")) for r in sub])
    print("NOTE: latency std/p95 is what matters for release timing (error = v_tip * jitter).")


if __name__ == "__main__":
    main()
