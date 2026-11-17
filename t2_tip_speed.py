"""T2: max TCP speed / acceleration of one arm during fast joint-space swings.

Dry-run by default (prints the planned poses, TCP path, height margin, FK-only checks).
To move the arm:  --execute   (arm must have clear space; keep the e-stop in reach)

    python t2_tip_speed.py --arm left --rates 10 20 30 50 70 100 --reps 3 --execute

Method
- Go to the ready pose slowly, then swing ready -> ready+dq -> ready (MOVE J, firmware
  interpolation) at increasing speed-rate settings. J1 stays fixed so the swing stays in the
  arm's own vertical plane (no sweep toward the other arm).
- Record joint feedback (~200 Hz), compute TCP speed/accel with URDF FK.
- Ramp is aborted when peak joint speed exceeds --vmax (rad/s) or arm status != normal.
- Also reports a ballistic "if released at peak speed" range to size the throw.
Uses MOVE-J position mode only (no MIT mode).
"""
import argparse
import csv
import time
from datetime import datetime

import numpy as np

import common as C


def plan_check(chain, q0, dq, z_min):
    q1 = q0 + dq
    print("ready q (deg):", np.round(np.degrees(q0), 1))
    print("swing q (deg):", np.round(np.degrees(q1), 1))
    ok = True
    for name, q in (("ready", q0), ("swing", q1)):
        if not C.within_limits(q):
            print(f"  !! {name} pose violates joint limits (margin {C.LIMIT_MARGIN_DEG} deg)")
            ok = False
    path = C.path_tcp(chain, q0, q1)
    print("TCP ready (m):", np.round(path[0], 3), " TCP swing (m):", np.round(path[-1], 3))
    print(f"TCP z along path: min={path[:, 2].min():.3f} max={path[:, 2].max():.3f} m (base frame, z up)")
    print(f"TCP x range {path[:, 0].min():.3f}..{path[:, 0].max():.3f}  y range {path[:, 1].min():.3f}..{path[:, 1].max():.3f}")
    if path[:, 2].min() < z_min:
        print(f"  !! TCP dips below z_min={z_min} m (table/base collision risk)")
        ok = False
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left", choices=list(C.ARM_CAN))
    ap.add_argument("--rates", type=int, nargs="+", default=[10, 20, 30, 50, 70, 100])
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--dq-deg", type=float, nargs=6, default=[0, 25, -25, 0, 0, 0],
                    help="swing offset from the ready pose (deg, 6 joints)")
    ap.add_argument("--ready-deg", type=float, nargs=6, default=None, help="override ready pose (deg)")
    ap.add_argument("--z-min", type=float, default=0.06, help="min allowed TCP height (m, base frame)")
    ap.add_argument("--vmax", type=float, default=3.0, help="abort ramp if a joint exceeds this (rad/s)")
    ap.add_argument("--dwell", type=float, default=0.4)
    ap.add_argument("--payload", default="", help="label, e.g. gripper+bottle")
    ap.add_argument("--no-stow", action="store_true", help="do NOT fold+disable at the end (default folds first)")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--yes", action="store_true")
    a = ap.parse_args()

    chain = C.load_chain()
    q0 = np.radians(a.ready_deg) if a.ready_deg else C.READY_Q.copy()
    dq = np.radians(a.dq_deg)
    print(f"arm={a.arm} ({C.can_if(a.arm)}) rates={a.rates} reps={a.reps} payload='{a.payload}'")
    if not plan_check(chain, q0, dq, a.z_min):
        raise SystemExit("plan failed safety checks; adjust --dq-deg / --ready-deg / --z-min")
    if not a.execute:
        print("\nDRY RUN: nothing sent. Add --execute to move the arm.")
        return

    foreign = C.detect_foreign_commands(C.can_if(a.arm), 1.0)
    if foreign:
        raise SystemExit(f"ABORT: other process is commanding this arm (CAN IDs {[hex(i) for i in sorted(foreign)]}).")
    if not C.confirm("The arm will move quickly. Workspace clear? E-stop in reach?", a.yes):
        raise SystemExit("cancelled")

    p = C.connect(a.arm)
    if C.arm_status_code(p) not in (0, None):
        raise SystemExit(f"arm status = {C.arm_status_code(p)} (not normal); clear it first")
    q_start, _ = C.get_q(p)
    path0 = C.path_tcp(chain, q_start, q0)
    print("start joints (deg):", np.round(np.degrees(q_start), 1), f" path to ready: TCP z min={path0[:, 2].min():.3f} m")
    if path0[:, 2].min() < a.z_min:
        raise SystemExit("path from current pose to the ready pose dips below z_min; aborting")
    # hold the CURRENT pose while enabling so the arm cannot jump to a stale target
    for _ in range(30):
        C.send_joints(p, q_start, 5, margin_deg=0.0)
        time.sleep(0.01)
    if not C.enable_arm(p):
        raise SystemExit("could not enable arm")
    for _ in range(30):
        C.send_joints(p, q_start, 5, margin_deg=0.0)
        time.sleep(0.01)

    tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    summary = []
    try:
        print("moving to ready pose at 10% ...")
        if not C.move_to(p, q0, 10, timeout=20):
            raise SystemExit("did not reach ready pose")
        for rate in a.rates:
            for rep in range(a.reps):
                time.sleep(a.dwell)
                rec = C.Recorder(p).start()
                t_start = time.time()
                ok1 = C.move_to(p, q0 + dq, rate, timeout=8)
                ok2 = C.move_to(p, q0, rate, timeout=8)
                rows = rec.stop()
                if len(rows) < 20:
                    print("too few samples"); continue
                t = rows[:, 1]
                q = rows[:, 2:8]
                t, qs, pos, vel, speed, acc = C.tip_kinematics(chain, t, q)
                jv = np.abs(np.gradient(qs, t, axis=0))
                i_pk = int(np.argmax(speed))
                tf, rng, land = C.ballistic_from_release(pos[i_pk], vel[i_pk])
                row = {
                    "rate": rate, "rep": rep, "reached": ok1 and ok2,
                    "peak_tip_speed": float(speed.max()), "peak_tip_acc": float(acc.max()),
                    "peak_joint_speed_rad_s": float(jv.max()) if len(jv) else float("nan"),
                    "vz_at_peak": float(vel[i_pk][2]), "vh_at_peak": float(np.hypot(vel[i_pk][0], vel[i_pk][1])),
                    "ballistic_flight_s": tf, "ballistic_range_m": rng,
                }
                summary.append(row)
                np.savetxt(C.HERE / "data" / f"t2_{a.arm}_{tag}_r{rate}_{rep}.csv",
                           np.column_stack([t - t[0], qs, pos, vel, speed]), delimiter=",",
                           header="t,q1,q2,q3,q4,q5,q6,x,y,z,vx,vy,vz,speed", comments="")
                print(f"rate {rate:3d}% rep {rep}: peak tip {row['peak_tip_speed']:.2f} m/s, "
                      f"acc {row['peak_tip_acc']:.1f} m/s^2, joint {row['peak_joint_speed_rad_s']:.2f} rad/s, "
                      f"ballistic range {rng:.2f} m, reached={row['reached']}")
                if not (ok1 and ok2) or row["peak_joint_speed_rad_s"] > a.vmax or C.arm_status_code(p) not in (0, None):
                    print("STOP ramp: not reached / joint speed over --vmax / arm status abnormal")
                    raise StopIteration
    except StopIteration:
        pass
    except KeyboardInterrupt:
        print("interrupted: holding position")
    finally:
        try:
            C.hold_here(p, 10)  # stop motion first
            if a.no_stow:
                print("--no-stow: arm left enabled at its last position")
            else:
                C.stow_then_disable(p, a.arm)  # fold, and only then power down
        except Exception as e:  # noqa: BLE001
            print("stow/hold failed:", e, "- arm left enabled and holding")

    if summary:
        path = C.HERE / "data" / f"t2_summary_{a.arm}_{tag}.csv"
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(summary[0]))
            w.writeheader(); w.writerows(summary)
        print(f"\nsaved {path}")
        print("\n=== max over reps, per speed-rate ===")
        for rate in sorted({r["rate"] for r in summary}):
            sub = [r for r in summary if r["rate"] == rate]
            print(f"rate {rate:3d}%: tip speed {max(r['peak_tip_speed'] for r in sub):.2f} m/s, "
                  f"acc {max(r['peak_tip_acc'] for r in sub):.1f} m/s^2")


if __name__ == "__main__":
    main()
