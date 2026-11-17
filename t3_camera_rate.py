"""T3: RealSense D435 depth / IR stream rate, drops, latency, depth validity. No robot commands.

    python t3_camera_rate.py --serial 262022070197 --secs 5
    python t3_camera_rate.py --serial 262022070197 --modes 848x480@90 848x100@300

Opens ONLY depth + left-IR streams (never colour) and never sets device options, so it does not
disturb another process that is streaming colour from the same camera.
SDK serials: top=262022070197, left(wrist)=261922074380, right(wrist)=262322073587
"""
import argparse
import time

import numpy as np
import pyrealsense2 as rs


def run_mode(serial, w, h, fps, secs, with_depth=True):
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.infrared, 1, w, h, rs.format.y8, fps)
    if with_depth:
        cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    pipe = rs.pipeline()
    try:
        prof = pipe.start(cfg)
    except Exception as e:  # noqa: BLE001
        print(f"  start FAILED: {e}")
        return
    depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
    ir_ts, d_ts, ir_no, lat, exp, valid, med = [], [], [], [], [], [], []
    t_end = time.time() + secs
    n_timeouts = 0
    try:
        while time.time() < t_end:
            try:
                fs = pipe.wait_for_frames(2000)
            except RuntimeError:
                n_timeouts += 1
                continue
            now_ms = time.time() * 1e3
            ir = fs.get_infrared_frame(1)
            if ir:
                ir_ts.append(ir.get_timestamp())
                ir_no.append(ir.get_frame_number())
                lat.append(now_ms - ir.get_timestamp())
                if ir.supports_frame_metadata(rs.frame_metadata_value.actual_exposure):
                    exp.append(ir.get_frame_metadata(rs.frame_metadata_value.actual_exposure))
            if with_depth:
                d = fs.get_depth_frame()
                if d:
                    d_ts.append(d.get_timestamp())
                    a = np.asanyarray(d.get_data())
                    hh, ww = a.shape
                    roi = a[int(hh * .3):int(hh * .7), int(ww * .3):int(ww * .7)]
                    valid.append(float((roi > 0).mean()))
                    nz = roi[roi > 0]
                    if nz.size:
                        med.append(float(np.median(nz)) * depth_scale)
    finally:
        pipe.stop()

    WARM = 15  # first frames carry unsynchronised timestamps; drop them

    def rate(ts, no, name):
        if len(ts) < WARM + 5:
            print(f"  {name}: too few frames ({len(ts)})")
            return
        t = np.array(ts[WARM:])
        no = no[WARM:] if no else no
        dt = np.diff(t)
        fps_meas = (len(t) - 1) / ((t[-1] - t[0]) / 1e3)
        drops = int(np.sum(np.diff(no) > 1)) if no else -1
        print(f"  {name}: n={len(t)} fps={fps_meas:6.1f}  dt mean={dt.mean():.2f} std={dt.std():.2f} max={dt.max():.2f} ms  "
              f"frame-no gaps={drops}")

    rate(ir_ts, ir_no, "IR1  ")
    if with_depth:
        rate(d_ts, [], "depth")
        if valid:
            print(f"  depth centre-ROI valid={np.mean(valid) * 100:.1f}%  median range={np.median(med) if med else float('nan'):.2f} m")
    if len(lat) > WARM + 5:
        L = np.array(lat[WARM:])
        print(f"  host-clock minus frame timestamp: median={np.median(L):.1f} ms, p95={np.percentile(L, 95):.1f} ms, "
              f"max={L.max():.1f} ms (includes camera/host clock offset; use for trend, not absolute latency)")
    if len(exp) > WARM + 5:
        E = np.array(exp[WARM:])
        print(f"  IR exposure: median={np.median(E):.0f} us, p95={np.percentile(E, 95):.0f} us, max={E.max():.0f} us "
              f"(frame period {1e6 / fps:.0f} us)")
    if n_timeouts:
        print(f"  frame timeouts: {n_timeouts}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", default="262022070197")
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--modes", nargs="+", default=["848x480@90", "848x480@60", "848x100@300"])
    a = ap.parse_args()
    for m in a.modes:
        res, f = m.split("@")
        w, h = (int(x) for x in res.split("x"))
        print(f"\n== {a.serial} {m} for {a.secs:.0f}s ==")
        run_mode(a.serial, w, h, int(f), a.secs)


if __name__ == "__main__":
    main()
