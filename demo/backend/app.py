"""PIPER catch-demo dashboard backend (new, independent of the <console-owner> console).

Default is READ-ONLY: it opens the four arms' feedback without sending a single CAN frame. Control
endpoints (leader->follower MIT teleop, align, stow, e-stop) exist but answer 403 unless the server was
started with DEMO_ALLOW_CONTROL=1.

    cd demo && DEMO_ALLOW_CONTROL=0 python -m uvicorn backend.app:app --host 127.0.0.1 --port 8071
"""
import asyncio
import json
import os
import secrets
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))  # common.py

import common as C  # noqa: E402
from fastapi import FastAPI, HTTPException, Request  # noqa: E402
from fastapi.responses import FileResponse, StreamingResponse  # noqa: E402
from fastapi.staticfiles import StaticFiles  # noqa: E402

from backend import gravity  # noqa: E402
from backend.arms import ARMS, ArmReader, scan_can  # noqa: E402
from backend.teleop import Teleop, TeleopError  # noqa: E402
from backend.mittest import MitTest, hold_follower  # noqa: E402
from backend import throw  # noqa: E402
from backend.posemove import PoseMove  # noqa: E402
from pydantic import BaseModel  # noqa: E402

ALLOW_CONTROL = os.environ.get("DEMO_ALLOW_CONTROL", "0") == "1"
WORKSPACE = Path(os.environ.get("PIPER_WORKSPACE", str(Path.home() / "piper-workspace"))).expanduser()
MODEL_DIR = Path(os.environ.get("PIPER_MODEL_DIR", str(WORKSPACE / "public/model/piper")))
PROFILE_PATH = Path(os.environ.get("PIPER_PROFILE", str(WORKSPACE / "configs/piper.json")))
TOKEN = secrets.token_urlsafe(16)

READERS = {n: ArmReader(n) for n in ARMS}
TELEOPS = {}
MITTESTS = {}
POSEMOVES = {}
LATEST = {"t": 0, "arms": {}, "teleop": {}}
CHAIN = C.load_chain()
PROFILE = {}


def load_profile():
    global PROFILE
    try:
        PROFILE = json.loads(PROFILE_PATH.read_text())
    except Exception:  # noqa: BLE001
        PROFILE = {}


def connect_all():
    ports = scan_can()
    for r in READERS.values():
        if r.piper is None:
            r.connect(ports)
    axes = PROFILE.get("mit", {}).get("axes", {})
    grip = PROFILE.get("control", {}).get("gripper_effort_raw", 1000)
    for side in ("left", "right"):
        f, m = READERS[f"{side}_follower"], READERS[f"{side}_master"]
        if side not in POSEMOVES and f.piper is not None:
            POSEMOVES[side] = PoseMove(f, side)
        if side not in MITTESTS and f.piper is not None:
            MITTESTS[side] = MitTest(f, side, axes.get(side) or {"kp": [10.0] * 6, "kd": [0.8] * 6, "residual_wire": [0.0] * 6})
        if side not in TELEOPS and f.piper is not None and m.piper is not None:
            cfg = axes.get(side) or {"kp": [10.0] * 6, "kd": [0.8] * 6, "residual_wire": [0.0] * 6}
            TELEOPS[side] = Teleop(m, f, side, cfg, gripper_effort_raw=grip)


def derive(s):
    """Add TCP pose and the static-gravity comparison used for MIT feed-forward."""
    if not s.get("present"):
        return s
    import math

    q = [math.radians(x) for x in s["q_deg"]]
    try:
        s["tcp_m"] = [round(float(x), 4) for x in C.fk_tcp(CHAIN, q)]
        g = gravity.gravity_nm(s["q_deg"])
        s["gravity_nm"] = g
        s["gravity_wire"] = gravity.wire_from_nm(g)
        s["effort_minus_gravity_nm"] = [e - x for e, x in zip(s["effort_nm"], g)]
    except Exception as exc:  # noqa: BLE001
        s["derive_error"] = str(exc)
    return s


def sampler():
    last_scan = 0.0
    while True:
        try:
            if time.time() - last_scan > 3.0:  # pick up adapters that appear later (leaders powered on)
                last_scan = time.time()
                if any(r.piper is None for r in READERS.values()):
                    connect_all()
            arms = {n: derive(r.snapshot()) for n, r in READERS.items()}
            LATEST.update(t=time.time(), arms=arms, teleop={s: t.status() for s, t in TELEOPS.items()},
                          mittest={s: t.status() for s, t in MITTESTS.items()}, posemove={s: t.status() for s, t in POSEMOVES.items()})
        except Exception as exc:  # noqa: BLE001
            LATEST["sampler_error"] = f"{type(exc).__name__}: {exc}"
        time.sleep(0.04)


app = FastAPI(title="PIPER catch demo")


LAG = {"recent": []}


async def lag_monitor():
    """Server-side diagnostic: how late does the asyncio loop wake up (GIL / CPU starvation)?"""
    while True:
        t0 = time.perf_counter()
        await asyncio.sleep(0.05)
        LAG["recent"] = (LAG["recent"] + [(time.perf_counter() - t0 - 0.05) * 1e3])[-200:]
        LATEST["loop_lag_ms"] = {"now": round(LAG["recent"][-1], 1), "max_10s": round(max(LAG["recent"]), 1)}


@app.on_event("startup")
async def _startup():
    load_profile()
    connect_all()
    threading.Thread(target=sampler, daemon=True, name="sampler").start()
    asyncio.create_task(lag_monitor())


def guard(request: Request, control=True):
    """Mutations: control must be unlocked, token present, same origin."""
    if control and not ALLOW_CONTROL:
        raise HTTPException(403, "服务以只读模式启动（DEMO_ALLOW_CONTROL=0），控制已锁定")
    if request.headers.get("x-demo-token") != TOKEN:
        raise HTTPException(403, "token 无效")
    origin = request.headers.get("origin")
    if origin and origin.split("://", 1)[-1] != request.headers.get("host"):
        raise HTTPException(403, "跨源请求被拒绝")


@app.get("/health")
def health():
    return {"ok": True, "allow_control": ALLOW_CONTROL, "gravity_model_sha256": gravity.model_sha256()}


def stow_poses():
    try:
        d = json.loads(C.STOW_FILE.read_text())
        return {side: d.get(side, {}).get("deg") for side in ("left", "right")}
    except Exception:  # noqa: BLE001
        return {"left": None, "right": None}


@app.get("/api/config")
def config():
    bases = {s: PROFILE.get("arms", {}).get(s, {}) for s in ("left", "right")}
    return {"allow_control": ALLOW_CONTROL, "token": TOKEN, "arms": {n: {k: v for k, v in a.items() if k != "serial"} for n, a in ARMS.items()},
            "bases": bases, "urdf": "/model/piper_description_v100_adapter.urdf", "package_root": "/model",
            "mit": {"kp": [10.0] * 6, "kd": [0.8] * 6, **{s: PROFILE.get("mit", {}).get("axes", {}).get(s) for s in ("left", "right")}},
            "joint_limits_deg": C.JOINT_LIMITS_DEG.tolist(), "tcp_z_m": C.TCP_Z, "stow": stow_poses()}


@app.get("/api/throw/presets")
def throw_presets():
    return {k: {"label": v["label"], "kind": "oblique" if "j1" in v else "vertical", "catch_frac": v.get("catch_frac", 0.75)} for k, v in throw.PRESETS.items()}


@app.get("/api/throw/plan")
def throw_plan(preset: str = "mid", aim: float = -50.0, catch_frac: float | None = None):
    """Offline up-toss simulation for the 3D preview. Pure computation: sends nothing to any arm."""
    try:
        return throw.plan_throw(preset, aim, {s: PROFILE.get("arms", {}).get(s) for s in ("left", "right")}, catch_frac=catch_frac)
    except ValueError as exc:
        raise HTTPException(400, str(exc))


@app.get("/api/state")
def state():
    return LATEST


@app.get("/api/stream")
async def stream(request: Request):
    async def gen():
        while True:
            if await request.is_disconnected():
                break
            yield f"data: {json.dumps(LATEST)}\n\n"
            await asyncio.sleep(0.05)

    return StreamingResponse(gen(), media_type="text/event-stream", headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


def teleop_of(side):
    t = TELEOPS.get(side)
    if t is None:
        raise HTTPException(409, "该侧主臂或从臂的 USB-CAN 未连接")
    return t


@app.post("/api/teleop/heartbeat")
def heartbeat(request: Request):
    guard(request)
    for t in list(TELEOPS.values()) + list(MITTESTS.values()) + list(POSEMOVES.values()):
        t.heartbeat()
    return {"ok": True}


@app.post("/api/hold")
def hold_all(request: Request):
    """急停保持: stop every motion session, leave MIT, hold each follower where it IS (enabled, not limp)."""
    guard(request)
    for t in list(TELEOPS.values()) + list(MITTESTS.values()) + list(POSEMOVES.values()):
        t.stop_evt.set()
    out = {}
    for rnd in range(3):  # repeat: a loop thread may have one more frame in flight
        for side in ("left", "right"):
            f = READERS[f"{side}_follower"]
            if f.piper is None:
                continue
            try:
                hold_follower(f)
                out[side] = {"ok": True, "mode_feed": int(f.piper.GetArmStatus().arm_status.mode_feed)}
            except Exception as exc:  # noqa: BLE001
                out[side] = {"ok": False, "error": str(exc)}
        time.sleep(0.08)
    return out


class PoseReq(BaseModel):
    q_deg: list[float]
    gripper_mm: float | None = None


def posemove_of(side):
    m = POSEMOVES.get(side)
    if m is None:
        raise HTTPException(409, "该侧从臂的 USB-CAN 未连接")
    return m


@app.post("/api/pose/{side}/check")
def pose_check(side: str, req: PoseReq, request: Request):
    """Read-only: limits, path height, travel/time, static gravity torque for a target pose. Sends nothing."""
    guard(request, control=False)
    return posemove_of(side).check(req.q_deg)


@app.post("/api/pose/{side}/move")
def pose_move(side: str, req: PoseReq, request: Request):
    guard(request)
    if req.gripper_mm is not None and not (0.0 <= req.gripper_mm <= 100.0):
        raise HTTPException(400, "夹爪开度必须在 0~100 mm")
    try:
        return posemove_of(side).start(req.q_deg, req.gripper_mm)
    except ValueError as exc:
        raise HTTPException(409, str(exc))


@app.post("/api/fw")
def fw_query(request: Request):
    guard(request)
    out = {}
    for side, m in MITTESTS.items():
        try:
            out[side] = m.fw()
        except Exception as exc:  # noqa: BLE001
            out[side] = {"version": None, "ok": False, "error": str(exc)}
    return out


@app.post("/api/mittest/{side}/{step}")
def mittest(side: str, step: str, request: Request):
    guard(request)
    m = MITTESTS.get(side)
    if m is None:
        raise HTTPException(409, "该侧从臂的 USB-CAN 未连接")
    try:
        if step == "stop":
            m.stop()
        else:
            m.start(step)
    except ValueError as exc:
        raise HTTPException(409, str(exc))
    return m.status()


@app.post("/api/teleop/{side}/preflight")
def preflight(side: str, request: Request):
    guard(request, control=False)  # read-only listen; allowed while locked
    return teleop_of(side).preflight()


@app.post("/api/teleop/{side}/{action}")
def teleop_action(side: str, action: str, request: Request):
    guard(request)
    t = teleop_of(side)
    try:
        if action == "align":
            t.heartbeat()
            t.align()
        elif action == "start":
            t.start()
        elif action == "stop":
            t.stop()
        else:
            raise HTTPException(404, "unknown action")
    except TeleopError as exc:
        raise HTTPException(409, str(exc))
    return t.status()


@app.post("/api/estop")
def estop(request: Request):
    guard(request)
    return {"errors": {s: t.estop() for s, t in TELEOPS.items()}}


@app.post("/api/estop/resume")
def estop_resume(request: Request):
    guard(request)
    for t in TELEOPS.values():
        t.resume()
    return {"ok": True}


@app.post("/api/follower/{side}/stow")
def stow(side: str, request: Request):
    guard(request)
    t = teleop_of(side)
    if t.active():
        raise HTTPException(409, "遥操进行中，请先停止")
    if C.detect_foreign_commands(t.follower.can, 0.5):
        raise HTTPException(409, "从臂总线上有其它控制者")

    def run():
        p = t.follower.piper
        try:
            q, _ = C.get_q(p)
            with t.follower.tx_lock:
                for _ in range(30):
                    C.send_joints(p, q, 5, margin_deg=0.0)
                    time.sleep(0.01)
            if C.enable_arm(p):
                C.stow_then_disable(p, side)
        except Exception as exc:  # noqa: BLE001
            t._set("fault", error=f"归拢失败: {exc}")

    threading.Thread(target=run, daemon=True, name=f"stow-{side}").start()
    return {"ok": True, "message": "正在慢速归拢，到位后才会失能"}


if MODEL_DIR.exists():
    app.mount("/model", StaticFiles(directory=str(MODEL_DIR)), name="model")
app.mount("/", StaticFiles(directory=str(ROOT / "frontend"), html=True), name="frontend")
