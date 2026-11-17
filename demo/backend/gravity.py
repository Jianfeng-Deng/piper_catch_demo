"""Static gravity feedforward for MIT, ported from the console's gravity.py (same math, same wire scaling).

physical torque (N*m) -> V3 wire command:  wire = nm / c,  c = 1.0 for J1..J3 and 0.813252 for the
wrist motors J4..J6, clamped to +/-16. Uses the identified inertial parameters in piper_identified.urdf.
"""
import hashlib
import math
from functools import lru_cache
from pathlib import Path
from xml.etree import ElementTree as ET

MODEL_FILE = Path(__file__).resolve().parents[2] / "piper_identified.urdf"
WRIST_C = 0.813252
WIRE_SCALES = (1.0, 1.0, 1.0, WRIST_C, WRIST_C, WRIST_C)
WIRE_LIMIT = 16.0
G = 9.81


def _matmul3(a, b):
    return tuple(tuple(a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c] for c in range(3)) for r in range(3))


def _matvec3(m, v):
    return tuple(m[r][0] * v[0] + m[r][1] * v[1] + m[r][2] * v[2] for r in range(3))


def _add3(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _rz(angle):
    c, s = math.cos(angle), math.sin(angle)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


@lru_cache(maxsize=1)
def model(path=MODEL_FILE):
    root = ET.fromstring(Path(path).read_bytes())
    chain = []
    for i in range(1, 7):
        origin = root.find(f"joint[@name='joint{i}']").find("origin")
        r, p, y = map(float, origin.get("rpy").split())
        cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
        ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
        rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
        basis = _matmul3(_matmul3(_rz(y), ry), rx)
        offset = tuple(map(float, origin.get("xyz").split()))
        link = root.find(f"link[@name='link{i}']/inertial")
        com = tuple(map(float, link.find("origin").get("xyz").split()))
        chain.append((basis, offset, com, float(link.find("mass").get("value"))))
    return tuple(chain)


def model_sha256(path=MODEL_FILE):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def gravity_nm(q_deg):
    """Physical gravity torque (N*m) at each of the 6 joints for static pose q (degrees)."""
    if len(q_deg) != 6 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in q_deg):
        raise ValueError("invalid joint vector")
    orientation = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    pivot = (0.0, 0.0, 0.0)
    pivots, axes, centers, masses = [], [], [], []
    for q, (basis, offset, com, mass) in zip(q_deg, model()):
        pivot = _add3(pivot, _matvec3(orientation, offset))
        orientation = _matmul3(orientation, basis)
        pivots.append(pivot)
        axes.append((orientation[0][2], orientation[1][2], orientation[2][2]))
        orientation = _matmul3(orientation, _rz(math.radians(q)))
        centers.append(_add3(pivot, _matvec3(orientation, com)))
        masses.append(mass)
    out = []
    for i in range(6):
        ax, ay, _ = axes[i]
        px, py, _ = pivots[i]
        out.append(sum(G * m * (ax * (p[1] - py) - ay * (p[0] - px)) for m, p in zip(masses[i:], centers[i:])))
    return out


def wire_from_nm(nm, residual=None):
    res = residual or [0.0] * 6
    return [max(-WIRE_LIMIT, min(WIRE_LIMIT, t / c + r)) for t, c, r in zip(nm, WIRE_SCALES, res)]


def wire_references(q_deg, residual=None):
    """Same output as the console's gravity.wire_references (+ optional residual_wire)."""
    return wire_from_nm(gravity_nm(q_deg), residual)
