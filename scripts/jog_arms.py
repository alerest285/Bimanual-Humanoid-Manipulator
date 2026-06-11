#!/usr/bin/env python
"""Keyboard jog — drive the YAM arms MANUALLY, no headset, for sim→real checks.

Runs the same ArmIK + sinks as teleop, so a keypress in sim and a keypress on the
Linux host produce the same joint commands (hardware additionally passes through
the JointCommandShaper). Watch it live on the dashboard / Unity / --viz while
jogging.

    uv run python scripts/jog_arms.py                    # render sink (sim, default)
    uv run python scripts/jog_arms.py --sink hw          # REAL arms (Linux host)
    uv run python scripts/jog_arms.py --sink hw --side right   # single-arm bring-up

`--sink hw` never touches the ORCA hands (the jog has no hand keys) and tees
commands to the render stream so the dashboard shows the jog live.

Keys:
    TAB        switch side (left/right)         1..6   select joint
    = / -      jog selected joint + / - step    [ / ]  halve / double step
    w / s      EE forward / back                a / d  EE left / right
    r / f      EE up / down                     h      go home (rest pose)
    p          print state                      q,ESC  quit
"""
from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from bimanual_teleop.config import SIDES, load_rig                  # noqa: E402
from bimanual_teleop.arms.ik import ArmIK                           # noqa: E402
from bimanual_teleop.vr.frames import SE3, quat_to_R                # noqa: E402


class _ArmShim:
    """Just enough controller surface for RenderSink.build_state()."""

    def __init__(self, rig: dict, side: str, ik: ArmIK):
        self.ik = ik
        self.base_R = quat_to_R(rig["arms"][side]["base_quat"])
        self.base_pos = np.asarray(rig["arms"][side]["base_pos"], dtype=float)
        self.cmd_pos = None
        self.cmd_R = None


class _EngineShim:
    def __init__(self, rig: dict, iks: dict):
        self.arm = {s: _ArmShim(rig, s, iks[s]) for s in SIDES}
        self.calib_status = None


class JogSession:
    """Manual joint/EE jogging through the real IK — testable without a TTY.

    `sides` limits which arms are PUSHED to the sink (single-arm bring-up); the
    IK + render shims always cover both so the dashboard scene stays complete."""

    def __init__(self, rig: dict, sink, sides=SIDES):
        self.rig = rig
        self.sink = sink
        self.sides = tuple(sides)
        self.ik = {s: ArmIK(rig, s) for s in SIDES}
        self.engine = _EngineShim(rig, self.ik)
        self.side = "right" if "right" in self.sides else self.sides[0]
        self.joint = 5                      # 0-based; j6 selected by default
        self.joint_step = np.radians(3.0)
        self.ee_step = 0.015                # m per nudge
        for s in self.sides:
            self.sink.set_arm(s, self.ik[s].q)

    def next_side(self) -> str:
        """Cycle the active side among the joggable ones (TAB)."""
        i = self.sides.index(self.side)
        self.side = self.sides[(i + 1) % len(self.sides)]
        return self.side

    # ---- actions ----------------------------------------------------------- #
    def step_joint(self, direction: int) -> np.ndarray:
        ik = self.ik[self.side]
        q = ik.q
        j = self.joint
        q[j] = float(np.clip(q[j] + direction * self.joint_step,
                             ik.soft_lo[j], ik.soft_hi[j]))
        ik.seed(q)
        self._push(self.side)
        return q

    def nudge_ee(self, d_world) -> np.ndarray:
        """Move the wrist target by a world-frame delta through the two-stage IK
        (exactly the solve path teleop uses)."""
        ik = self.ik[self.side]
        shim = self.engine.arm[self.side]
        d_base = shim.base_R.T @ np.asarray(d_world, dtype=float)
        target_p = ik.fk_wrist().translation() + d_base
        target = SE3.from_rotation_and_translation(ik.fk_ee().rotation(), target_p)
        for _ in range(6):
            ik.solve(target)
        shim.cmd_pos = target_p.copy()
        shim.cmd_R = ik.fk_ee().rotation().as_matrix()
        self._push(self.side)
        return ik.q

    def home(self) -> None:
        ik = self.ik[self.side]
        ik.reset()
        self.engine.arm[self.side].cmd_pos = None
        self.engine.arm[self.side].cmd_R = None
        self._push(self.side)

    def _push(self, side: str) -> None:
        self.sink.set_arm(side, self.ik[side].q)

    def publish(self, hz: float, t: float) -> None:
        if hasattr(self.sink, "publish"):
            self.sink.publish(self.engine, None, {s: False for s in SIDES}, hz, t)

    def status_line(self) -> str:
        q = np.degrees(self.ik[self.side].q)
        qs = " ".join(f"j{i+1}{'*' if i == self.joint else ''}={q[i]:+6.1f}" for i in range(6))
        return (f"[{self.side.upper()}] step={np.degrees(self.joint_step):.1f}°/"
                f"{self.ee_step*100:.1f}cm  {qs}")


def _make_sink(kind: str, rig: dict, sides=SIDES):
    if kind == "hw":
        # The jog never commands hands (no hand keys) — arms only, and only the
        # wired sides. Tee to the render stream so the dashboard mirrors the jog.
        from bimanual_teleop.hardware import HardwareSink, TeeSink
        hw = HardwareSink(rig, sides=sides, hands=False)
        try:
            from bimanual_teleop.render_sink import RenderSink
            return TeeSink(hw, RenderSink(rig))
        except Exception as e:
            print(f"[jog] render mirror disabled ({e}) — hardware jog unaffected")
            return hw
    from bimanual_teleop.render_sink import RenderSink
    return RenderSink(rig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sink", choices=["render", "hw"], default="render",
                    help="render = sim/Unity/dashboard preview; hw = REAL arms (Linux host)")
    ap.add_argument("--side", choices=["left", "right", "both"], default="both",
                    help="which arm(s) to drive — use a single side when only one is powered")
    args = ap.parse_args()

    sides = SIDES if args.side == "both" else (args.side,)
    rig = load_rig()
    sink = _make_sink(args.sink, rig, sides)
    jog = JogSession(rig, sink, sides)
    print(__doc__.split("Keys:")[1])
    print(f"sink={args.sink}  sides={','.join(sides)}  |  watch on the dashboard: "
          f"uv run python scripts/dashboard.py")
    print(jog.status_line(), flush=True)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    t0 = time.monotonic()
    try:
        while True:
            t = time.monotonic() - t0
            jog.publish(60.0, t)
            if not select.select([sys.stdin], [], [], 1 / 60)[0]:
                continue
            ch = sys.stdin.read(1)
            if ch in ("q", "\x1b"):
                break
            elif ch == "\t":
                jog.next_side()
            elif ch in "123456":
                jog.joint = int(ch) - 1
            elif ch == "=":
                jog.step_joint(+1)
            elif ch == "-":
                jog.step_joint(-1)
            elif ch == "[":
                jog.joint_step = max(np.radians(0.5), jog.joint_step / 2)
                jog.ee_step = max(0.002, jog.ee_step / 2)
            elif ch == "]":
                jog.joint_step = min(np.radians(12.0), jog.joint_step * 2)
                jog.ee_step = min(0.06, jog.ee_step * 2)
            elif ch == "w":
                jog.nudge_ee([-jog.ee_step, 0, 0])      # forward = world −X
            elif ch == "s":
                jog.nudge_ee([+jog.ee_step, 0, 0])
            elif ch == "a":
                jog.nudge_ee([0, -jog.ee_step, 0])      # left = world −Y
            elif ch == "d":
                jog.nudge_ee([0, +jog.ee_step, 0])
            elif ch == "r":
                jog.nudge_ee([0, 0, +jog.ee_step])
            elif ch == "f":
                jog.nudge_ee([0, 0, -jog.ee_step])
            elif ch == "h":
                jog.home()
            elif ch == "p":
                pass                                     # status prints below anyway
            else:
                continue
            print("\r" + jog.status_line() + " " * 8, end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()
        if hasattr(sink, "close"):
            sink.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
