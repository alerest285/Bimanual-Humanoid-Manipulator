"""HardwareSink: the real-robot backend behind the same set_arm/set_hand interface
as RenderSink, so TeleopEngine drives visualization or hardware unchanged.

Every arm command passes through a per-side JointCommandShaper before touching
CAN: clamped to the physical joint limits, per-joint speed-capped
(`hardware.rate_limit`), and smoothed by a critically-damped tracker
(`hardware.smooth_hz`) feeding the YAM's motor-side MIT PD. The shaper
initializes from the arm's MEASURED pose, so the first command glides from
wherever the robot actually is — no startup snap.

NOTE: this is the synchronous, single-process bring-up sink. For production the
arms want a dedicated ~250 Hz CAN loop per side (separate process / SCHED_FIFO),
decoupled from vision/IK via latest-value buffers — see README "Hardware day" and
the recon architecture. This class is the correct *logic*; wrap each arm in its
own process when you need the rate.

Partial rigs: pass `sides=("right",)` / `hands=False` to drive only the devices
that are actually powered (single-arm bring-up). Commands addressed to an
unconfigured side are silently dropped — the engine and jog always speak both
sides; the sink owns the knowledge of what is wired.
"""
from __future__ import annotations

import time

import numpy as np

from .config import SIDES
from .logging_utils import get_logger
from .safety.shaper import JointCommandShaper

log = get_logger("hardware")


def arm_shaper(rig: dict, q0) -> JointCommandShaper:
    """The hardware-boundary shaper for one YAM arm, from rig config. Factored out
    so the safety wiring is unit-testable without the i2rt SDK."""
    hw = rig.get("hardware", {})
    limits = rig["arms"]["joint_limits"]
    return JointCommandShaper(
        q0,
        rate_limit=float(hw.get("rate_limit", 1.2)),
        smooth_hz=float(hw.get("smooth_hz", 3.0)),
        lo=limits["lower"],
        hi=limits["upper"],
    )


class HardwareSink:
    def __init__(self, rig: dict, sides=SIDES, hands: bool = True):
        from .arms.yam_driver import YamArm
        self.sides = tuple(sides)
        self.arms = {s: YamArm(rig["arms"][s]["can_channel"]) for s in self.sides}
        self.hands = {}
        if hands:
            from .hands.real_driver import RealHand
            self.hands = {s: RealHand(model_name=rig["hands"][s]["model_name"]) for s in self.sides}
        self.shapers = {}
        for s in self.sides:
            try:
                q0 = self.arms[s].state()                  # glide from the MEASURED pose
            except Exception as e:
                q0 = np.asarray(rig["arms"][s]["neutral_q"], dtype=float)
                log.warning("%s arm: could not read measured pose (%s); shaper starts at rig neutral", s, e)
            self.shapers[s] = arm_shaper(rig, q0)

    def set_arm(self, side: str, q: np.ndarray) -> None:
        if side not in self.arms:
            return                                         # side not wired on this rig
        self.arms[side].command(self.shapers[side].shape(q, time.monotonic()))

    def set_hand(self, side: str, joints_deg: dict) -> None:
        if side not in self.hands:
            return
        self.hands[side].set_joint_positions(joints_deg)

    def close(self) -> None:
        for h in self.hands.values():
            try:
                h.release()
            except Exception:
                pass
        for a in self.arms.values():
            try:
                a.close()
            except Exception:
                pass


class TeeSink:
    """Forward commands to the hardware AND the render stream, so the dashboard
    shows the live session while the metal moves. Hardware first — the render
    copy is best-effort cosmetics."""

    def __init__(self, hw, render):
        self.hw = hw
        self.render = render

    def set_arm(self, side, q):
        self.hw.set_arm(side, q)
        try:
            self.render.set_arm(side, q)
        except Exception:
            pass

    def set_hand(self, side, joints_deg):
        self.hw.set_hand(side, joints_deg)
        try:
            self.render.set_hand(side, joints_deg)
        except Exception:
            pass

    def publish(self, *args, **kwargs):
        if hasattr(self.render, "publish"):
            try:
                self.render.publish(*args, **kwargs)
            except Exception:
                pass

    def close(self):
        self.hw.close()
        try:
            self.render.close()
        except Exception:
            pass
