"""Run the teleop pipeline against REAL hardware (Linux control host).

This is the hardware bring-up entrypoint. It reuses the exact TeleopEngine +
controllers + supervisor as the render path — only the *sink* changes (HardwareSink →
YAM CAN + ORCA serial). Single-process bring-up form; for production split each
arm into its own ~250 Hz CAN process (see README "Hardware day").

Prereqs (Ubuntu): SocketCAN up (`sudo ip link set can0 up type can bitrate 1000000`,
same for can1), i2rt SDK installed, ORCA hands tensioned + calibrated.

    python -m bimanual_teleop.launch.run_hw --vr orbit
    python -m bimanual_teleop.launch.run_hw --vr orbit --record recordings/hw_session.npz
    python -m bimanual_teleop.launch.run_hw --vr replay recordings/hw_session.npz --clutch recorded

SAFETY: starts in IDLE (not following). Engage via the configured clutch. Ctrl+C
or e-stop releases torque on all devices.
"""
from __future__ import annotations

import argparse
import signal
import sys
import time

from ..config import load_rig
from ..engine import TeleopEngine
from ..safety.clutch import GestureClutch, RecordedClutch
from ..safety.supervisor import Supervisor
from ..vr.ingest import make_source
from ..vr.replay import SessionRecorder


class _TeeSink:
    """Forward engine commands to the hardware AND the render stream, so the
    dashboard shows the live session while the metal moves. Hardware first —
    the render copy is best-effort cosmetics."""

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

    def close(self):
        self.hw.close()
        try:
            self.render.close()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--vr", choices=["vuer", "orbit", "fake", "replay"], default="orbit")
    ap.add_argument("replay_path", nargs="?", help="session .npz when --vr replay")
    ap.add_argument("--clutch", choices=["gesture", "recorded"], default="gesture",
                    help="hardware engage policy (default: gesture; recorded is for replay sessions)")
    ap.add_argument("--record", metavar="PATH", default=None,
                    help="write VR frames + engage state to a replayable .npz session")
    ap.add_argument("--hz", type=float, default=None, help="override control rate")
    ap.add_argument("--no-render", action="store_true",
                    help="skip the render.state stream (dashboard stays blind)")
    args = ap.parse_args()

    if sys.platform == "darwin":
        print("WARNING: real YAM control needs Linux/SocketCAN; macOS can't run the CAN loop.")

    rig = load_rig()
    rig["vr"]["transport"] = args.vr
    if args.vr == "replay":
        if not args.replay_path:
            ap.error("--vr replay needs a session file: run_hw --vr replay session.npz")
        rig["vr"]["replay_path"] = args.replay_path
    hz = args.hz or rig["control"]["arm_hz"]
    # Hardware speed derating: scale the IK joint-velocity budget down for real
    # motors (the sink's JointCommandShaper independently caps speed again).
    scale = float(rig.get("hardware", {}).get("max_vel_scale", 0.35))
    rig["ik"]["max_vel"] = float(rig["ik"]["max_vel"]) * scale
    print(f"[hw] ik.max_vel derated x{scale:.2f} -> {rig['ik']['max_vel']:.1f} rad/s; "
          f"shaper rate_limit {rig.get('hardware', {}).get('rate_limit', 1.2)} rad/s")

    src = make_source(rig)
    clutch = RecordedClutch(src) if args.clutch == "recorded" else GestureClutch()

    from ..hardware import HardwareSink
    sink = HardwareSink(rig)
    render = None
    if not args.no_render:
        try:
            from ..render_sink import RenderSink
            render = RenderSink(rig)
            sink = _TeeSink(sink, render)
            print(f"[hw] render.state mirrored for the dashboard "
                  f"({rig['vr'].get('unity_json_endpoint', 'tcp://127.0.0.1:8102')})")
        except Exception as e:
            print(f"[hw] render mirror disabled ({e}) — hardware loop unaffected")
    engine = TeleopEngine(rig, sink)
    supervisor = Supervisor(rig, clutch)
    src.start()
    recorder = SessionRecorder() if args.record else None
    push_calib = hasattr(src, "set_calib")   # in-headset calibration countdown (Vuer)

    # Same rationale as run_teleop: children of background/non-interactive shells
    # inherit SIGINT=SIG_IGN and never see KeyboardInterrupt — on hardware that
    # means NO torque release on stop. Install handlers unconditionally; TERM
    # takes the same e-stop path.
    def _request_stop(signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    period = 1.0 / hz
    rate_hz = 0.0
    try:
        while True:
            t = time.monotonic()   # shared clock with source stamps + supervisor staleness
            frame = src.latest()
            engaged = supervisor.update(frame, t)
            if recorder is not None and frame is not None:
                recorder.add(frame, engaged, t)
            engine.tick(frame, engaged, t)
            if render is not None:
                rate_hz = 0.9 * rate_hz + 0.1 / max(period, 1e-6)
                try:
                    render.publish(engine, frame, engaged, rate_hz, t)
                except Exception:
                    pass
            if push_calib:
                src.set_calib(engine.calib_status)
            dt = period - (time.monotonic() - t)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        print("\nstopping — releasing torque")
    finally:
        # Mask further signals from the FIRST teardown line: nothing may abort
        # the torque release or the recording save.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        supervisor.estop()
        src.stop()
        sink.close()
        if recorder is not None:
            recorder.save(args.record)
            print(f"recorded {len(recorder)} frames -> {args.record}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
