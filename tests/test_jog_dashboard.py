"""Keyboard jog (JogSession) + dashboard (StateFeed/server) — headless tests."""
from __future__ import annotations

import http.client
import importlib.util
import json
import threading
import uuid
from pathlib import Path

import numpy as np
import pytest

from bimanual_teleop.config import load_rig
from bimanual_teleop.render_sink import RenderSink

REPO = Path(__file__).resolve().parents[1]


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, REPO / "scripts" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture()
def render_sink():
    rig = load_rig()
    rig["vr"]["unity_json_endpoint"] = None
    sink = RenderSink(rig, endpoint=f"inproc://jog-test-{uuid.uuid4()}")
    yield rig, sink
    sink.close()


def test_jog_joint_step_moves_only_selected_joint_within_limits(render_sink):
    rig, sink = render_sink
    jog = _load("jog_arms").JogSession(rig, sink)
    jog.side, jog.joint = "right", 5
    q0 = jog.ik["right"].q
    q1 = jog.step_joint(-1)
    assert abs((q1 - q0)[5] + jog.joint_step) < 1e-9
    assert np.allclose((q1 - q0)[:5], 0.0)
    for _ in range(500):                                   # hammer into the soft stop
        jog.step_joint(-1)
    q = jog.ik["right"].q
    assert q[5] >= jog.ik["right"].soft_lo[5] - 1e-9       # clamped, never past
    assert np.allclose(jog.ik["left"].q, jog.ik["left"].q0)  # other arm untouched


def test_jog_ee_nudge_moves_wrist_in_world_direction(render_sink):
    rig, sink = render_sink
    mod = _load("jog_arms")
    jog = mod.JogSession(rig, sink)
    jog.side = "right"
    shim = jog.engine.arm["right"]
    p0 = shim.base_R @ jog.ik["right"].fk_wrist().translation() + shim.base_pos
    for _ in range(12):
        jog.nudge_ee([-jog.ee_step, 0.0, 0.0])             # forward = world −X
    p1 = shim.base_R @ jog.ik["right"].fk_wrist().translation() + shim.base_pos
    d = p1 - p0
    assert d[0] < -0.05                                     # moved forward
    assert abs(d[0]) > max(abs(d[1]), abs(d[2]))            # dominantly forward
    jog.publish(60.0, 1.0)                                  # build_state works on the shim
    assert "right" in jog.status_line().lower() or "RIGHT" in jog.status_line()


class _Recorder:
    """Minimal sink: records which sides get commanded."""

    def __init__(self):
        self.calls = []

    def set_arm(self, side, q):
        self.calls.append(("arm", side))

    def set_hand(self, side, joints_deg):
        self.calls.append(("hand", side))


def test_jog_single_side_pushes_only_active_side():
    """Single-arm bring-up: --side right must never command the unpowered arm."""
    rig = load_rig()
    sink = _Recorder()
    jog = _load("jog_arms").JogSession(rig, sink, sides=("right",))
    assert jog.side == "right"
    assert jog.next_side() == "right"               # TAB cannot escape to a dead arm
    jog.step_joint(+1)
    jog.nudge_ee([0.0, 0.0, jog.ee_step])
    jog.home()
    assert sink.calls and {s for _, s in sink.calls} == {"right"}


def test_hardware_sink_skips_unwired_sides_and_tee_is_hardware_first():
    """HardwareSink drops commands for sides it was not built with (so the
    engine/jog can keep speaking both), and TeeSink commands hardware FIRST with
    render failures swallowed."""
    from bimanual_teleop.hardware import HardwareSink, TeeSink

    hw = HardwareSink(load_rig(), sides=(), hands=False)   # no devices: constructible anywhere
    hw.set_arm("left", np.zeros(6))                        # unwired side: silently dropped
    hw.set_hand("right", {"thumb_mcp": 0.0})
    hw.close()

    calls = []

    class _Hw:
        def set_arm(self, side, q):
            calls.append(("hw_arm", side))

        def set_hand(self, side, j):
            calls.append(("hw_hand", side))

        def close(self):
            calls.append(("hw_close", None))

    class _DeadRender:
        def set_arm(self, side, q):
            raise RuntimeError("render down")

        def set_hand(self, side, j):
            raise RuntimeError("render down")

        def close(self):
            raise RuntimeError("render down")

    tee = TeeSink(_Hw(), _DeadRender())
    tee.set_arm("right", np.zeros(6))           # render failure must not block hardware
    tee.set_hand("right", {})
    tee.publish(None, None, {}, 0.0, 0.0)       # no usable render publish -> no-op
    tee.close()
    assert calls == [("hw_arm", "right"), ("hw_hand", "right"), ("hw_close", None)]


def test_dashboard_serves_injected_state():
    mod = _load("dashboard")
    feed = mod.StateFeed("tcp://127.0.0.1:1")               # never started — injected state
    feed.latest = {"status": {"hz": 100.0, "tracked": {"left": True, "right": False},
                              "engaged": {"left": False, "right": False}, "calib": None},
                   "arms": {}, "op": {"hands": {}}}
    srv = mod.make_server(feed, "127.0.0.1", 0)
    port = srv.server_address[1]
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    try:
        conn = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
        conn.request("GET", "/state")
        d = json.loads(conn.getresponse().read())
        assert d["connected"] is False
        assert d["state"]["status"]["hz"] == 100.0
        conn.request("GET", "/")
        page = conn.getresponse().read().decode()
        assert "dashboard" in page and "torso" in page
    finally:
        srv.shutdown()
        srv.server_close()


def test_dashboard_serves_meshes_and_mesh_transforms():
    """/meshes returns per-side triangle soups and /state carries live per-geom
    world transforms computed from the streamed joint state."""
    mod = _load("dashboard")
    assets = mod.MeshAssets(max_tris_per_link=60)
    assert {"left", "right", "stand"} <= set(assets.geoms)  # + hand_* when the ORCA description exists
    assert len(assets.geoms["right"]) == 6                  # base + link1..5
    assert len(assets.geoms["stand"]) == 6                  # AgileX frame parts
    rig = load_rig()
    q = list(rig["arms"]["right"]["neutral_q"])
    T = assets.transforms({"right": {"q": q}, "left": {}})
    assert "right" in T and len(T["right"]) == 6 and len(T["right"][0]) == 16
    assert "left" not in T                                  # no q -> no transforms

    feed = mod.StateFeed("tcp://127.0.0.1:1")
    feed.latest = {"status": {"hz": 50.0, "tracked": {"left": False, "right": True},
                              "engaged": {"left": False, "right": False}, "calib": None},
                   "arms": {"right": {"q": q}}, "op": {"hands": {}}}
    feed.rx_time = 1.0
    srv = mod.make_server(feed, "127.0.0.1", 0, meshes=assets)
    port = srv.server_address[1]
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    try:
        conn = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
        conn.request("GET", "/meshes")
        m = json.loads(conn.getresponse().read())
        assert len(m["left"]) == 6 and len(m["left"][0]) % 9 == 0
        conn.request("GET", "/state")
        d = json.loads(conn.getresponse().read())
        assert len(d["mesh_T"]["right"]) == 6
    finally:
        srv.shutdown()
        srv.server_close()


def test_dashboard_articulated_hand_follows_joints():
    """The hand rendering (real ORCA model when the sibling description repo is
    present, parametric otherwise) articulates with the streamed joint angles and
    attaches at the streamed EE pose."""
    import numpy as np
    from bimanual_teleop.hands.joint_map import ORCA_JOINT_ORDER
    mod = _load("dashboard")
    assets = mod.MeshAssets(max_tris_per_link=60)
    arms = {"right": {"ee_pos": [-0.3, 0.1, 1.0], "ee_quat": [1.0, 0.0, 0.0, 0.0]}}
    open_q = [0.0] * 17
    fist_q = [80.0 if ("mcp" in n or "pip" in n) else 0.0 for n in ORCA_JOINT_ORDER]
    mk = lambda q: {"arms": arms, "hand_render": {"right": {"names": list(ORCA_JOINT_ORDER), "q": q}}}
    if assets.hand_mode == "real":
        T_open = np.asarray(assets.hand_transforms(mk(open_q))["right"])
        T_fist = np.asarray(assets.hand_transforms(mk(fist_q))["right"])
        assert T_open.shape == T_fist.shape and T_open.shape[1] == 16
        assert np.all(np.isfinite(T_open)) and np.all(np.isfinite(T_fist))
        assert np.abs(T_open - T_fist).max() > 0.005          # fingers actually moved
        assert "hand_right" in assets.geoms and len(assets.geoms["hand_right"]) > 10
    else:
        a = np.asarray(assets.hand_world(mk(open_q))["right"])
        b = np.asarray(assets.hand_world(mk(fist_q))["right"])
        assert a.shape == b.shape and np.all(np.isfinite(a)) and np.all(np.isfinite(b))
        assert np.linalg.norm(a - b) > 0.05
