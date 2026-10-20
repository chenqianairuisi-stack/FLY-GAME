"""Nonblocking delivery and recognition must survive waypoint transitions."""
from types import SimpleNamespace

from ican_delivery_examples.full_mission import DeliveryMission


def test_throw_continues_motion_without_confirmation():
    sent, failures = [], []
    state = SimpleNamespace(carried={"red"}, now=lambda: 10.,
        pose=SimpleNamespace(position=SimpleNamespace(x=1., y=2., z=1.05)),
        session="test", index=8, route=[{"tangent": [1., 0.], "speed": .45}]*8, pending_throws={}, measured_velocity=[.9, 0., 0.],
        config={"cruise_speed": .45}, drop=SimpleNamespace(publish=sent.append),
        event=lambda *a, **kw: None, finish=lambda *a: failures.append(a))
    DeliveryMission.trigger_throw(state, dict(color="red", name="drop_1", point=[1.4, 2.]))
    assert not failures
    assert len(sent) == 1
    assert state.departure_velocity == (.45, 0.)
    assert not state.pending_throws["test_8_throw"]["recorded"]
    # No hold, sleep, confirmation, or detector is available on this stub.


def test_recognition_and_pending_throw_survive_advance():
    request = {"window": "window_2", "token": "test"}
    pending = {"shot": {"recorded": False}}
    state = SimpleNamespace(route=[{"name": "drop_3"}, {"name": "drop_4_approach"}],
        index=0, now=lambda: 10., event=lambda *a, **kw: None,
        request=request, detection={"label": "bird"}, pending_throws=pending)
    DeliveryMission.advance(state)
    assert state.index == 1
    assert state.request is request
    assert state.detection == {"label": "bird"}
    assert state.pending_throws is pending


def test_async_confirmation_does_not_repeat_release():
    sent = []
    token = "test_8_throw"
    pending = dict(point="drop_1", color="red", sim_time=10., aircraft=[1., 2., 1.05],
        target=[1.4, 2.], trigger_velocity=[.9, 0., 0.], command="unused",
        last_sent=10., attempts=1, recorded=False, motion_saved=False)
    state = SimpleNamespace(now=lambda: 10.4, pending_throws={token: pending},
        throw_results={token: {"status": "released"}}, dropped={"red"},
        report={"drops": [], "throws": {}}, drop=SimpleNamespace(publish=sent.append),
        event=lambda *a, **kw: None)
    DeliveryMission.monitor_throws(state)
    DeliveryMission.monitor_throws(state)
    assert len(state.report["drops"]) == 1
    assert not sent
