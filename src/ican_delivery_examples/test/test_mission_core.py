import math
from itertools import permutations
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from ican_delivery_examples.mission_core import angle, body_velocity, make_route, parse_order, payload_offset


ROOT = Path(__file__).resolve().parents[3]


def test_order_is_a_permutation():
    assert parse_order("3142") == ["black", "red", "white", "yellow"]
    assert parse_order("red, yellow, black, white") == parse_order("1234")
    for invalid in ("1111", "123", "12345", "red,red,black,white", "5678"):
        with pytest.raises(ValueError):
            parse_order(invalid)


def test_velocity_and_payload_coordinates():
    assert body_velocity(1., 0., math.pi/2) == pytest.approx((0., -1.))
    assert angle(-2*math.pi+.1) == pytest.approx(.1)
    assert payload_offset("black", math.pi) == pytest.approx((-.04, -.04))


@pytest.mark.parametrize("order", ["".join(p) for p in permutations("1234")])
def test_route_crosses_openings_and_finishes_at_home(order):
    cfg = yaml.safe_load((ROOT / "src/ican_delivery_examples/config/full_demo.yaml").read_text())["delivery_mission"]["ros__parameters"]
    for key in ("windows", "drop_points"):
        cfg[key] = list(zip(cfg[key][::2], cfg[key][1::2]))
    route = make_route(cfg, parse_order(order))
    world = ET.parse(ROOT / "binary_install/share/ican_delivery_sim/worlds/delivery_task2.world")
    windows = [world.find(f".//model[@name='window_{i}']") for i in (1, 2)]
    previous = [*cfg["home"], .2, math.pi]
    crossed = []
    for step in route:
        if step["kind"] != "move":
            continue
        target = step["target"]
        for i, window in enumerate(windows, 1):
            wx, wy, *_ = map(float, window.findtext("pose").split())
            if (previous[0]-wx)*(target[0]-wx) < 0:
                ratio = (wx-previous[0])/(target[0]-previous[0])
                y, z = [previous[j]+ratio*(target[j]-previous[j]) for j in (1, 2)]
                # A segment near this window must clear the full 0.7 m body.
                if abs(y-wy) < .85:
                    assert abs(y-wy)+.35 < .50
                    assert z-.20 > .40
                    assert z+.34 < 1.15
                    crossed.append(i)
        previous = target
    assert crossed == [1, 2]
    assert route[-1]["target"][:3] == pytest.approx([*cfg["home"], cfg["land_z"]])
    assert [s["color"] for s in route if s["kind"] == "drop"] == parse_order(order)
    third = next(s for s in route if s["name"] == "drop_3_approach")
    fourth = next(s for s in route if s["name"] == "drop_4_approach")
    assert third["target"][3] == fourth["target"][3] == 0.
    assert third["target"][2] > fourth["target"][2]
    assert next(s for s in route if s["name"] == "drop_3")["detect"] == "window_2"
    fourth_index = next(i for i,s in enumerate(route) if s["name"] == "drop_4")
    assert [s["name"] for s in route[fourth_index+1:]] == ["return_home", "landing"]

