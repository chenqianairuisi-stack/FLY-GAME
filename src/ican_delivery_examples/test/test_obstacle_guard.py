from types import SimpleNamespace

import numpy as np

from ican_delivery_examples.full_mission import DeliveryMission


def guard(points, vx=.16, vy=0.):
    state = SimpleNamespace(config=dict(lidar_enabled=True, obstacle_margin=.025),
                            points=np.array(points, dtype=float))
    return DeliveryMission.obstacle(state, vx, vy)


def test_window_opening_clears_full_aircraft_height():
    # In base coordinates at crossing height .74: upper frame starts at
    # +.41, lower frame ends at -.34, side frame starts +/- .50.
    assert not guard([[.5, 0., .42], [.5, .1, .42], [.5, 0., -.35],
                      [.5, .1, -.35], [.5, .51, 0.], [.5, -.51, 0.]])


def test_body_level_obstacle_stops_motion():
    assert guard([[.48, 0., 0.], [.49, .1, .1]])
    assert not guard([[-.48, 0., 0.], [-.49, .1, .1]])
    assert guard([[0., .48, 0.], [.1, .49, .1]], vx=0., vy=.16)
    assert not guard([[.48, 0., 0.], [.49, .1, .1]], vx=0., vy=0.)
