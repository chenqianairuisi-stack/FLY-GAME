"""The flight limit applies to the 3D norm and all command magnitudes."""
import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist

from ican_delivery_examples.full_mission import DeliveryMission


@pytest.mark.parametrize("velocity", [
    (0., 0., 0.), (.1, -.1, 0.), (.25, 0., 0.),
    (0., 0., .65), (0., 0., -.65), (.25, .25, .65), (5., -5., -.65),
])
def test_actual_3d_speed_never_exceeds_limit(velocity):
    sent = []
    state = SimpleNamespace(config={"max_speed": .5, "sim_velocity_gain": 2.},
                            cmd=SimpleNamespace(publish=sent.append))
    command = Twist()
    command.linear.x, command.linear.y, command.linear.z = velocity
    command.angular.z = 1.8
    DeliveryMission.publish_velocity(state, command)
    assert len(sent) == 1
    actual = (command.linear.x*2, command.linear.y*2, command.linear.z*2)
    assert math.hypot(*actual) <= .5+1e-12
    assert command.angular.z == 1.8
    speed = math.hypot(*velocity)
    factor = min(1., .25/speed) if speed else 1.
    assert actual == pytest.approx(tuple(v*factor*2 for v in velocity))
