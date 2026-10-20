"""Geometry and route for the supplied practice world (metres / radians)."""
import math

COLORS = {"1": "red", "2": "yellow", "3": "black", "4": "white"}


def parse_order(value):
    parts = value.replace(",", " ").split()
    if len(parts) == 1 and len(parts[0]) == 4:
        parts = list(parts[0])
    result = [COLORS.get(p, p) for p in parts]
    if len(result) != 4 or set(result) != set(COLORS.values()):
        raise ValueError("投放顺序须包含红黄黑白各一次，例如 1234 或 red,yellow,black,white")
    return result


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def body_velocity(vx, vy, heading):
    c, s = math.cos(heading), math.sin(heading)
    return c*vx + s*vy, -s*vx + c*vy


def payload_offset(color, heading):
    # Plugin's four tray compartments, relative to base_footprint.
    x, y = {"black": (.04, .04), "white": (.04, -.04),
            "red": (-.04, .04), "yellow": (-.04, -.04)}[color]
    c, s = math.cos(heading), math.sin(heading)
    return c*x - s*y, s*x + c*y


def make_route(config, order):
    steps = []

    def move(name, x, y, z, heading, speed=None):
        steps.append(dict(kind="move", name=name, target=[x, y, z, heading],
                          speed=speed or config["cruise_speed"]))

    home_x, home_y = config["home"]
    w1, w2 = config["windows"]
    z = config["crossing_z"]
    east, west = config["east_lane_x"], config["west_lane_x"]
    move("takeoff", home_x, home_y, z, math.pi)
    move("window_1_approach", east, w1[1], z, math.pi)
    move("window_1_observe", east, w1[1], config["observe_z"], math.pi)
    steps.append(dict(kind="detect", name="window_1"))
    move("window_1_align", east, w1[1], z, math.pi)
    move("window_1_cross", west, w1[1], z, math.pi, config["crossing_speed"])
    # Clear the window and reach throwing height before the delivery circuit.
    move("delivery_entry", .62, -.80, float(config.get("throw_z", 1.05)), math.pi)
    previous = (.62, -.80)
    throw_lead = float(config.get("throw_lead", .22))
    throw_z = float(config.get("throw_z", 1.05))
    drop4_throw_z = float(config.get("drop4_throw_z", .74))
    for i, ((x, y), color) in enumerate(zip(config["drop_points"], order), 1):
        heading = 0. if i >= 3 else math.pi
        dx, dy = payload_offset(color, heading)
        # Preplan a fly-through release point, accounting for tray position
        # and horizontal travel of the thrown payload.
        travel_x, travel_y = x - dx - previous[0], y - dy - previous[1]
        length = math.hypot(travel_x, travel_y) or 1.
        height = drop4_throw_z if i == 4 else throw_z
        # Lower throws spend less time airborne and need less lead.
        lead = throw_lead * math.sqrt((height-.145)/(throw_z-.145))
        lead_x, lead_y = lead * travel_x / length, lead * travel_y / length
        release_x, release_y = x - dx - lead_x, y - dy - lead_y
        move(f"drop_{i}_approach", release_x, release_y, height, heading, config.get("delivery_speed", .25))
        steps[-1]["flyby"] = True
        steps[-1]["tangent"] = [travel_x/length, travel_y/length]
        steps.append(dict(kind="drop", name=f"drop_{i}", color=color, point=[x, y],
                          detect="window_2" if i == 3 else None))
        # Keep the next release calculation tied to the planned tangent,
        # rather than to a fixed elapsed time. This stays valid when cruise
        # speed is raised substantially.
        previous = (release_x + .20*travel_x/length,
                    release_y + .20*travel_y/length)
    # By the third target the vehicle has already turned east.  The direct
    # return leg crosses the second window, so there is no extra observe,
    # alignment, or pose correction after the fourth throw.
    move("return_home", home_x, home_y, height, 0.)
    move("landing", home_x, home_y, config["land_z"], 0., .24)
    return steps
