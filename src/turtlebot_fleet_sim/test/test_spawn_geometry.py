"""Canonical example spawns are clear in both supported worlds."""
from math import cos, hypot, sin
from pathlib import Path

from turtlebot_fleet_sim.fleet_config import load_fleet_config


ROOT = Path(__file__).parents[1]
ROBOT_RADIUS = 0.18


def circle_clear_of_box(point, center, size, yaw=0.0):
    dx, dy = point[0] - center[0], point[1] - center[1]
    local_x = cos(yaw) * dx + sin(yaw) * dy
    local_y = -sin(yaw) * dx + cos(yaw) * dy
    gap_x = max(abs(local_x) - size[0] / 2.0, 0.0)
    gap_y = max(abs(local_y) - size[1] / 2.0, 0.0)
    return hypot(gap_x, gap_y) > ROBOT_RADIUS


def test_example_spawns_clear_official_and_custom_obstacles():
    robots = load_fleet_config(ROOT / 'config/fleet.example.yaml').robots
    points = [(robot.spawn.x, robot.spawn.y) for robot in robots]
    # Official turtlebot3_world ROS-symbol cylinders plus its four larger mesh
    # obstacles, conservatively bounded as 0.8 m discs.
    official = [((x, y), 0.15) for x in (-1.1, 0.0, 1.1) for y in (-1.1, 0.0, 1.1)]
    official += [((3.5, 0.0), 0.8), *((p, 0.55) for p in ((1.8, 2.7), (1.8, -2.7), (-1.8, 2.7), (-1.8, -2.7)))]
    custom = [((-2.0, 1.8), (0.2, 4.8), 0.0),
              ((2.5, -1.5), (3.2, 0.2), 0.45), ((3.8, 3.1), (0.8, 0.8), 0.0)]
    for point in points:
        assert all(hypot(point[0] - center[0], point[1] - center[1]) > radius + ROBOT_RADIUS
                   for center, radius in official)
        assert all(circle_clear_of_box(point, center, size, yaw)
                   for center, size, yaw in custom)
    assert hypot(points[0][0] - points[1][0], points[0][1] - points[1][1]) > 2 * ROBOT_RADIUS
