"""Guards the borrowed-geometry contract with vla_bridge.

`envs/route.py` imports path-following geometry from the vla_bridge ROS
package rather than copying it, so training and the robot follow routes by
the same code. That coupling is only safe while those modules stay free of
ROS imports, which is what these tests check. If one fails, either the import
path moved or ROS leaked in, and `envs/route.py` is the single place to fix.
"""

from __future__ import annotations

import ast

import pytest
from envs import route


def test_the_borrowed_module_is_where_we_think() -> None:
    assert route.VLA_BRIDGE.is_dir()
    assert (route.VLA_BRIDGE / "vla_bridge" / "subgoal.py").is_file()
    assert (route.VLA_BRIDGE / "vla_bridge" / "goal_resolution.py").is_file()


@pytest.mark.parametrize("module", ["subgoal.py", "goal_resolution.py"])
def test_borrowed_modules_import_no_ros(module: str) -> None:
    """The reason this reuse is possible at all.

    Parsed rather than imported, so the test reports the offending module
    instead of dying on an ImportError inside the training venv.
    """
    source = (route.VLA_BRIDGE / "vla_bridge" / module).read_text()
    imported: set[str] = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    forbidden = {"rclpy", "geometry_msgs", "nav_msgs", "std_msgs", "tf2_ros"}
    assert not (imported & forbidden), (
        f"{module} now imports ROS: {imported & forbidden}"
    )


def test_the_geometry_we_depend_on_is_present() -> None:
    for name in (
        "Pose2D",
        "Projection",
        "carrot",
        "heading_error",
        "point_at_arc",
        "project",
        "remaining",
        "route_length",
    ):
        assert hasattr(route, name), f"vla_bridge no longer exports {name}"


def test_project_and_remaining_agree_on_a_straight_route() -> None:
    line = [route.Pose2D(0.0, 0.0, 0.0), route.Pose2D(4.0, 0.0, 0.0)]
    assert route.route_length(line) == pytest.approx(4.0)
    assert route.project(line, 1.0, 0.0).arc == pytest.approx(1.0)
    assert route.remaining(line, 1.0, 0.0) == pytest.approx(3.0)


def test_cross_track_offset_is_perpendicular_distance() -> None:
    line = [route.Pose2D(0.0, 0.0, 0.0), route.Pose2D(4.0, 0.0, 0.0)]
    assert route.project(line, 2.0, 0.75).offset == pytest.approx(0.75)


def test_carrot_past_the_end_degrades_to_the_goal() -> None:
    line = [route.Pose2D(0.0, 0.0, 0.0), route.Pose2D(2.0, 0.0, 0.0)]
    far = route.point_at_arc(line, 99.0)
    assert (far.x, far.y) == pytest.approx((2.0, 0.0))
