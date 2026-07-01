import math
import subprocess
import time


def compute_undock_pose(dock_x, dock_y, dock_angle, distance_m=0.8):
    """
    Compute the pose 800 mm in front of the docking station.
    This pose is reused as the pre-dock goal.
    """

    undock_x = dock_x + distance_m * math.cos(dock_angle)
    undock_y = dock_y + distance_m * math.sin(dock_angle)

    return {
        "x": undock_x,
        "y": undock_y,
        "yaw": dock_angle,
    }


def localize(x, y, angle, map_name, localization_callback):
    """
    Ask the ROS layer to localize the robot on a map at a known pose.

    Returns True or False.
    """

    return bool(localization_callback(x, y, angle, map_name))


def go_to(x, y, map_name, navigation_callback):
    """
    Navigate from current pose to target x/y.

    Yaw is not forced.
    """

    return bool(navigation_callback(x, y, None, map_name))


def undock(dock_x, dock_y, dock_angle, map_name, navigation_callback):
    """
    Move from docked position to the undock/pre-dock pose.
    """

    undock_pose = compute_undock_pose(
        dock_x,
        dock_y,
        dock_angle,
        distance_m=0.8
    )

    return bool(navigation_callback(
        undock_pose["x"],
        undock_pose["y"],
        undock_pose["yaw"],
        map_name
    ))


def dock(dock_x, dock_y, dock_angle, map_name, navigation_callback, dock_callback):
    """
    Docking sequence:
    1. Go to pre-dock pose, 800 mm in front of dock.
    2. Force final dock alignment angle.
    3. Run reverse shim docking behavior.
    """

    pre_dock_pose = compute_undock_pose(
        dock_x,
        dock_y,
        dock_angle,
        distance_m=0.8
    )

    reached_pre_dock = navigation_callback(
        pre_dock_pose["x"],
        pre_dock_pose["y"],
        pre_dock_pose["yaw"],
        map_name
    )

    if not reached_pre_dock:
        return False

    return bool(dock_callback(dock_x, dock_y, dock_angle, map_name))