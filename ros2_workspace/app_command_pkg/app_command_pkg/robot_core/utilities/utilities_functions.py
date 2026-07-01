import json
import time
from datetime import datetime


class RobotUtilityState:
    def __init__(self):
        self.recording_active = False
        self.recorded_measures = []


def start_record_measures(state):
    """
    Enable contamination flag/pose recording.
    """

    state.recording_active = True
    state.recorded_measures = []

    return {
        "success": True,
        "reason": "recording_started",
        "timestamp": datetime.now().isoformat(),
    }


def stop_record_measures(state):
    """
    Disable contamination flag/pose recording.
    """

    state.recording_active = False

    return {
        "success": True,
        "reason": "recording_stopped",
        "timestamp": datetime.now().isoformat(),
        "record_count": len(state.recorded_measures),
    }


def get_current_pose(get_pose_callback):
    """
    Return current robot pose using a callback from the ROS layer.

    Expected callback return:
        {
            "x": float,
            "y": float,
            "yaw": float
        }
    """

    pose = get_pose_callback()

    if pose is None:
        return None

    required_keys = ["x", "y", "yaw"]

    for key in required_keys:
        if key not in pose:
            return None

    if pose["x"] is None or pose["y"] is None or pose["yaw"] is None:
        return None

    return {
        "x": float(pose["x"]),
        "y": float(pose["y"]),
        "yaw": float(pose["yaw"]),
        "timestamp": datetime.now().isoformat(),
    }


def associate_flag_pose(state, flag_result, pose):
    """
    Associate a contamination flag result with a robot pose.

    Only records if recording is active.
    """

    if not state.recording_active:
        return {
            "success": False,
            "reason": "recording_not_active",
        }

    if flag_result is None:
        return {
            "success": False,
            "reason": "missing_flag_result",
        }

    if pose is None:
        return {
            "success": False,
            "reason": "missing_pose",
        }

    record = {
        "timestamp": datetime.now().isoformat(),
        "pose": pose,
        "flag_result": flag_result,
    }

    state.recorded_measures.append(record)

    return {
        "success": True,
        "reason": "flag_pose_recorded",
        "record_count": len(state.recorded_measures),
        "record": record,
    }