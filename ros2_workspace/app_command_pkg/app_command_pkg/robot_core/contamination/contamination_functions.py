import time
from statistics import mean

from .defaults import DEFAULT_CONTAMINATION_PARAMETERS


class ContaminationState:
    def __init__(self):
        self.parameters = DEFAULT_CONTAMINATION_PARAMETERS.copy()

        self.recording_active = False
        self.recorded_measures = []

        self.measurement_active = False
        self.measurement_continuous = False
        self.current_frame = []

        self.last_calibration_result = None
        self.last_measurement_result = None
        self.last_flag_result = None

def collect_measurement_frame(state, get_sample_callback, duration_s, sample_period_s=0.05):
    """
    Collect valid probe samples during a fixed duration.

    Returns
    -------
    list
        List of valid samples.
    """

    duration_s = float(duration_s)

    if duration_s <= 0:
        return []

    samples = []
    start_time = time.time()

    while time.time() - start_time < duration_s:
        sample = get_sample_callback()

        if validate_measurement(sample, state.parameters):
            samples.append(sample)

        time.sleep(sample_period_s)

    return samples

def probe_calibration(state, get_sample_callback, duration_s=None):
    """
    Measure background noise during a given duration and update contamination
    background parameters.

    Parameters
    ----------
    state : ContaminationState
        Current contamination state.

    get_sample_callback : callable
        Function returning the latest probe sample.
        Expected format:
            {
                "alpha_cps": float,
                "beta_gamma_cps": float
            }

    duration_s : float | None
        Calibration duration. If None, uses robot default parameter.

    Returns
    -------
    dict
        Calibration result.
    """

    if duration_s is None:
        duration_s = state.parameters.get("calibration_time_s", 10.0)

    duration_s = float(duration_s)

    if duration_s <= 0:
        return {
            "success": False,
            "reason": "invalid_duration",
            "duration_s": duration_s,
            "sample_count": 0,
        }

    samples = collect_measurement_frame(
        state,
        get_sample_callback,
        duration_s
    )

    if len(samples) == 0:
        result = {
            "success": False,
            "reason": "no_valid_samples",
            "duration_s": duration_s,
            "sample_count": 0,
        }
        state.last_calibration_result = result
        return result

    alpha_background = mean(sample["alpha_cps"] for sample in samples)
    beta_gamma_background = mean(sample["beta_gamma_cps"] for sample in samples)

    state.parameters["alpha_background_cps"] = alpha_background
    state.parameters["beta_gamma_background_cps"] = beta_gamma_background

    result = {
        "success": True,
        "reason": "calibration_complete",
        "duration_s": duration_s,
        "sample_count": len(samples),
        "alpha_background_cps": alpha_background,
        "beta_gamma_background_cps": beta_gamma_background,
    }

    state.last_calibration_result = result
    return result

def filter_flag(mean_measurement, contamination_params):
    """
    Assign a contamination flag to a mean measurement.

    Expected mean_measurement format:
        {
            "alpha_mean_cps": float,
            "beta_gamma_mean_cps": float
        }

    Returns
    -------
    dict
        Flag result.
    """

    if mean_measurement is None:
        return {
            "flag": "INVALID",
            "reason": "missing_mean_measurement",
        }

    required_keys = ["alpha_mean_cps", "beta_gamma_mean_cps"]

    for key in required_keys:
        if key not in mean_measurement:
            return {
                "flag": "INVALID",
                "reason": f"missing_{key}",
            }

    alpha_mean_cps = mean_measurement["alpha_mean_cps"]
    beta_gamma_mean_cps = mean_measurement["beta_gamma_mean_cps"]

    if alpha_mean_cps is None or beta_gamma_mean_cps is None:
        return {
            "flag": "INVALID",
            "reason": "none_mean_value",
        }

    epsilon = contamination_params.get("epsilon", 0.35)
    detector_surface_cm2 = contamination_params.get("detector_surface_cm2", 100.0)

    if epsilon <= 0 or detector_surface_cm2 <= 0:
        return {
            "flag": "INVALID",
            "reason": "invalid_detector_parameters",
            "epsilon": epsilon,
            "detector_surface_cm2": detector_surface_cm2,
        }

    alpha_background_cps = contamination_params.get("alpha_background_cps", 0.0)
    beta_gamma_background_cps = contamination_params.get("beta_gamma_background_cps", 0.0)

    alpha_net_cps = max(0.0, alpha_mean_cps - alpha_background_cps)
    beta_gamma_net_cps = max(0.0, beta_gamma_mean_cps - beta_gamma_background_cps)

    alpha_bq_cm2 = alpha_net_cps / (epsilon * detector_surface_cm2)
    beta_gamma_bq_cm2 = beta_gamma_net_cps / (epsilon * detector_surface_cm2)

    alpha_warning = contamination_params.get("alpha_warning_bq_cm2", 0.4)
    alpha_danger = contamination_params.get("alpha_danger_bq_cm2", 4.0)

    beta_gamma_warning = contamination_params.get("beta_gamma_warning_bq_cm2", 0.4)
    beta_gamma_danger = contamination_params.get("beta_gamma_danger_bq_cm2", 4.0)

    alpha_flag = _single_channel_flag(
        alpha_bq_cm2,
        alpha_warning,
        alpha_danger
    )

    beta_gamma_flag = _single_channel_flag(
        beta_gamma_bq_cm2,
        beta_gamma_warning,
        beta_gamma_danger
    )

    final_flag = _worst_flag(alpha_flag, beta_gamma_flag)

    if alpha_bq_cm2 >= beta_gamma_bq_cm2:
        dominant_channel = "alpha"
        dominant_activity_bq_cm2 = alpha_bq_cm2
    else:
        dominant_channel = "beta_gamma"
        dominant_activity_bq_cm2 = beta_gamma_bq_cm2

    result = {
        "flag": final_flag,
        "alpha_flag": alpha_flag,
        "beta_gamma_flag": beta_gamma_flag,

        "alpha_mean_cps": alpha_mean_cps,
        "beta_gamma_mean_cps": beta_gamma_mean_cps,

        "alpha_background_cps": alpha_background_cps,
        "beta_gamma_background_cps": beta_gamma_background_cps,

        "alpha_net_cps": alpha_net_cps,
        "beta_gamma_net_cps": beta_gamma_net_cps,

        "alpha_bq_cm2": alpha_bq_cm2,
        "beta_gamma_bq_cm2": beta_gamma_bq_cm2,

        "dominant_channel": dominant_channel,
        "dominant_activity_bq_cm2": dominant_activity_bq_cm2,

        "reason": "flag_computed",
    }

    return result

def _single_channel_flag(activity_bq_cm2, warning_threshold, danger_threshold):
    """
    Return RAS, WARNING, or DANGER for one channel.
    """

    if danger_threshold <= warning_threshold:
        return "INVALID"

    if activity_bq_cm2 >= danger_threshold:
        return "DANGER"

    if activity_bq_cm2 >= warning_threshold:
        return "WARNING"

    return "RAS"

def _worst_flag(flag_a, flag_b):
    """
    Return the most severe flag.
    """

    severity = {
        "RAS": 0,
        "WARNING": 1,
        "DANGER": 2,
        "INVALID": 3,
    }

    if severity.get(flag_a, 3) >= severity.get(flag_b, 3):
        return flag_a

    return flag_b

def validate_measurement(sample, parameters):
    """
    Validate one probe sample before using it.
    """

    if sample is None:
        return False

    if "alpha_cps" not in sample or "beta_gamma_cps" not in sample:
        return False

    alpha = sample["alpha_cps"]
    beta_gamma = sample["beta_gamma_cps"]

    if alpha is None or beta_gamma is None:
        return False

    min_valid = parameters.get("min_valid_cps", 0.0)
    max_valid = parameters.get("max_valid_cps", 100000.0)

    if not min_valid <= alpha <= max_valid:
        return False

    if not min_valid <= beta_gamma <= max_valid:
        return False

    return True

def load_contamination_parameters(state, params=None):
    """
    Load contamination parameters into state.

    If params is None, reload robot defaults.
    If params is provided, defaults are loaded first, then params override them.
    """

    state.parameters = DEFAULT_CONTAMINATION_PARAMETERS.copy()

    if params is not None:
        update_contamination_parameters(state, params)

    return state.parameters.copy()

def update_contamination_parameters(state, params):
    if params is None:
        return state.parameters.copy()

    for key, value in params.items():
        if key not in state.parameters:
            continue

        if not _is_valid_parameter_value(key, value):
            continue

        if isinstance(state.parameters[key], bool):
            state.parameters[key] = bool(value)
        else:
            state.parameters[key] = float(value)

    return state.parameters.copy()

def _is_valid_parameter_value(key, value):
    """
    Validate contamination parameter values before accepting updates.
    """

    if value is None:
        return False

    numeric_positive_keys = {
        "default_frame_time_s",
        "calibration_time_s",
        "epsilon",
        "detector_surface_cm2",
        "alpha_warning_bq_cm2",
        "alpha_danger_bq_cm2",
        "beta_gamma_warning_bq_cm2",
        "beta_gamma_danger_bq_cm2",
        "max_valid_cps",
    }

    numeric_non_negative_keys = {
        "alpha_background_cps",
        "beta_gamma_background_cps",
        "min_valid_cps",
    }

    boolean_keys = {
        "stop_on_danger",
        "pause_on_danger",
        "warn_on_invalid",
        "auto_record_during_mission",
    }

    if key in numeric_positive_keys:
        try:
            return float(value) > 0.0
        except (TypeError, ValueError):
            return False

    if key in numeric_non_negative_keys:
        try:
            return float(value) >= 0.0
        except (TypeError, ValueError):
            return False

    if key in boolean_keys:
        return isinstance(value, bool)

    return False

def measure_contamination(state, get_sample_callback, continuous=False, frame_time_s=None):
    """
    Measure contamination over one frame, compute mean values, then assign a flag.

    If continuous is False:
        one frame is measured and returned.

    If continuous is True:
        this function still measures one frame only.
        The caller/ROS layer decides whether to call it again.
    """

    if frame_time_s is None:
        frame_time_s = state.parameters.get("default_frame_time_s", 5.0)

    frame_time_s = float(frame_time_s)

    if frame_time_s <= 0:
        result = {
            "success": False,
            "reason": "invalid_frame_time",
            "frame_time_s": frame_time_s,
            "sample_count": 0,
            "flag_result": {
                "flag": "INVALID",
                "reason": "invalid_frame_time",
            },
        }

        state.last_measurement_result = result
        state.last_flag_result = result["flag_result"]
        return result

    state.measurement_active = True
    state.measurement_continuous = bool(continuous)

    samples = collect_measurement_frame(
        state,
        get_sample_callback,
        frame_time_s
    )

    state.measurement_active = False

    if len(samples) == 0:
        result = {
            "success": False,
            "reason": "no_valid_samples",
            "frame_time_s": frame_time_s,
            "sample_count": 0,
            "flag_result": {
                "flag": "INVALID",
                "reason": "no_valid_samples",
            },
        }

        state.last_measurement_result = result
        state.last_flag_result = result["flag_result"]
        return result

    alpha_mean = mean(sample["alpha_cps"] for sample in samples)
    beta_gamma_mean = mean(sample["beta_gamma_cps"] for sample in samples)

    mean_measurement = {
        "alpha_mean_cps": alpha_mean,
        "beta_gamma_mean_cps": beta_gamma_mean,
    }

    flag_result = filter_flag(
        mean_measurement,
        state.parameters
    )

    result = {
        "success": True,
        "reason": "measurement_complete",
        "continuous": bool(continuous),
        "frame_time_s": frame_time_s,
        "sample_count": len(samples),
        "mean_measurement": mean_measurement,
        "flag_result": flag_result,
    }

    state.last_measurement_result = result
    state.last_flag_result = flag_result

    return result







