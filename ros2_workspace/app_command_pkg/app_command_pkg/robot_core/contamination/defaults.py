DEFAULT_CONTAMINATION_PARAMETERS = {
    # Measurement frame
    "default_frame_time_s": 5.0,
    "calibration_time_s": 30.0,

    # Detector
    "epsilon": 0.35,
    "detector_surface_cm2": 100.0,

    # Background noise
    "alpha_background_cps": 0.0,
    "beta_gamma_background_cps": 0.0,

    # Flag thresholds in Bq/cm²
    "alpha_warning_bq_cm2": 0.4,
    "alpha_danger_bq_cm2": 4.0,
    "beta_gamma_warning_bq_cm2": 0.4,
    "beta_gamma_danger_bq_cm2": 4.0,

    # Sensor sanity limits
    "min_valid_cps": 0.0,
    "max_valid_cps": 100000.0,

    # Behavior defaults
    "stop_on_danger": False,
    "pause_on_danger": True,
    "warn_on_invalid": True,

    # Recording
    "auto_record_during_mission": True,
}