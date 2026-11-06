#pragma once
#include "navigation.h"
// Replace using tools/fit_calibration.py output from this specific sensor.
// Identity means UNCALIBRATED, not perfect or factory-verified performance.
inline imu::Calibration measuredCalibration() { return imu::Calibration{}; }
inline bool measuredCalibrationProvided() { return false; }
