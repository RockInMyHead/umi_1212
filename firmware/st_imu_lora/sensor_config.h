#pragma once
#include "navigation.h"
#include "magnetometer_calibration.h"
namespace sensor_config {
// Set true only after measured axis/sign checks for all installed chips.
constexpr bool mounting_verified=false;
// Map each chip frame into the SAME body frame. Orthogonal, determinant +1.
constexpr double accel_to_body[3][3]={{1,0,0},{0,1,0},{0,0,1}};
constexpr double gyro_to_body[3][3]={{1,0,0},{0,1,0},{0,0,1}};
constexpr double mag_to_body[3][3]={{1,0,0},{0,1,0},{0,0,1}};
// Calibrate accelerometer/gyro AFTER these mounting transforms; the navigation
// Calibration matrices/offsets are then defined in the common body frame.
constexpr bool magnetometer_enabled=false;
// Known, calibrated local field in your chosen local navigation frame, microtesla.
// Zero is deliberately not a fabricated magnetic field or a geographic heading.
constexpr imu::Vec3 magnetic_field_nav={0,0,0};
constexpr double local_gravity_m_s2=9.80665;
// Zero disables Earth-rate correction. Supply only in a surveyed/georeferenced
// navigation frame; an ENU Earth vector is wrong in arbitrary startup heading.
constexpr imu::Vec3 earth_rate_nav_rad_s={0,0,0};
constexpr double magnetic_sigma_uT=1.5; // tuning prior, not measured accuracy
constexpr bool barometer_enabled=true;
constexpr double barometer_sigma_m=0.7; // include thermal/pressure/timing errors
// Hypsometric scale requires ambient AIR temperature, not a self-heated chip.
constexpr double mean_air_temperature_k=288.15; // standard-atmosphere assumption
constexpr double magnetic_latency_rotation_bound_rad_s=4.0;
constexpr double barometric_vertical_speed_bound_m_s=5.0;
} // namespace sensor_config
