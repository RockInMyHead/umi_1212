#pragma once
#include "navigation.h"
// Replace with tools/fit_magnetometer.py output for the assembled device.
// Applied in the LIS3MDL chip frame BEFORE mag_to_body mounting rotation.
namespace magnetic_calibration {
constexpr bool provided=false;
constexpr imu::Vec3 hard_iron_uT={0,0,0};
constexpr double soft_iron[3][3]={{1,0,0},{0,1,0},{0,0,1}};
}
