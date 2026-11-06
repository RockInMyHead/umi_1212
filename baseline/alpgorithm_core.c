#include "algorithm_core.h"
#include <math.h>

// PHYSICAL DEFINITION OF GLOBAL VARIABLES (Fixes Linker Undefined Reference)
Vector3D acc_global_old = {0.0f, 0.0f, 0.0f};
Vector3D vel_global = {0.0f, 0.0f, 0.0f};

// Earth's exact local gravity constant (can be adjusted for latitude)
const float GRAVITY_EARTH = 9.80665f;

// Adaptive ZUPT Parameters
#define ZUPT_WINDOW_SIZE 16        // Window size for variance calculation (must be power of 2 for optimization)
const float ZUPT_THRESHOLD = 0.008f; // Variance threshold floor to detect absolute standstill

// Static Ring Buffers to monitor acceleration history in Global Frame
static Vector3D acc_window[ZUPT_WINDOW_SIZE] = {0};
static int zupt_idx = 0;
static bool window_ready = false;

/**
 * Tactical Function: Rotates a local sensor vector into the Global Earth Frame.
 * Uses Quaternion-derived rotation matrix for absolute precision.
 */
Vector3D rotate_to_global_frame(Vector3D local_vec, Quaternion q) {
    Vector3D global_vec;

    // Pre-calculate products for optimization and speed
    float ww = q.w * q.w;
    float xx = q.x * q.x;
    float yy = q.y * q.y;
    float zz = q.z * q.z;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float yz = q.y * q.z;
    float wx = q.w * q.x;
    float wy = q.w * q.y;
    float wz = q.w * q.z;

    // Apply exact Direction Cosine Matrix (DCM) derived from Quaternion
    global_vec.x = local_vec.x * (ww + xx - yy - zz) + local_vec.y * 2.0f * (xy - wz)         + local_vec.z * 2.0f * (xz + wy);
    global_vec.y = local_vec.x * 2.0f * (xy + wz)         + local_vec.y * (ww - xx + yy - zz) + local_vec.z * 2.0f * (yz - wx);
    global_vec.z = local_vec.x * 2.0f * (xz - wy)         + local_vec.y * 2.0f * (yz + wx)         + local_vec.z * (ww - xx - yy + zz);

    return global_vec;
}

/**
 * Helper function to clamp values between min and max to prevent mathematical domain errors
 */
float clamp_value(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * Processes raw localized acceleration, filters gravity via orientation, and maps true 3D trajectory.
 */
TacticalMovement execute_tactical_navigation(Vector3D acc_local_raw, Quaternion orientation, float dt) {
    TacticalMovement output = {0.0f, 0.0f, 0.0f, 0.0f};

    // 1. Project local acceleration into the stable Global World Frame
    Vector3D acc_global = rotate_to_global_frame(acc_local_raw, orientation);

    // 2. Exact Hard-Coded Gravity Compensation (Z-axis alignment with Earth core)
    acc_global.z -= GRAVITY_EARTH;

    // CRITICAL FIX: Update ZUPT Buffer BEFORE Noise Gate to prevent artificial zero-variance locks
    acc_window[zupt_idx] = acc_global;
    zupt_idx++;
    if (zupt_idx >= ZUPT_WINDOW_SIZE) {
        zupt_idx = 0;
        window_ready = true;
    }

    // 3. Hardware Noise Gate (Dead-Zone Filter to suppress mathematical drift)
    if (fabs(acc_global.x) < 0.15f) acc_global.x = 0.0f;
    if (fabs(acc_global.y) < 0.15f) acc_global.y = 0.0f;
    if (fabs(acc_global.z) < 0.15f) acc_global.z = 0.0f;

    // 5. Compute Statistical Variance Across the Window
    bool force_zero_velocity = false;
    if (window_ready) {
        Vector3D mean = {0.0f, 0.0f, 0.0f};
        Vector3D var  = {0.0f, 0.0f, 0.0f};

        // Calculate Mean
        for (int i = 0; i < ZUPT_WINDOW_SIZE; i++) {
            mean.x += acc_window[i].x;
            mean.y += acc_window[i].y;
            mean.z += acc_window[i].z;
        }
        mean.x /= (float)ZUPT_WINDOW_SIZE;
        mean.y /= (float)ZUPT_WINDOW_SIZE;
        mean.z /= (float)ZUPT_WINDOW_SIZE;

        // Calculate Variance (Mean Squared Error floor)
        for (int i = 0; i < ZUPT_WINDOW_SIZE; i++) {
            float dx = acc_window[i].x - mean.x;
            float dy = acc_window[i].y - mean.y;
            float dz = acc_window[i].z - mean.z;
            var.x += dx * dx;
            var.y += dy * dy;
            var.z += dz * dz;
        }
        var.x /= (float)ZUPT_WINDOW_SIZE;
        var.y /= (float)ZUPT_WINDOW_SIZE;
        var.z /= (float)ZUPT_WINDOW_SIZE;

        // Total operational variance magnitude
        float total_variance = var.x + var.y + var.z;

        // Lock condition trigger if total signal noise drops below the vibration floor
        if (total_variance < ZUPT_THRESHOLD) {
            force_zero_velocity = true;
        }
    }

    // 6. Jerk Derivative Calculation (Beta tracking for dynamic movement profiles)
    float jerk_x = (acc_global.x - acc_global_old.x) / dt;
    float jerk_y = (acc_global.y - acc_global_old.y) / dt;
    float jerk_z = (acc_global.z - acc_global_old.z) / dt;

    // Save historical state immediately before Taylor evaluation to prevent integration phase shift
    Vector3D acc_last = acc_global_old;
    acc_global_old = acc_global;

    // 7. High-Precision Kinematic Integration (Taylor Series Expansion)
    float dx = vel_global.x * dt + (acc_last.x * dt * dt) / 2.0f + (jerk_x * dt * dt * dt) / 6.0f;
    float dy = vel_global.y * dt + (acc_last.y * dt * dt) / 2.0f + (jerk_y * dt * dt * dt) / 6.0f;
    float dz = vel_global.z * dt + (acc_last.z * dt * dt) / 2.0f + (jerk_z * dt * dt * dt) / 6.0f;

    // 8. Update Tactical Velocity States for the next calculation matrix
    vel_global.x += acc_last.x * dt + (jerk_x * dt * dt) / 2.0f;
    vel_global.y += acc_last.y * dt + (jerk_y * dt * dt) / 2.0f;
    vel_global.z += acc_last.z * dt + (jerk_z * dt * dt) / 2.0f;
    // 9. Adaptive ZUPT Velocity Lock Execution
    // If variance analysis reports absolute sensor standstill, forcefully smash velocity vectors to absolute zero
    if (force_zero_velocity) {
        vel_global.x = 0.0f;
        vel_global.y = 0.0f;
        vel_global.z = 0.0f;

        // Suppress spatial displacements during confirmed static locked phase
        dx = 0.0f;
        dy = 0.0f;
        dz = 0.0f;
    }
    // 10. Calculate true Scalar Distance Displacement Vector (3D Euclidean metric)
    output.distance = sqrt(dx * dx + dy * dy + dz * dz);

    // 11. Establish Directional Vectors using absolute Cosine Space mappings
    if (output.distance > 0.00001f) {
        output.alpha = acos(clamp_value(dx / output.distance, -1.0f, 1.0f)) * 57.29578f; // Global X intercept angle
        output.beta  = acos(clamp_value(dy / output.distance, -1.0f, 1.0f)) * 57.29578f; // Global Y intercept angle
        output.gamma = acos(clamp_value(dz / output.distance, -1.0f, 1.0f)) * 57.29578f; // Global Z intercept angle
    }

    return output;
}
