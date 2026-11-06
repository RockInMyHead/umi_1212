#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "MPU6050.h"

// The sensor instance is created in the sketch.
extern MPU6050 mpu;

// Calibrates accelerometer and gyro offsets. Keep the sensor flat and stationary.
inline void calibrate_mpu_after_button() {
    Serial.println("Starting MPU6050 calibration. Keep the sensor flat and stationary.");

    // Keep the built-in LED on while calibration is running.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    Serial.println("Calibration complete. Active offsets:");
    mpu.PrintActiveOffsets();

    digitalWrite(LED_BUILTIN, LOW);
}

#endif
