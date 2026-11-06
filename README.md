# Accel/gyro processing — ST 10-DOF wearable revision

This local revision targets the user's **LIS331DLH + I3G4250D + LIS3MDL + LPS25HB** module, carried by hand or mounted on the body. It replaces the inconsistent navigation pipeline recovered from the original repository with a tested C++ error-state Kalman filter, calibration tools, sensor integration examples and reproducible eight-hour simulations.

**The module alone cannot guarantee small horizontal position error over eight hours.** A magnetometer measures magnetic field and a barometer measures pressure; neither supplies horizontal position. The experiments deliberately include failure cases as well as position-aided cases. The low-error position-aided results require an additional independent reference that is not present on this module. See the actual [results and charts](docs/RESULTS.md), not a claimed “absolute minimum” error.

The live GitHub repository was unavailable during implementation. This is a downloadable replacement package based on recovered cached sources, **not a pushed change or verified current checkout**. One listed legacy file could not be recovered. The [repository audit](docs/REPOSITORY_AUDIT.md) identifies every inspected, replaced and unavailable file.

## What changed

- One consistent Hamilton quaternion convention, exact finite rotation updates, midpoint force rotation, and optional local Earth rotation/Coriolis correction.
- A **25-state error model**: position, velocity, attitude error, accelerometer bias, gyro bias, pressure-altitude bias, residual magnetic bias, and six symmetric magnetic scale/cross-axis terms. Persistent calibration error is not treated as infinitely averageable white noise.
- Joint innovation gates, covariance correction and attitude tangent-frame reset; exact angular covariance propagation prevents artificial growth during continuous rotation.
- Magnetometer fusion after hard/soft-iron calibration and axis alignment; magnetic magnitude and innovation checks reject obvious interference. Some disturbances remain indistinguishable from real field changes.
- Barometric relative-altitude fusion with a separate pressure-offset state. Weather uncertainty remains; factory temperature compensation does not remove weather.
- Removed per-axis acceleration dead zones and automatic zero-velocity clamping. Quiet signals are diagnostic only. True rigid rest must be independently asserted.
- Explicit timestamp, sample loss, saturation and nonfinite-input handling; no fabricated elapsed time after missing data. Faults latch until deliberate reinitialization.
- Actual ST register-level integration with checked transactions, separate sensor freshness/timing, per-chip axis mapping, and independent radio scheduling. The old MPU6050 adapter is retained only as a legacy example.

## Build and reproduce

The numerical core and tests require a C++17 compiler and Make, with no external matrix library:

```sh
make test
python3 simulation/run_benchmark.py --sensitivity
```

The benchmark runs the **actual C++ core**, not a separate mathematical imitation. It compares unchanged recovered baseline C++ code on identical IMU samples. Its manifest records the source hashes, compiler, command and runtime. The simulation contains known analytic truth, noise, quantization, thermal drift, Earth rotation, magnetic calibration residuals/interference, changing pressure and reference outages. Three seeds are regression evidence, not a population-level accuracy guarantee.

Optional calibration/adapter checks and recorded-data replay:

```sh
make test-tools  # Python 3 + NumPy for the magnetic fit
make replay
./work-build/replay initial_state.csv samples.csv estimates.csv
```

See [replay format and synchronization requirements](docs/REPLAY.md). Re-render the scientific charts with `python3 simulation/plot_results.py` after installing Matplotlib in your own environment.

## Hardware integration

Start with [the actual module's commissioning guide](docs/HARDWARE.md) and `firmware/st_imu_lora/`. The application MCU was not specified. The provided platform adapter assumes **ESP32** and requires explicit pin configuration and real board compilation. The algorithm itself is portable C++.

The four ST sensors have separate clocks, addresses and frames. The example selects ±4 g accelerometer and ±500 degrees/s gyro ranges, polls checked data, and exposes timing uncertainty. Asynchronous fusion in the example uses documented age-dependent measurement uncertainty; demanding dynamic applications require measured delays and timestamp-aligned replay. The long simulation does not validate this approximation or firmware execution time.

Magnetic fusion starts disabled until calibration, axis mapping and a valid local field reference are supplied. A uniform-field ellipsoid fit cannot determine sensor-to-body rotational alignment. A generated calibration header is not a substitute for an independent check. The barometer needs an initial pressure reference and a declared air-temperature model.

**For a hand or torso, a detected footstep is not a stationary interval.** Keep the rest input false unless the module itself is genuinely motionless. A foot-mounted navigation algorithm cannot be transplanted unchanged to a handheld unit. Pedestrian dead reckoning would additionally need a validated gait model, step length and walking-direction estimate; device yaw is not necessarily walking direction. This package does not silently impose those assumptions.

## Where to look

| Location | Purpose |
|---|---|
| `src/navigation.h`, `src/navigation.cpp` | Portable estimator and explicit API contract |
| `firmware/st_imu_lora/` | Actual four-chip ST module integration example |
| `tools/fit_calibration.py` | Six-face acceleration and gyro-temperature calibration |
| `tools/fit_magnetometer.py` | Hard/soft-iron ellipsoid fit with held-out screening |
| `tools/replay.cpp` | Replay calibrated recorded IMU/magnetic/barometric/aiding data |
| `tests/` | Numerical, physical, observability and convergence regressions |
| `simulation/` | Independent truth generator, seeded eight-hour runs and provenance |
| `baseline/` | Recovered original sources, excluded from the deployed estimator |
| [Mathematics](docs/MATHEMATICS.md) | Equations, Jacobians, units, approximations and physical limits |
| [Validation plan](docs/VALIDATION_PLAN.md) | What a real eight-hour acceptance test still needs |

## What is verified, and what is not

Host tests check quaternion/measurement signs, zero-motion gravity cancellation, slow acceleration, no false rest during constant velocity, long-run covariance behavior, magnetic/pressure observability, input faults, calibration fitting and telemetry encoding. A noiseless independent trajectory demonstrates second-order integration convergence as the timestep is reduced. Exact results are in [executed validation](tests/VALIDATION.md).

There is no real sensor recording, motion-capture/reference trajectory, complete target-controller specification or board endurance test in this task. Sensor noise and residual calibration values in the simulator are explicitly assumed. Neither the uncertainty matrix nor a good synthetic run certifies a real eight-hour error bound. Strong absolute-position performance requires observable reference information and measured hardware calibration, not merely a more complicated filter.
