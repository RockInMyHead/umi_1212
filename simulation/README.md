# Reproducible eight-hour synthetic experiment

Run from the repository root with a C++17 compiler, Make, and Python 3:

```sh
python3 simulation/run_benchmark.py --sensitivity
```

This compiles and runs the actual production `src/navigation.cpp`, unit tests, and the cached original C++ estimator in `baseline/`. It runs nine scenarios for eight simulated hours at 50 Hz, with three random seeds per scenario: 38,880,000 time steps, each evaluating both estimators. It then runs three-minute sample-rate comparisons at 200, 100, and 50 Hz. The final profile is `st10dof_calibrated_synthetic`, following the hardware clarification: LIS331DLH accelerometer, I3G4250D gyroscope, LIS3MDL magnetometer, and LPS25HB pressure sensor. Earlier MPU-style intermediate results are superseded. Accelerated simulation is not an eight-hour hardware endurance test. `results/manifest.json` records exact source SHA-256 hashes, compiler, operating system, command, and elapsed wall time. Random normal sampling is reproducible within the recorded C++ standard library; the C++ standard does not guarantee identical variates across different libraries.

The only estimator used by the new-filter rows is the C++ implementation linked into the benchmark. The simulator neither duplicates that algorithm nor uses its rotation functions to generate truth. It evaluates an analytic trajectory, analytic velocity and acceleration, and analytic Euler-rate-to-body-rate conversion. Central finite differences independently check the translational derivatives and `dR/dt = R [omega]x` before every run. Three-point Gaussian quadrature produces interval-average accelerometer and gyro signals. Initial position, velocity, and orientation are known exactly in this experiment; both estimators receive that initial orientation, and all routes start at zero position and velocity. This is a favorable prior, not an achieved attitude-calibration result.

## Physical and stochastic model

The frame is local east/north/up at an assumed known latitude of 45 degrees north, and initial heading is surveyed. The compact true trajectories stay within tens of metres. Earth rotation is `Omega = (0, 7.292115e-5/sqrt(2), 7.292115e-5/sqrt(2)) rad/s`. Effective gravity is `(0,0,-9.80665) m/s²`. Ideal body measurements satisfy:

```
specific_force_body = R' (acceleration_nav - gravity_nav + 2 Omega × velocity_nav)
gyro_body           = omega_body_relative_to_nav + R' Omega
```

The new filter receives the correct Earth-rate configuration. The original baseline has no corresponding configuration. The model omits transport-rate changes, gravity gradients, lever-arm effects, clock jitter, sensor FIFO loss, mechanical vibration, saturation, and scale-factor/misalignment residuals. Coriolis and Earth rotation are included. Local-frame formulas cease to describe physically meaningful estimated positions after large unaided divergence; billion-metre error numbers indicate estimator failure, not navigation over those literal distances.

Each body-axis measurement has unknown-to-the-filter residual bias, temperature drift, bias random walk, white sample noise, and quantization. The residual biases assume that initial offset, scale, alignment, and thermal calibration have already been performed. These small residuals are substantially better than raw factory zero-rate errors and thermal shifts. The model values are engineering assumptions; they are **not measured performance or guaranteed ST10DOF accuracy**.

| Quantity | Assumed value |
|---|---|
| Accelerometer white diffusion amplitude | `0.004 m/s² sqrt(s)`; sample standard deviation `0.004/sqrt(dt)` |
| Gyro white diffusion amplitude | `0.0006 rad/s sqrt(s)`; sample standard deviation `0.0006/sqrt(dt)` |
| Initial accelerometer bias | `(0.006, -0.004, 0.008) m/s²` |
| Initial gyro bias | `(0.00003, -0.00002, 0.00004) rad/s` |
| Accelerometer bias random walk | `0.00002 m/s² /sqrt(s)` |
| Gyro bias random walk | `0.0000002 rad/s /sqrt(s)` |
| Accelerometer temperature slope | `(0.0008, -0.0005, 0.0010) m/s²/°C` |
| Gyro temperature slope | `(0.000003, -0.000002, 0.000002) rad/s/°C` |
| Temperature | `25 + 8(1-exp(-t/1800)) + 2 sin(2 pi t/7200)` °C |
| Accelerometer quantization increment | `0.002 × 9.80665 m/s²`, 12-bit ±4 g LIS331DLH |
| Gyro quantization increment | `0.0175 × (pi/180) rad/s`, selected ±500 dps I3G4250D |
| Independent position aiding | 1 Hz, independent Gaussian noise, `0.2 m` standard deviation per axis |

The LIS331DLH datasheet gives 2 mg/digit at ±4 g and a typical acceleration noise density of 218 µg/√Hz in its specified ±2 g condition. The I3G4250D specifies 17.5 mdps/digit at the selected ±500 dps range and typical rate-noise density 0.03 dps/√Hz. These justify device-appropriate quantization and caution against the much smaller gyro-noise assumption in the preliminary profile; continuous diffusion amplitudes here remain modeling choices, not direct datasheet guarantees. [LIS331DLH datasheet](https://www.st.com/resource/en/datasheet/lis331dlh.pdf), [I3G4250D datasheet](https://www.st.com/resource/en/datasheet/i3g4250d.pdf).

The filter receives temperature, but its temperature-calibration slopes remain zero; it does not receive the simulator's true residual bias or residual temperature model. Its conservative bias-walk priors are larger than the simulated walks, and the benchmark sets gyro diffusion to 0.0006 for both truth and filter. This experiment consequently does not establish optimal tuning. Hardware tuning requires stationary recordings, Allan analysis, temperature sweeps, verified reference motion, and timing/bandwidth characterization.

In the 10DOF scenarios, the true surveyed magnetic reference is `(0,20,-40) µT` in ENU. At 10 Hz, body-field measurements receive 0.4 µT independent Gaussian noise per axis, residual hard-iron offset `(0.1,-0.08,0.06) µT`, residual thermal slopes `(0.005,-0.004,0.003) µT/°C`, and the soft-iron matrix below. Each hour, a ten-second disturbance beginning at minute 30 adds `(30,-20,25) µT`. Quantization is `100/6842 µT` at ±4 gauss. The filter knows the reference field and white-noise variance, but not these residual errors or disturbance intervals. A field-norm check cannot detect every possible interference pattern; same-magnitude field rotations are an important unmodeled failure class.

```
soft_iron_residual = [[1.002, 0.001, -0.0005],
                     [0.001, 0.998, 0.0007],
                     [-0.0005, 0.0007, 1.001]]
```

The LIS3MDL datasheet gives sensitivity 6842 LSB/gauss at ±4 gauss. Its quoted noise values are condition-specific; the 0.4 µT simulator value is an engineering assumption rather than measured noise at this range. [LIS3MDL datasheet](https://www.st.com/resource/en/datasheet/lis3mdl.pdf).

The final 25-state estimator includes three persistent body-frame residual magnetic-bias states, initially zero with 1 µT standard deviation and 0.002 µT/√s bias random walk. It also represents six symmetric residual soft-iron coefficients, initially zero with fractional standard deviation 0.005 and random walk 0.000001/√s. These are uncertainty assumptions, not knowledge of the simulated offsets or matrix. The earlier 16-state model treated calibrated magnetic measurements as white-noise observations; the 19-state intermediate added bias uncertainty but did not capture orientation-dependent scale/cross-axis residuals. The benchmark deliberately retains these residual errors to expose covariance overconfidence. None of these states models arbitrary environmental interference or unknown sensor-to-body alignment. `--mag-systematics 0` removes only the residual hard-iron, soft-iron, and magnetic temperature terms; white noise, quantization, transient magnetic disturbances, all inertial errors, and weather drift remain unchanged. The default is `--mag-systematics 1`.

Barometer observations arrive at 25 Hz. Standard-atmosphere pressure is generated as `p=1013.25(1-z/44330)^5.255` hPa, with weather offset `-0.04 t/3600 + 0.05 sin(2 pi t/21600)` hPa, independent 0.018 hPa white noise (approximately 0.15 m), and quantization 1/4096 hPa. The inverse pressure-altitude formula yields the relative-altitude observation. The initial pressure reference is assumed known exactly. Weather is not supplied to the filter; its barometer-bias state cannot in general distinguish atmospheric drift from true slow vertical displacement. The LPS25HB supports 25 Hz and 4096 LSB/hPa; output noise depends on filtering. [LPS25HB datasheet](https://www.st.com/resource/en/datasheet/lps25hb.pdf).

## Scenarios and information differences

1. `stationary_verified`: fixed tilted sensor. The new filter is told that independent evidence confirms translational and rotational rest. This permits zero-velocity and stationary-gravity/rate updates. The baseline receives the same IMU values but cannot consume this external evidence.
2. `stop_go_verified`: a 120-second cycle with 20 seconds at rest, an 80-second quintic translation between endpoints 20 m east, 10 m north and 2 m up, then 20 seconds at rest. Alternate cycles reverse the trajectory. Orientation changes smoothly with translation. Rest evidence is supplied only when the full sample interval is stationary. Zero velocity does not reveal where the sensor stopped.
3. `continuous_imu_only`: a bounded, smoothly started three-dimensional periodic route, with sinusoidal roll, pitch, and yaw. No verified-rest flag or external position is supplied. This is the direct comparison with equal measurement information. It exposes the unobservability of unconstrained long-term position and heading, including cases in which the baseline's erroneous motion suppression happens to give smaller position error.
4. `continuous_position_1hz`: the exact same continuous route and IMU samples, with external position at 1 Hz for the new filter. Improvement is conditional on adding that independent sensor; it cannot be credited to software alone.
5. `continuous_position_outages_outliers`: identical IMU and position-noise streams, but position updates are absent for 60 seconds at hours 1, 3, 5, and 7. Every 1,000-second position observation, when available, additionally has `(100,-100,100) m` error. The innovation gate must reject such observations. Final error alone conceals the outages; inspect maximum and RMS errors and the time series.
6. `continuous_10dof_only`: the same continuous route and IMU realization, with the onboard magnetometer and barometer observations described above, but no external position or verified rest. This is the relevant unconstrained ST10DOF navigation case. Magnetic heading and pressure altitude do not constitute a horizontal position reference.
7. `stop_go_10dof_verified`: the stop/go route with onboard magnetometer and barometer and genuinely independently verified complete stops. A body-mounted or handheld unit does not receive a zero-velocity update merely because a foot enters stance.
8. `continuous_10dof_position_1hz`: the continuous route with all onboard sensors plus independent 1 Hz position. This explicitly requires an additional positioning sensor beyond ST10DOF.
9. `walking_10dof_no_stance`: the continuous route with a smooth 30-second onset of 1.8 Hz translation and attitude oscillations resembling a carried device. Translation amplitudes are 2.5 cm east, 1.5 cm north, and 3.5 cm up; attitude amplitudes are 0.10, 0.08, and 0.06 radians. No stance or zero-velocity flag is supplied. This is a generic analytic oscillating trajectory, not a validated model of a particular person's gait or mounting location.

Seeds 1–3 run independently. Scenarios 3–6 and 8 use identical IMU realizations; scenarios 2 and 7 share the stop/go realization. Independent engines and distribution instances generate position, magnetic, and pressure noise, so aiding cannot alter the IMU stream. Corresponding baseline rows must match exactly within a seed. The code deliberately does not impose zero velocity on quiet-but-moving IMU signals. A baseline with aggressive motion clipping can produce deceptively smaller errors on a bounded route; this is not evidence that it measures motion correctly. The baseline consumes only accelerometer and gyro measurements, so 10DOF-versus-baseline comparisons include added information as well as algorithm changes.

## Results and interpretation

`results/summary.csv` contains 54 rows: nine scenarios × three seeds × two estimators. Position final, maximum, and RMS errors are Euclidean norms computed at **every** sample. Separate horizontal and vertical errors expose the unobserved horizontal drift and pressure-height limitations. Tilt error is the angle between true and estimated body-frame gravity directions. Yaw error is the shortest wrapped horizontal-heading difference. Path is the integral of speed; the signed path error is estimated minus true path. Magnetic/barometric acceptance and rejection counts are separate from aggregate update counts. These metrics describe different quantities and are not interchangeable.

`coverage_xyz_3sigma` is the fraction of samples for which all three position errors separately lie within three reported standard deviations. It is not a chi-squared NEES test, a probability guarantee, or independent experimental validation; samples are strongly correlated. Three seeds are a regression experiment, not a large Monte Carlo confidence study. Filter fault bits and failed prediction samples are recorded separately from large-but-finite estimation error. Large covariance or zero fault flags do not make a divergent trajectory usable. `final_covariance_cholesky_ok` checks positive definiteness of the diagonally scaled final covariance without adding numerical jitter; it does not guarantee statistical consistency or positive definiteness at every intervening sample.

`ablation_no_mag_systematics/` records the continuous 10DOF scenario with the systematic magnetic residuals disabled. Reproduce it with `python3 simulation/run_benchmark.py --scenario 5 --mag-systematics 0 --output simulation/ablation_no_mag_systematics`. It is an explanatory ablation, not a claim that real magnetic calibration is perfect. The paired model comparison evaluates the earlier 16-state and 19-state variants against the final 25-state variant using exactly the same updated sensor quantization, signal generator, and seeds. `results/model_comparison.csv` contains individual seed results; the adjacent JSON records source/data hashes and confirms paired baseline agreement. Earlier candidate source snapshots remain in the local working files. These model changes were motivated by tests on seeds 1–3, so those seeds are development regressions.

The additional `holdout_continuous/`, `holdout_walking/`, and `holdout_aided/` directories use seeds 4–6 after the model and priors are frozen. Reproduce with `python3 simulation/run_benchmark.py --scenario 5 --seed-start 4 --output simulation/holdout_continuous`, the same command with `--scenario 8 --output simulation/holdout_walking`, and `--scenario 7 --output simulation/holdout_aided`. These are held-out noise realizations under the same systematic-error and trajectory assumptions, not independent hardware or environmental validation. The final model is not tuned to these held-out results.

`results/timeseries.csv` contains one snapshot per minute plus samples immediately before and at each outage boundary. It is compact for plotting; use `summary.csv` for true samplewise maximum error. `sensitivity/` contains short runs with fixed diffusion amplitudes at different sample rates. Their random samples differ with rate, so they test stability and broad sensitivity rather than deterministic convergence. The 50 Hz IMU grid represents hypothetical time-aligned interval measurements after resampling; it does not reproduce the devices' native ODR, digital filters, or asynchronous data-ready timing. These runs do not validate firmware timing or noise characteristics. Magnetometer 10 Hz and barometer 25 Hz updates are synchronous in the model and have no unmodeled latency.

Unit tests cover axis/quaternion conventions, tilted rest, acceleration below the old dead zone, moving constant velocity without false ZUPT, exact constant-rate quaternion integration, invalid input and fail-stop behavior, delayed/future aiding rejection, measurement outlier gating, covariance symmetry/Cholesky checks, and an independent infinitesimal covariance-sign oracle. Tests intentionally distinguish measurement availability from apparent IMU quietness.

`tests/physics_tests.cpp` additionally checks an eight-hour constant-spin covariance invariant, independent finite differences of the gravity/Earth/force Jacobians, Earth static equilibrium, and update-correction signs. `tests/trajectory_tests.cpp` uses an independent noiseless analytic moving/rotating trajectory with Earth effects for 240 seconds. Halving `dt` from 0.02 to 0.01 to 0.005 seconds reduces the final position integration error from 0.00796298 to 0.00199074 to 0.000497685 metres: the expected second-order convergence. This demonstrates integration convergence under ideal input, not hardware position accuracy.
