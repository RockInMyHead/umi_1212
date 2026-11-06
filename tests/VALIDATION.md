# Executed validation

On the recorded Apple Clang C++17 environment, `make all test` passed:

- `navigation_tests`: 9,240 assertions, zero failures. Most assertions check each propagation step; this is not 9,240 independent experiments.
- `physics_tests`: 11,993 assertions, including eight simulated hours of constant spin with a covariance invariant, measurement-Jacobian finite differences, Earth equilibrium, and correction signs. The expanded ST10DOF suite also checks magnetic heading correction, field-magnitude and same-magnitude directional disturbance rejection, absent yaw information in a vertical magnetic field, the fixed-pose magnetic heading/bias ambiguity, six soft-iron derivative and correction cases, retained heading/scale ambiguity, and the barometric height/offset ambiguity.
- `trajectory_tests`: independent noiseless 240-second translation/rotation with Earth effects. Final position integration errors for time steps 0.02, 0.01, and 0.005 seconds were 0.00796298 m, 0.00199074 m, and 0.000497685 m. Halving the time step reduced the error by four, as required by the second-order test.

The navigation and expanded physics test executables also passed AddressSanitizer and UndefinedBehaviorSanitizer with no diagnostics. For example:

```sh
c++ -Isrc -std=c++17 -O1 -g -fsanitize=address,undefined \
  -fno-omit-frame-pointer tests/navigation_tests.cpp src/navigation.cpp \
  -o /tmp/navigation_tests_sanitized
/tmp/navigation_tests_sanitized
```

`make replay test-tools` also passed compilation of the recorded-data replay tool, six Python calibration/protocol tests, three NumPy magnetometer-fit tests, portable adapter primitives, and ST register/conversion tests with a fake bus. `make test` requires only the C++ toolchain; `make test-tools` additionally requires Python and NumPy. Fake-bus tests cannot verify electrical behavior or hardware timing.

An eight-hour paired-world pressure test supplies identical measurements for either a stationary device with pressure-reference drift or a device climbing 30 m at 1.04 mm/s with no weather drift. The estimator cannot distinguish these worlds: its final errors are approximately 29.9998 m and 0.000167 m respectively. This deliberately demonstrates an observability limitation, not an implementation failure.

The long benchmark runs the actual C++ estimator for nine eight-hour synthetic ST10DOF scenarios and three independent seeds. Nine further eight-hour trials use held-out seeds 4–6 for continuous onboard-only, walking-like onboard-only, and position-aided motion. Three eight-hour magnetic-systematics ablations complete 39 full-duration trials of the frozen 25-state model. All 39 have zero failed predictions and numeric fault bits, and pass the final covariance Cholesky check. Exact commands, source hashes, and environments are recorded beside each result set; the main run is `simulation/results/manifest.json`. These are accelerated simulations, not 312 hours of hardware operation.

The position-aided scenario achieved 0.2119–0.2138 m RMS 3D error on development seeds and 0.2108–0.2134 m on held-out seeds. This assumes independent 1 Hz position fixes with 0.20 m Gaussian noise per axis. Onboard-only horizontal estimates remain unusable. Their divergence and the calibration-ablation results are reported in `docs/RESULTS.md`; zero numeric-fault bits and 100% reported-box coverage are not accuracy guarantees.

Replay smoke checks passed for both 18/24-column sample formats and 23/26-column initial-state formats, including level rest, magnetic/barometric update counts, and repeated-timestamp rejection. ST and legacy Arduino staging preserved the canonical core source byte for byte; staging and fake-bus checks are not a real board build.

No test in this package establishes measured noise parameters of the actual ST10DOF unit, on-device CPU throughput, interrupt/data-ready timing, radio reliability, or measured eight-hour hardware performance. Those require the target board and reference recordings.
