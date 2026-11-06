# Eight-hour validation plan

This is the acceptance plan and limitations checklist. It distinguishes tests that should be run from results actually recorded by the repository. A listed scenario is **not** a claim that it has passed. Use the generated results/report for executed runs and their exact parameters.

## 1. Define the experiment before tuning

The intended module contains LIS331DLH, I3G4250D, LIS3MDL and LPS25HB sensors. Record unit identity, firmware revision, each sensor's full-scale range, filter setting, independent sample timestamps, calibration matrices, temperature range, mounting, motion type and available external observations. Specify whether the target is position, travelled distance, velocity or attitude: improving one does not guarantee improvement in the others. The original MPU6050 benchmark settings must not be presented as measurements of this module.

For an eight-hour claim, simulated or recorded elapsed time must reach 28,800 seconds. Multiplying the last error of a short run by a factor is not an eight-hour test. At 100 Hz this means 2,880,000 propagated intervals per run. Execution faster than real time is valid for numerical simulation and is not a hardware soak test.

Keep model-development/tuning seeds separate from validation seeds. Save all seeds, configuration, compiler settings and the code revision. Compare algorithms using the same generated sensor samples and the same initial information. If only the new implementation can consume external measurements, report its aided run separately from an unaided baseline comparison.

## 2. Independent trajectory and sensor models

Generate continuous truth independently of estimator mechanization. Derive position, velocity, acceleration, orientation and angular velocity analytically where possible. Otherwise integrate truth at a substantially higher rate and aggregate into the estimator's interval-average input model. Reusing the estimator's own integrator to generate truth can conceal discretization errors.

For the declared local Earth model, construct ideal measurements from truth as

```
f_body = R_truth^T (a_truth - g_n + 2 Ω_n × v_truth)
w_body = body angular rate relative to nav + R_truth^T Ω_n.
```

Apply independent sensor scale/misalignment, offsets, temperature drift, residual bias random walk and white noise. Include quantization and clipping in dedicated robustness runs. If correlated digital filtering is modeled, account for its phase delay and autocorrelation; do not claim an independent-noise experiment validates that case.

Truth bias parameters must not be directly exposed to the estimator except where explicitly testing perfect calibration. The default engineering noise priors are not measured module Allan parameters. Datasheet noise floors at specified conditions do not determine temperature hysteresis, bias stability or mounting errors.

For magnetic truth, generate `m_true=R_true^T B_true`, then add calibration residuals, noise and separately controlled interference. Give the estimator only its declared reference field, not the instantaneous disturbed truth. Exercise fields with different inclination and horizontal strength. Treat mag samples as their own observations at their own acquisition times; do not repeat a single sample as fresh evidence at every IMU tick.

For pressure truth, generate pressure from independently defined height and atmospheric reference-pressure/temperature histories, then add pressure sensor noise and bias before converting to relative altitude. Include uncertainty in the initially measured reference pressure. A benchmark that generates altitude directly with independent Gaussian noise and zero weather is useful but must be labeled as that simplified model. It does not validate pressure conversion, enclosure dynamics or atmospheric drift.

## 3. Algebra and deterministic regression checks

1. **Frame convention:** identity and ±90° rotations about X/Y/Z, plus a noncommuting rotation sequence. Check that synthetic stationary force rotates to `[0,0,g]` and gravity removal is zero.
2. **Jacobian signs:** use central finite differences for `R Exp(δθ)f`, `R^T g_up`, `R^T Ω_n`, all observable measurement blocks and the quaternion injection/reset. Check nontrivial attitude, force and angular-rate vectors.
3. **Covariance propagation:** check symmetry, nonnegative eigenvalues or successful positive-definite factorization after an appropriate numerical tolerance, and dimensional time scaling. Verify position/velocity process-noise cross covariance.
4. **Correction consistency:** position and velocity residuals must reduce when accepted; a large innovation must be rejected without changing state/covariance. Rejected, nonfinite or stale measurements must not be counted as accepted.
5. **Small-angle and large-angle math:** zero rotation, tiny rotation, a finite one-axis rotation and many repeated turns. Verify finite values and unit quaternion norm. Small-angle limiting branches must match their surrounding formulas.
6. **Constant force:** with known attitude and exact input, reproduce `v=v0+a t` and `p=p0+v0 t+a t²/2` within the stated integration approximation.
7. **Constant velocity:** initialize known nonzero velocity and provide gravity-only, zero-body-turn data with `verified_stationary=false`. Preserve travel; quiet must not zero velocity.
8. **Slow real motion:** use acceleration below the old 0.08 m/s² dead zone. The new core must integrate it, rather than silently discard it.
9. **Earth convention:** a truly fixed sensor must measure `R^T Ω_n`; with correct Earth configuration, attitude should remain fixed. A turning sensor must use the changing body-frame Earth vector.
10. **Magnetic heading:** a small initial yaw error must be corrected with the correct sign under a known nonvertical field. Verify the magnetic Jacobian by finite differences. A vertical field must not manufacture yaw information.
11. **Magnetic rejection:** reject a large field-magnitude error. Also test a directionally rotated field with unchanged magnitude and informative attitude covariance; norm gating alone cannot detect it. Preserve state and covariance on rejection. Separately expose the failure case where broad heading uncertainty permits disturbed measurements through the gate.
12. **Pressure-altitude gauge:** with equal uncertain initial height and offset, a single barometer observation must update their sum and retain uncertainty in their difference. Under an independently known height, the same observation can estimate offset. Reject inconsistent large jumps without changing state or covariance.
13. **Persistent magnetic-bias gauge:** at a fixed pose, repeated field observations must retain the analytic yaw/bias ambiguity. Check both zero-innovation observations and a fixed unknown residual offset. Bias and attitude may share the correction according to their priors; neither is uniquely identified by that pose.
14. **Persistent magnetic-scale gauge:** verify all six symmetric-scale columns and the scaled attitude Jacobian independently. Use basis-axis fields and known attitude for black-box sign/gain checks. At a fixed pose preserve the analytic yaw/scale uncertainty and coefficients whose field component is zero. Confirm that invalid nonpositive calibration matrices raise a fault rather than being silently clipped.

Numerical finite-difference tests validate local derivatives. They do not validate sensor parameters, physical observability or eight-hour performance.

## 4. Duration and motion matrix

Run the following at the intended sampling rate; use several seeds for each stochastic condition.

| Scenario | Available information | Purpose |
|---|---|---|
| Eight-hour stationary, fixed temperature | Independently verified rigid rest | Numerical longevity and stationary bias behavior; not proof of navigation accuracy |
| Eight-hour stationary with warm-up, temperature cycles and model mismatch | Same rest assertions; calibrated and uncalibrated temperature variants | Test temperature compensation and residual uncertainty |
| Eight-hour repeated accelerate/cruise/decelerate/rest, with changing direction | Known initial pose/velocity and only truthful stop assertions | Assess aided-by-stops drift without fabricating absolute references |
| Same trajectory with no stop assertions | Same IMU and initialization | Expose unaided growth and demonstrate that quiet is not used as ground truth |
| Continuous smooth motion without stops | Optional explicit noisy position/velocity updates | Assess a practical route to long-duration bounded error |
| Same aided trajectory with observation outages and injected outliers | Only time-correct measurements available at that instant | Recovery, innovation gating and honest uncertainty growth |
| 3D tilted motion, yaw turns and noncommuting angular motion | Declared observations only | Catch gravity sign, timing, coning and excitation weaknesses |
| Slow constant-speed platform and slow rotations | No rest assertions | Detect false stationarity and false zero-angular-rate bias learning |
| Strong vibration and high acceleration | Stated bandwidth, clipping and sample model | Quantify model failure outside benign dynamics |
| Eight-hour 10DOF trajectory, quiet magnetic environment | Calibrated field, declared reference, pressure origin and truthful stops only | Assess actual additional module sensors without inventing horizontal position observations |
| Same trajectory with magnetic interference, including same-norm rotations | Identical estimator reference and gates | Show rejection, heading degradation and recovery; report rejected fraction |
| Constant-height weather drift and a real slow climb with corresponding pressure evolution | Same pressure model, with and without truthful stops/height reference | Expose height/offset ambiguity instead of silently treating barometer height as truth |
| Pressure port transients and repeated stale mag/baro data | Realistic separate acquisition timestamps | Check protection against transients and overcounting observations |
| Magnetic systematics ablation | Identical IMU/white-noise streams, first with and then without calibration residuals | Separate systematic-model inconsistency from mechanization and white-noise behavior |

Use trajectories that return to their origin **and** trajectories that do not. Small final error on a closed path can coexist with large intermediate errors. A true-stationary synthetic run can look excellent simply because zero velocity supplies the answer; report it as that restricted condition.

If the simulation supplies true stop labels, describe them explicitly as **truth-provided stop assertions**. This tests estimation given a valid constraint, not automatic stop recognition. Never train or tune a detector on truth and then report those same labels as inferred sensor output.

## 5. Metrics

For every run, report elapsed duration, sample count/rate, random seed, calibration assumptions, aiding schedule and missing-data policy. At minimum measure:

- Final, RMS and maximum 3D position error against truth.
- Horizontal and vertical errors separately, and velocity error.
- Attitude geodesic error `2 acos(clamp(abs(dot(q_est,q_true)),0,1))`; do not compare quaternion components directly.
- Heading error separately from total attitude error; magnetic validity and reference-field assumptions must accompany it.
- Pressure-altitude offset error, vertical position error, and their covariance/correlation separately; a good fit to their sum can hide a wrong separation.
- Residual magnetic-bias/scale error and heading/calibration covariance; include fixed-pose and diverse-rotation conditions. Do not score calibration estimation as uniquely correct where it is unobservable.
- Endpoint displacement error separately from travelled-distance error.
- Accepted/rejected measurements, faults, false rest assertions/detections and actual aiding outages.
- Empirical coverage of stated uncertainty, with NEES/NIS only where the corresponding statistical assumptions and observable state comparison are valid.
- Runtime and memory on the **target processor**, plus missed samples and FIFO overruns. Desktop throughput is not embedded throughput.

For an ensemble, report the number of independent seeds and the median, 95th percentile and worst observed metrics. A 95th percentile computed from a tiny seed count is unstable; retain individual runs and do not call the largest observed error a guaranteed bound. Avoid quoting a percentage improvement if the baseline denominator is near zero or if input/aiding conditions differ.

If empirical coverage is grossly below the stated interval level, treat it as a failed uncertainty model even when the covariance is numerically positive definite and no fault bit is set. Investigate calibration residuals, temporal correlation, missing nuisance states, nonlinear observability and discretization with controlled ablations. Do not repair the reported result merely by changing its label or multiplying uncertainty until the validation seeds pass; confirm changes on independent conditions and seeds.

## 6. Timestamp, fault and deployment tests

Test repeated/reversed timestamps, missing samples, short and long gaps, nonfinite input, zero/invalid quaternion, saturation, invalid covariance and long-running sensor-clock wrap. Handle each explicitly and verify what state/time/covariance remains after rejection. A gap must not be filled by replaying a stale sample as if it were new measurement evidence.

With real hardware, use data-ready acquisition or the FIFO and determine the actual sample period. LoRa transmission and logging must not change integration intervals or block acquisition. Verify gyro/accelerometer synchronization and DLPF timing. Record raw samples so the exact stream can be replayed through both implementations.

Stationary acceleration and angular-rate updates reuse IMU data from propagation; independent-measurement covariance is an approximation. Dedicated consistency tests should expose any resulting overconfidence. When the physical sensor is clipped, inputs have been irreversibly lost; rejection and covariance growth cannot reconstruct the true motion.

## 7. Hardware evidence required before an accuracy claim

1. Warm up the assembled device and measure static noise and bias at relevant temperatures, filter settings and supply conditions. Characterize Allan deviation over a sufficient record and inspect stationarity of the record itself.
2. Calibrate accelerometer bias/scale across opposing faces, check noncalibration orientations, and calibrate gyro rate scale against a reference turntable if required by the budget.
   Calibrate the magnetometer in the assembled, powered enclosure across varied orientations; check sensor-to-body alignment and current-dependent interference. Compare barometric height against an independent height/reference-pressure record while temperature and weather change.
3. Record an independent position/attitude reference, with known accuracy and synchronization, over representative motion and stops. Ensure calibration and validation datasets differ.
4. Run the assembled acquisition, computation and radio system for eight real hours. Retain temperature, timing, saturation, fault and packet-loss logs.
5. Repeat across units, temperatures, mounting conditions and motion profiles before generalizing beyond the tested setup.

Until that evidence exists, describe numerical outputs as **simulation results under declared assumptions**. Neither the simulation nor a covariance matrix proves the absolute smallest achievable real-world error.

## Primary references

- [ST LIS331DLH datasheet](https://www.st.com/resource/en/datasheet/lis331dlh.pdf) and [I3G4250D datasheet](https://www.st.com/resource/en/datasheet/i3g4250d.pdf): actual module inertial sensor characteristics and conversion conditions.
- [ST LIS3MDL datasheet](https://www.st.com/resource/en/datasheet/lis3mdl.pdf) and [LPS25HB datasheet](https://www.st.com/resource/en/datasheet/lps25hb.pdf): magnetic/pressure noise, range and operating modes; not navigation accuracy guarantees.
- [NOAA Air Pressure](https://www.noaa.gov/jetstream/atmosphere/air-pressure): pressure varies with atmospheric conditions even without sensor motion.
- [Joan Solà, Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/pdf/1711.02508): quaternion conventions, local error dynamics and tangent-frame reset.
- [Johan Wahlström and Isaac Skog, Fifteen Years of Progress at Zero Velocity](https://arxiv.org/html/2008.09208v1): limitations of stationary constraints, motion-dependent detection and sensor/model mismatch.
