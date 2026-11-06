# Mathematical model and physical limits

This implementation estimates local position, velocity, attitude, residual inertial and magnetic sensor biases, six residual magnetic scale/cross-axis terms and a pressure-altitude offset. It is a 25-state error-state Kalman filter (ESKF), with a unit quaternion for nominal attitude. The user's module contains LIS331DLH acceleration, I3G4250D angular-rate, LIS3MDL magnetic-field and LPS25HB pressure sensors. It is not an eight-hour accuracy guarantee, an absolute-position sensor, or a proof of globally minimum estimation error. Its covariance describes the chosen stochastic model; unmodeled calibration, temperature, timing, vibration, magnetic, pressure and motion errors can exceed that covariance.

## 1. What can and cannot be observed

An accelerometer measures **specific force**, not velocity or position. Ignoring Earth rotation and spatial gravity variation, two trajectories with the same attitude and acceleration but different initial position or constant velocity produce identical IMU signals. No processing of those signals can distinguish those trajectories. Gravity supplies a vertical reference but no absolute yaw reference. Unknown bias and unknown motion can also imitate one another.

Consequently, eight hours of arbitrary motion cannot have a guaranteed small position error with this module alone. A calibrated magnetometer in a known, stable field can supply heading information, and pressure can constrain relative height. Neither measures horizontal position or constant horizontal velocity. Trusted stops, external position/velocity observations or valid application-specific motion constraints are necessary to control the corresponding errors. Each such input must actually be available to the deployed estimator.

Low IMU variance means **quiet**, not necessarily stationary: a smoothly moving vehicle can be quiet. `predict(sample, true)` asserts independently verified **translational and rotational** rest. The default argument is false. Do not set it merely because acceleration is near gravity, the gyroscope is quiet, or the application wants a stable display. Internal quiet detection is an additional consistency check, not evidence of zero velocity by itself.

For the user's handheld/body-mounted placement, a foot touching the ground does **not** establish this condition. The hand and torso continue translating and rotating through ordinary gait. Step events must never be passed as `verified_stationary=true`; even a shoe-stance model would need different assumptions from the combined rigid-rest update used here.

Even verified zero velocity does not measure absolute position or yaw. Observability of residual biases depends on motion and excitation. A single fixed stationary pose cannot uniquely separate horizontal accelerometer bias from small tilt error. Initialization and covariance must retain that uncertainty.

## 2. Conventions and units

- Quaternion `q=(w,x,y,z)` uses Hamilton multiplication, scalar first.
- `R(q)` maps body-frame vectors into a fixed local navigation frame with positive Z upward.
- Position is in metres, velocity in m/s, accelerometer specific force in m/s², gyroscope rate in rad/s, time in seconds and temperature in °C.
- `g_n=(0,0,-g)`, with configured positive `g`. This is effective local gravity.
- Biases are expressed in the calibrated body frame.
- Error state order is `δx=[δp, δv, δθ, δb_a, δb_g, δb_h, δb_m, δs_m]`; index 15 is the scalar barometric altitude offset in metres, indices 16–18 are residual body-frame magnetic bias in µT, and indices 19–24 are dimensionless symmetric residual magnetic coefficients `xx, yy, zz, xy, xz, yz`.
- True state equals nominal plus error for vector states; `q_true=q⊗Exp(δθ)` for orientation. Angular error is therefore a **right/local** perturbation.
- `skew(u)v=u×v`.

For a rotation vector `φ`, with `a=||φ||`,

```
Exp(φ) = [cos(a/2), sin(a/2) φ/a].
```

The continuous small-angle limit is used at zero. This exponential preserves finite rotation geometry; normalization controls numerical round-off. It does not remove sensor bias.

For these conventions, expected stationary specific force in body coordinates is

```
R(q)^T [0,0,g]
 = g [2(xz-wy), 2(yz+wx), w²-x²-y²+z²].
```

The first two signs matter. Using the third **column** of `R` here instead of its third **row** gives the wrong gravity direction at nonzero tilt.

## 3. Calibration and temperature compensation

Before estimation, each raw physical-unit sensor vector is corrected as

```
a_c = M_a {a_raw - a_offset - a_temp_slope (T-T_ref)}
w_c = M_g {w_raw - w_offset - w_temp_slope (T-T_ref)}.
f   = a_c - b_a
w   = w_c - b_g.
```

The filter biases `b_a,b_g` represent **residual** error after this calibration. They are not a second copy of the full offset. A temperature model measured on one unit must not be silently transferred to another. A linear temperature model is an engineering approximation; hysteresis, warm-up, mounting stress and nonlinear temperature response remain possible.

Opposing, known ±axis gravity orientations support bias and diagonal scale calibration. A complete cross-axis matrix requires sufficient reference orientations and attention to the rotational ambiguity of magnitude-only calibration. One stationary pose does not provide a complete accelerometer calibration. Gyroscope scale and misalignment require known angular motion, not only a zero-rate average.

The actual module's component specifications must replace the original repository's MPU6050 assumptions:

| Sensor | Manufacturer characterization | Consequence for the model |
|---|---|---|
| LIS331DLH | Typical acceleration density 218 µg/√Hz at ±2 g; sensitivity is specified using a 12-bit representation | Convert the register format correctly; characterize installed bias and scale. [ST datasheet](https://www.st.com/resource/en/datasheet/lis331dlh.pdf) |
| I3G4250D | Typical rate density 0.03 °/s/√Hz at 50 Hz bandwidth | Do not retain the old MPU6050 gyro-noise assumption. [ST datasheet](https://www.st.com/resource/en/datasheet/i3g4250d.pdf) |
| LIS3MDL | Typical RMS magnetic noise 3.2, 3.2, 4.1 mgauss for X/Y/Z at ±12 gauss, ultrahigh-performance mode | These are 0.32, 0.32, 0.41 µT RMS under those conditions, not a spectral density or heading-accuracy guarantee. [ST datasheet](https://www.st.com/resource/en/datasheet/lis3mdl.pdf) |
| LPS25HB | Pressure noise 0.01 hPa RMS with embedded filtering, 0.03 hPa without; characterization under controlled conditions | Noise, absolute pressure accuracy and weather-induced apparent height are different errors. Filtering introduces temporal correlation. [ST datasheet](https://www.st.com/resource/en/datasheet/lps25hb.pdf) |

These values are characterization priors rather than measured residual errors of the assembled module. They cannot predict an individual unit's eight-hour navigation accuracy. Magnetometer hard-iron offset, soft-iron distortion and alignment to the IMU require separate calibration over sufficiently varied orientations. One stationary pose cannot identify that calibration and absolute heading simultaneously.

## 4. Nominal mechanization

With optional constant Earth angular velocity `Ω_n` expressed in the navigation frame,

```
p_dot = v
v_dot = R f + g_n - 2 Ω_n × v
R_dot = R skew(w) - skew(Ω_n) R
b_a_dot = 0
b_g_dot = 0.
```

The nominal barometric altitude offset, residual magnetic bias and magnetic scale terms also have zero modeled derivatives: `b_h_dot=0`, `b_m_dot=0`, `s_m_dot=0`; their uncertainties are driven by configurable random walks.

For a genuinely ENU frame at latitude `L`, `Ω_n=[0, Ω cos(L), Ω sin(L)]`, with Earth rotation magnitude approximately `7.292115e-5 rad/s`. In a frame with arbitrary startup heading, rotate this vector into that frame; merely supplying latitude does not establish the missing heading. Zero configuration disables Earth correction.

The finite rotation step for constant rates is

```
q_next = Exp(-Ω_n dt) ⊗ q ⊗ Exp(w dt).
```

The force is rotated using midpoint orientation. This reduces the timing error from rotating an entire interval's force with its endpoint attitude. Samples are treated as interval averages for `(previous_time, sample.time_s]`. They must not be arbitrary latest readings repeated across missing intervals.

The model is **local**, with constant gravity and Earth rate. It omits navigation-frame transport rate, Earth curvature, gravity gradients and changing latitude/altitude. It is appropriate only where those approximations are acceptable relative to the error budget. Long travel requires a geodetic mechanization, not simply a longer run of this local model. There is no full higher-order coning/sculling compensation; interval averaging and midpoint rotation do not reconstruct unresolved vibration or aliased dynamics.

## 5. Error dynamics and signs

The following Jacobians follow by substituting `R_true=R(I+skew(δθ))` and retaining first-order terms:

```
δp_dot  = δv
δv_dot  = -2 skew(Ω_n) δv - R skew(f) δθ - R δb_a - R n_a
δθ_dot  = -skew(w) δθ - δb_g - n_g
δb_a_dot = n_ba
δb_g_dot = n_bg.
```

The sixteenth error state is `δb_h_dot=n_bh`. Indices 16–18 obey `δb_m_dot=n_bm`; indices 19–24 obey `δs_m_dot=n_sm`. They have no inertial dynamics coupling and receive direct process-noise variance increments `s_bh² dt`, `s_bm² dt I_3` and `s_sm² dt I_6`, respectively.

Thus the nonzero continuous system blocks are

```
F[p,v]       = I
F[v,v]       = -2 skew(Ω_n)
F[v,theta]   = -R skew(f)
F[v,ba]      = -R
F[theta,theta] = -skew(w)
F[theta,bg]  = -I.
```

The noise-input blocks are `G[v,na]=-R`, `G[theta,ng]=-I`, `G[ba,nba]=I`, and `G[bg,nbg]=I`.

Notice that `w` in `F[theta,theta]` is the bias-corrected measured inertial angular rate. It is **not** `w-R^T Ω_n`. In a right-error derivation the navigation-frame rotation term cancels from this block. Reusing a left-error Jacobian with a right-error quaternion injection would be inconsistent.

Propagation uses a sparse approximation to `Phi=exp(F dt)`, with leading position/attitude and position/accelerometer-bias coupling retained. The frozen-rate attitude blocks are integrated exactly:

```
Phi[theta,theta] = Exp_SO3(-w dt)
Phi[theta,bg]    = -J_r(w dt) dt.
```

This matters over long rotating trajectories: a first-order block `I-skew(w)dt` is not orthogonal and artificially increases rotating-plane uncertainty at every step. The exact block avoids that artifact. The complete coupled transition is still not an exact matrix exponential. Validate sample-rate sensitivity and high-rate maneuvers; do not assume equivalence at large `dt`.

## 6. Process-noise convention

The configuration uses **continuous diffusion amplitude** `s`, defined by

```
E[n(t)n(t')^T] = s² I delta(t-t').
```

An independent interval-average white-noise sample therefore has standard deviation `s/sqrt(dt)`. A bias random-walk increment has standard deviation `s_bias sqrt(dt)`. These are different quantities with different units.

One-sided spectral amplitude density in a physical datasheet is not automatically the same convention: an ideal white spectrum with one-sided density `N` has diffusion intensity `N²/2`. Real digital filters also introduce bandwidth limits and temporal correlation. The defaults are engineering priors, not a literal sensor-characterization result. The simulator must declare the convention it uses, and a real unit should be characterized at the actual sample rate and filter configuration.

The position/velocity process noise includes cross covariance. For continuous white acceleration of intensity `q_a`, the implemented isolated pair has blocks `Q_pp=q_a dt³/3`, `Q_pv=q_a dt²/2`, `Q_vv=q_a dt`. Gyro noise adds `q_g dt I` to the attitude block, and bias walks add their respective intensity times `dt`. These noise blocks are positive semidefinite; the complete construction approximates full coupled continuous-noise integration. Higher-order gyro/bias-to-position noise terms are omitted.

For comparison, holding an independent interval-average acceleration error constant throughout each interval would give `Q_pp=q_a dt³/4`. A simulator using only such samples therefore omits unresolved within-interval position noise represented by the continuous model. That small discrepancy must not be confused with a sensor performance guarantee.

## 7. Measurements and correction

An external position or velocity observation uses its actual measurement variance in the same frame and at the current state time. Position has `H[p]=I`; velocity has `H[v]=I`. The current interface accepts diagonal covariance and synchronous measurements. Delayed measurements need buffering/replay outside the core; changing their timestamp to the current time is incorrect.

At independently verified rigid rest:

```
zero velocity: z_v=0,                h_v=v
specific force: z_a=a_c,             h_a=b_a-R^T g_n
angular rate: z_w=w_c,               h_w=b_g+R^T Ω_n.
```

Their nonzero Jacobians are

```
H_v[v]     = I
H_a[theta] = skew(-R^T g_n),  H_a[ba] = I
H_w[theta] = skew( R^T Ω_n),  H_w[bg] = I.
```

Both attitude signs are **positive**. For example, `(R Exp(δθ))^T u ≈ R^T u + skew(R^T u)δθ`.

The three-dimensional innovation is `r=z-h`, covariance `S=HPH^T+V`, gain `K=PH^T S^-1`. Gating compares `r^T S^-1 r` with the configured threshold. A numerical threshold alone is not a measured false-rejection probability unless the Gaussian, independent and correctly calibrated covariance assumptions hold. Badly underestimated covariance can reject valid recovery measurements.

The covariance correction uses an algebraic expansion of the Joseph form

```
P_corrected = (I-KH) P (I-KH)^T + K V K^T.
```

Sequential scalar updates are equivalent to the joint linearized update because the supplied observation covariance is diagonal; the joint innovation is gated before any correction. Floating-point subtraction in the expanded covariance expression can still lose precision. Runtime finiteness/diagonal checks are not a proof of full positive semidefiniteness; offline covariance checks remain appropriate.

Then `δx=Kr` is injected into the nominal state, including `q←q⊗Exp(δθ)`. Resetting the orientation error changes its tangent frame, so covariance is reset with `P←J P_corrected J^T`. The orientation reset block is the SO(3) right Jacobian

```
J_r(φ) = I - (1-cos(a))/a² skew(φ)
           + (a-sin(a))/a³ skew(φ)²,
```

whose small-angle expansion is `I-skew(φ)/2+skew(φ)²/6`. The other reset blocks are identities. These conventions agree with the local-error formulation in [Solà, Quaternion kinematics for the error-state Kalman filter, sections 5–6](https://arxiv.org/pdf/1711.02508).

Static accelerometer/gyroscope corrections reuse sensors that also drive propagation. Treating their measurement noise as independent of process noise is an approximation. Decimation and conservative variance help, but do not mathematically remove that correlation. A stringent covariance-consistency requirement needs a correlated-noise treatment or independent disjoint averaged measurements. Do not interpret small stationary covariance as a certified bound.

### Magnetic field observation

The caller supplies a calibrated magnetic vector in µT, aligned with the accelerometer/gyro body frame. The small residual symmetric matrix `S` contains coefficients `xx, yy, zz, xy, xz, yz`. A known navigation-frame reference `B_n` predicts

```
z_m = calibrated magnetic field
u = R^T B_n; M = I + S
h_m = M u + b_m
H_m[theta] = +M skew(u).
H_m[magnetic_bias] = I.
```

The six scale/cross-axis Jacobian columns, in the declared order, are

```
(ux,0,0), (0,uy,0), (0,0,uz),
(uy,ux,0), (uz,0,ux), (0,uz,uy).
```

The positive attitude sign follows the same right-perturbation derivative as gravity, followed by multiplication by the residual scale matrix. Input variances are in µT². The vector `b_m` models persistent residual hard-iron/thermal offset and `S` models persistent orientation-dependent scale/cross-axis residuals after prior calibration. Treating those errors as fresh independent white noise would incorrectly allow repeated observations to average them away. A zero reference disables meaningful magnetic fusion; inventing a north-pointing field does not establish true north. A surveyed field or a valid geomagnetic reference, including declination and inclination, must be expressed in the same navigation frame. A field parallel to gravity gives no independent yaw information.

A magnitude consistency gate, applied after subtracting estimated residual bias, followed by joint three-dimensional innovation gating rejects many disturbances. A field rotated by interference can retain the correct magnitude. If attitude uncertainty is large, even innovation gating may accept it; field magnitude and innovation checks are not proof of magnetic validity. Persistent interference can bias heading and position. Do not continuously fit the navigation-frame reference from the same measurements while claiming it remains an independent heading reference.

At one fixed pose the linear perturbation `δb_m=-M skew(u)δθ-δS u` leaves the observation unchanged. Residual magnetic bias, scale and attitude are not separately observable in that situation. Diverse rotations and independent attitude/motion information can help separate them; a single pose cannot establish an unknown calibration and true heading simultaneously. The filter must preserve their cross covariance and ambiguous uncertainty instead of reporting a precise heading merely after many repeated readings.

An analytic special case makes this explicit. Hold tilt and residual scale known, use horizontal field magnitude `B_H`, yaw prior variance `P_psi`, isotropic residual-bias variance `P_b`, and `n` independent magnetic observations with white variance `R_m`. The fixed-pose linear yaw posterior is

```
P_psi_post = P_psi (P_b + R_m/n) / (B_H² P_psi + P_b + R_m/n).
```

Its limit is nonzero when residual bias is uncertain. The independent physics tests verify that limit and the corresponding yaw/bias cross covariance. With `B_H=20 µT`, yaw prior 0.1 rad and bias prior 1 µT, the limiting yaw standard deviation is about 0.0447 rad, not zero.

Scale uncertainty produces another persistent ambiguity. With known residual bias, known tilt and `u=(20,0,-40) µT`, the terms `S_xy` and `S_yz` can imitate a yaw-induced transverse field error. Independent equal scale priors of variance `P_s` contribute transverse variance `P_s(20²+40²)`. Substituting that variance for `P_b` in the special-case expression gives the fixed-pose yaw limit. The physics tests verify this result, the entirely unobserved `S_yy` coefficient when `u_y=0`, all six Jacobian columns and their implementation axis/sign/gain.

The configured calibration priors and random walks are engineering assumptions requiring per-unit characterization. A 1 µT bias prior, 0.002 µT/√s bias walk, 0.005 scale prior and `1e-6/sqrt(s)` scale walk do not imply that every disturbed field obeys this model. The scale prior represents 0.5% coefficient uncertainty; its walk adds about 0.017% one-sigma coefficient diffusion over eight hours. These are not manufacturer navigation guarantees.

The symmetric residual model captures constant soft-iron gain/cross-axis errors in the assumed aligned body frame. It does not estimate an arbitrary residual rotation between magnetometer and IMU axes, nor an unknown time/spatially varying navigation-frame magnetic field. Large or fast distortions can exceed the model. Runtime health checks require `I+S` to remain positive definite and flag an invalid estimate rather than silently clipping coefficients without handling covariance. Sensitivity/ablation tests must distinguish fixed calibration residuals, thermal changes, soft iron and transient interference. Augmentation improves the model of persistent uncertainty; it does not certify covariance coverage or absolute accuracy for all conditions.

### Barometric altitude and its offset

Pressure is converted upstream into altitude relative to a declared pressure origin. Its measurement model is

```
z_h = relative pressure altitude, metres
h_h = p_z + b_h
H_h[p_z] = 1; H_h[b_h] = 1.
```

The scalar observation uses a positive variance in m² and scalar innovation gating. The offset state represents pressure-origin/calibration error and slowly changing apparent altitude, not a directly measured atmospheric correction. The sign is positive: a positive pressure-altitude error is modeled as positive `b_h`.

The transformation `p_z←p_z+c, b_h←b_h-c` leaves this observation unchanged. Barometry alone therefore observes their sum, not each separately. Initial origin information and subsequent independently known height or motion constraints determine their separation. A small posterior variance obtained from an unjustified initial height prior is not evidence of absolute altitude accuracy.

Under a local isothermal atmosphere, hydrostatic balance and the ideal gas law give `Δh=-(R_air T/g) ln(p/p_ref)`. Near sea-level standard conditions, the local sensitivity is about 8.3 metres per hPa: a weather change of 1 hPa can imitate roughly 8.3 metres of height without any motion. Standard-atmosphere formulas are another approximation, not a way to remove weather. [NOAA describes continuously varying atmospheric pressure](https://www.noaa.gov/jetstream/atmosphere/air-pressure), and its [pressure-altitude formula](https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf) assumes a standard reference atmosphere.

Pressure variance must be converted through the chosen altitude formula's derivative. Uncertainty in the shared reference pressure is persistent and correlated, so it belongs in origin/bias uncertainty rather than being treated as fresh independent sensor noise on every sample. Airflow over the pressure port, enclosure pressure, HVAC changes and temperature-profile errors also matter.

The configured barometric random walk is an engineering model, not an eight-hour weather bound. For example, `s_bh=0.02 m/sqrt(s)` implies only `s_bh sqrt(28800)≈3.39 m` one-sigma diffusion over eight hours; real atmospheric changes can produce much larger apparent-height changes. Increasing this noise allows uncertainty to grow but cannot make weather and true slow vertical movement independently observable. Without a reference barometer, known altitude or suitable external height observations, long-duration absolute altitude remains uncertain.

The independent physics test constructs two worlds with the same local-model sensor stream for eight hours: stationary height plus 30 m of apparent pressure-height drift, or a genuine 30 m climb at constant 1.04 mm/s and no pressure drift. Their initial vertical velocities differ within an unknown initial-velocity prior. With no verified rest/height observations, one estimator sees the same input in both cases and must give the same answer. At least one final height error is therefore 15 m or larger. This is an identifiability counterexample, not a claim that every run suffers that error.

## 8. Why filtering alone cannot solve eight-hour drift

In the implemented flat local model, a constant uncorrected acceleration error `b` gives `position_error=b t²/2`. At eight hours, an error of only one micro-g gives approximately 4.07 km in this simplified thought experiment. This is a sensitivity calculation, not a prediction of a particular sensor, and does not include spherical-Earth inertial error dynamics.

For a white-acceleration model with intensity `q`, known attitude and no aiding, single-axis position variance is `q t³/3`. Bias errors and attitude-to-gravity leakage add other terms. Low-pass filtering can suppress high-frequency noise while preserving an unknown DC bias; it cannot infer the missing absolute reference.

ZUPTs can correct correlated position, velocity and attitude errors more usefully than simply replacing velocity with zero, but false ZUPTs discard real travel. Residual position and yaw error remain. See [Wahlström and Skog, Fifteen Years of Progress at Zero Velocity](https://arxiv.org/html/2008.09208v1) for observability and motion-model limitations. A foot during gait is not automatically at rigid rotational rest; do not use the combined static update as a generic shoe-stance detector.

The strongest honest result is a reproducible improvement under explicit sensor, motion and aiding assumptions, followed by an instrumented eight-hour hardware test. Global optimality or an absolute smallest real-world error is not established by this implementation.

## 9. Handheld pedestrian alternatives

A separate step-based pedestrian estimator can replace unrestricted horizontal double integration with `Δp_step=length_step × walking_direction`. That uses a gait model in addition to sensor data. It requires reliable step detection, calibrated step length and an estimate of the pedestrian's travel direction. Magnetic device yaw alone is not walking direction: a person can turn the device in their hand, walk sideways or backward, climb stairs, or step in place.

Published smartphone PDR work explicitly handles carrying/motion modes and heading misalignment; it does not establish that one fixed offset works for arbitrary handheld motion. See [Pedestrian Dead Reckoning Based on Motion Mode Recognition Using a Smartphone](https://pmc.ncbi.nlm.nih.gov/articles/PMC6021937/) and [Carrying Position Independent User Heading Estimation](https://www.mdpi.com/1424-8220/16/5/677). A torso mount with a stable known alignment simplifies that problem but does not eliminate gait and heading errors.

Such a mode should be explicit and calibrated from representative recorded walking, not silently selected whenever inertial position drifts. It trades inertial integration errors for step-length, missed/false-step and heading errors. For scale, a constant 2% length error over 20 km gives 400 m of distance error; a constant 1° heading error over a straight 20 km route gives about 349 m of cross-track error. These are illustrative geometric calculations, not measured performance. An eight-hour horizontal bound still needs independently informative observations or validated constraints.
