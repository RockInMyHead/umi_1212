# Replaying recorded measurements

Build with a C++17 compiler from the project root:

```sh
c++ -O3 -std=c++17 -Isrc tools/replay.cpp src/navigation.cpp -o replay
./replay initial_state.csv samples.csv estimates.csv
```

The utility deliberately requires an initial state. Estimate it from a verified stationary calibration interval and an independent heading reference if the navigation frame is ENU. A gravity-only initialization has arbitrary yaw. Do not initialize every moving dataset with identity attitude and zero velocity. Inputs must already have per-unit scale, offset and temperature calibration applied, in m/s² and rad/s; the replay configuration's calibration is identity. Bias columns are remaining residual biases, not offsets already removed. The replay uses the core's documented noise priors; change `Config` in the utility for characterized hardware. No real sensor dataset is bundled.

`initial_state.csv` contains a header and one row, with 23 columns in this order:

```
time_s,qw,qx,qy,qz,px,py,pz,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz,attitude_valid,origin_valid,gravity,earth_x,earth_y,earth_z
0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,9.80665,0,0,0
```

`attitude_valid` and `origin_valid` must be 1: they declare the initialization is intentional. These flags do not verify its accuracy. The example is a known level stationary initial state in a nonrotating local model. For Earth corrections, supply the Earth vector in the actual navigation frame. Prior attitude uncertainty remains configured separately.

For the ST 10-DOF module, append `magnetic_field_x,magnetic_field_y,magnetic_field_z` to the initial-state header and the three calibrated local field components in microtesla to its row (26 columns total). Zero/default field disables magnetic aiding. A nonzero field requires the correct navigation-frame orientation, field magnitude and inclination; it is not obtained from the nominal sensor specification.

`samples.csv` must use this exact header (Unix newlines, no quoting):

```
time_s,ax,ay,az,gx,gy,gz,temperature_c,verified_stationary,saturated,px,py,pz,position_sigma,vx,vy,vz,velocity_sigma
0.01,0,0,9.80665,0,0,0,25,0,0,,,,,,,,
0.02,0,0,9.80665,0,0,0,25,0,0,0,0,0,0.2,,,,
```

Samples represent the preceding interval; timestamps must increase and gaps must not exceed `Config.max_dt`. Optional position and velocity groups are each either four empty fields or a complete measurement and positive one-axis standard deviation in metres or m/s. The utility squares sigma to obtain variance. Measurements must be synchronous, in the same frame, with lever arms already accounted for. Align/replay asynchronous measurements to their true times first; do not relabel delayed fixes. Flags are exactly 0/1. `verified_stationary` requires independent evidence of translational AND rotational rest.

For magnetic and barometric observations, append `mx,my,mz,mag_sigma,baro_altitude,baro_sigma` to the sample header and six fields to every row (24 total). A magnetic observation is four complete values in microtesla / microtesla standard deviation, or four empty fields. It must already be hard/soft-iron calibrated and aligned to the IMU body axes. A barometric observation is two complete values (relative altitude and standard deviation in metres), or two empty fields. The core estimates a residual barometer bias; pressure weather changes are not independently observable as altitude. Sample pressure using a documented origin and conversion, and do not confuse the barometer's package temperature with ambient air-column temperature.

Invalid IMU samples stop replay with a nonzero exit code. Gated aiding outliers increment the rejection counter and remain visible in output. Output uncertainty is model-dependent, not a hardware-certified error bound. To evaluate real accuracy, compare the estimates with an independent reference trajectory that was not fed into the filter.
