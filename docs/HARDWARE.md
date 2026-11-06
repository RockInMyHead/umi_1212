# Actual hardware: Troyka IMU 10-DOF v2

The user's module contains **LIS331DLH + I3G4250D + LIS3MDL + LPS25HB**, confirmed by [Amperka's v2 documentation](https://wiki.amperka.ru/products:troyka-imu-10-dof-v2). Use `firmware/st_imu_lora`. The previous MPU6050 example remains in `firmware/imu_lora` and [MPU6050_LEGACY.md](MPU6050_LEGACY.md); its register settings and scale factors do not apply to this board.

The controller MCU, wiring and recordings have not been supplied. The adapter targets **Arduino-ESP32 as an explicit integration assumption**, with RadioLib for the original SX1278 radio. It does not assume the sensor board is an ESP32. **No ESP32 SDK compilation or hardware run has been performed.** Host checks cover register configuration, numeric conversion, calibration fitting, timestamp/packet primitives and core behavior. They cannot establish real I2C timing, RF operation or eight-hour hardware accuracy.

## Build and configure

```sh
python3 tools/stage_arduino.py /path/to/new/troyka_navigation --sensor st
```

ST is the default. This creates a new sketch with canonical `src/navigation.h/.cpp`, the ST driver and common support headers; existing directories are never overwritten. Install the **Arduino-ESP32 board package and RadioLib**, select the real ESP32 model, configure `board_config.h`, then compile. This adapter uses checked `Wire` transactions directly; **I2Cdev and MPU6050 are not dependencies**. For another MCU, port the bus, clock, stack and radio integration. The core and ST register driver are portable C++.

Enter actual SDA/SCL, calibration button, radio SPI/NSS/reset/DIO0 pins and frequency. Mandatory values deliberately start invalid. The module's `I2C2` label does not identify a particular ESP32 hardware controller; connect its SDA/SCL to the bus chosen for the actual MCU. This adapter does not require a separate sensor data-ready wire. Check pin restrictions, supplies and pull-ups on the actual controller. The v2 module has level shifting; bare ST chips have different voltage requirements.

| Device | Default / alternate address | Required WHO_AM_I |
|---|---|---|
| LIS331DLH | `0x18 / 0x19` | `0x32` |
| I3G4250D | `0x68 / 0x69` | `0xD3` |
| LIS3MDL | `0x1C / 0x1E` | `0x3D` |
| LPS25HB | `0x5C / 0x5D` | `0xBD` |

Use the board's real solder-jumper configuration. Identity mismatches are faults, including older Troyka variants with L3G4200D/LPS331AP; they are not silently treated as equivalent chips. Configured registers are read back.

In `sensor_config.h`, measure and verify the independent `accel_to_body`, `gyro_to_body`, and `mag_to_body` transforms, then set `mounting_verified=true`. Each must be a proper rotation with determinant +1. Identity is a placeholder, not a claim about PCB chip orientation. Check ±axis acceleration, angular rate and magnetic directions physically. Navigation does not start while verification is false. A flexible or slipping hand/body mounting also changes the measurement problem.

The sketch requests a 32 KiB loop-task stack and reconstructs the global 25-state navigator in place to avoid a large stack temporary. A desktop C++17 build measured `sizeof(imu::Navigator)=15960` bytes with 8-byte `double`; this is a host ABI measurement, not an ESP32 RAM or stack certificate. The navigator, including its covariance workspaces, has static storage; other calculation arrays and dependencies still consume stack. Measure actual stack high-water mark, full firmware RAM use and worst-case execution time on the selected MCU. RadioLib's multi-argument `begin` remains in the checked upstream API; future versions may require equivalent `ConfigLoRa_t`. Pin dependencies after a successful board build and hardware qualification.

`local_gravity_m_s2` and `earth_rate_nav_rad_s` are exposed in `sensor_config.h`. The raw accelerometer's standard-g conversion stays 9.80665; local gravity is separate. Earth-rate correction defaults to zero. A nonzero vector requires a surveyed/georeferenced heading consistent with `magnetic_field_nav`; the adapter refuses nonzero Earth rate with magnetic initialization disabled. Enabling magnetism alone does not certify that geographic frame.

## ST configuration and conversion

The compact driver is independently implemented from ST register maps, without copying third-party driver code:

| Device | Configuration | Conversion |
|---|---|---|
| LIS331DLH | Normal, nominal 100 Hz, ±4g, BDU, high-pass off | Signed word /16 → left-justified 12-bit count; ×0.002×9.80665 → m/s² |
| I3G4250D | Nominal 100 Hz /25 Hz bandwidth, ±500 °/s, stream FIFO | Signed count ×0.0175×π/180 → rad/s |
| LIS3MDL | 40 Hz, ultrahigh-performance XYZ, ±4 gauss, BDU | Signed count ×100/6842 → µT |
| LPS25HB | 25 Hz, BDU, pressure512/temp64 averages, FIFO off | Pressure /4096 → hPa; temperature =42.5+signed_count/480 °C |

Sources: [LIS331DLH](https://www.st.com/resource/en/datasheet/lis331dlh.pdf), [I3G4250D](https://www.st.com/resource/en/datasheet/i3g4250d.pdf), [LIS3MDL](https://www.st.com/resource/en/datasheet/lis3mdl.pdf), [LPS25HB](https://www.st.com/resource/en/datasheet/lps25hb.pdf).

**I3G4250D has no BDU bit**: `CTRL_REG4` bit7 is reserved zero. A common L3G4200D driver assumption is wrong here. The gyro FIFO provides coherent six-byte samples. LIS331DLH ±8g mode, if selected later, uses nominal **3.9 mg per 12-bit digit**. Every range change requires matching conversion, clipping guard, calibration, noise and tests. The example's ±4g/±500°/s choice must be checked against actual handheld/body motion.

ST's gyro configuration table uses nominal 100/200/400/800 Hz, while its mechanical table lists typical actual 105/208/420/840 Hz. Measured timestamps determine integration intervals. The adapter's gyro noise density `0.0006 rad/s/√Hz` is a tuning prior of the same order as ST's 0.03°/s/√Hz figure, not measured performance. Characterize noise and bias on the actual unit.

## Freshness and timing limits

The loop attempts polling every 1 ms. It reads status/new-data bits for acceleration, magnetism and pressure and FIFO empty/count/overrun bits for gyro. Every new gyro sample is read once. More than one pending gyro sample faults this low-latency adapter; it does not assign invented times to a backlog. Acceleration overrun, bad I2C transaction/byte count, configuration mismatch, clipping, excessive IMU timing uncertainty/skew, or missing gyro data for >30 ms stops navigation. A button restart establishes a new origin epoch. Auxiliary overruns discard the observation and increment a counter.

**The four chips are not synchronized.** Each estimated timestamp is the midpoint between the last not-ready poll and first ready observation. Half this interval is reported as polling uncertainty. It includes read latency but does not identify internal conversion/filter delay or clock mismatch. At a gyro step, latest acceleration is used with permitted timestamp difference up to 15 ms. This zero-order approximation is not simultaneous sampling or exact interval-average IMU data. FIFO coherence does not synchronize conversion clocks.

Mag/baro observations are used at most once, with a documented approximation: recent observations are treated as current only within 40 ms (mag) or 80 ms (baro), adding variance from measured age and configurable angular/vertical-speed bounds. This is **not exact delayed-state replay**. Actual motion exceeding those bounds invalidates the approximation. Buffer/replay delayed measurements and characterize group delays for accurate fast motion. The core's synchronous API alone does not solve the timing problem.

LIS331DLH's 100 Hz point has an internal cutoff above Nyquist, so vibration/aliasing remains a limitation. A characterized higher-rate acquisition/decimation path is a further improvement once MCU timing and the real motion spectrum are known. Increasing algorithm complexity while dropping samples is counterproductive.

A 64-bit accumulator handles normal `micros()` wraps over eight hours. The loop must run more often than one full wrap interval; an unobserved hour-long pause cannot be reconstructed. Validate the actual clock against a stable reference when scale accuracy matters.

## Calibration and hand/body motion

Warm the module, place the **entire sensor assembly** at genuine translational and rotational rest and press the calibration button once. Collection requires ten seconds and enough fresh gyro/pressure samples. Magnitude and variation checks restart collection after common movement. A held button does not repeat calibration. A new press deliberately resets the origin.

The button is the operator's declaration of rest: quiet IMU signals cannot prove zero translational velocity. `VERIFIED_REST_PIN=-1` disables continuous stationary updates by default. **Foot stance, gait phase, ground contact and low IMU variance do not establish that a hand-held or torso-mounted sensor is stationary.** Never apply foot-mounted ZUPT logic to a hand or torso. Enable it only with independent rest truth for the sensor itself. One initial pose does not estimate accelerometer bias.

Fit measured acceleration/gyro calibration with:

```sh
python3 tools/fit_calibration.py six_faces.csv \
  --gyro-temperature gyro_temperature.csv \
  --report calibration_report.json --header calibration_config.h
```

CSV: `face,ax,ay,az,gx,gy,gz,temp_c,stationary`, using m/s², rad/s, `+x,-x,+y,-y,+z,-z`, and independently established `stationary=1`. For the ST adapter **apply mounting transforms before fitting**; the core calibration is in the common body frame. At least 200 fresh readings per accurately aligned face at stable temperature are required. Screening rejects missing faces, motion, implausible scales and excessive residuals. It fits diagonal accelerometer gain/offset, not cross-axis or acceleration temperature behavior. Optional gyro thermal fitting needs ≥600 fixed-orientation readings with coverage across a ≥10°C sweep. Validate separate poses, heating/cooling cycles and held-out sessions.

The ST adapter uses **LPS25HB chip temperature as a shared board-temperature proxy** for the core's thermal correction. Fit using that same proxy, mounting and thermal path. It is neither gyro die temperature nor ambient air. I3G4250D's relative temperature count has no invented Celsius offset; the driver retains its raw value. Fast thermal differences between chips can invalidate a proxy model. Identity calibration is supplied by default and flags `Uncalibrated`.

Core calibration is `corrected=M*(raw-offset-slope*(T-Tref))`. Residual screening does not certify absolute accuracy. Demanding results require full matrix/gyro scale characterization, thermal hysteresis, correlated noise and mounting repeatability. Preserve the original fresh timestamped data for replay.

## Magnetic calibration before heading aid

Magnetic fusion starts **disabled**. Rotate the assembled device in a clean constant field and record uncalibrated chip-frame LIS3MDL µT. The NumPy fitter requires ≥600 readings with full 3D coverage:

```sh
python3 tools/fit_magnetometer.py magnetic_rotations.csv \
  --field-uT MEASURED_LOCAL_MAGNITUDE \
  --report magnetic_fit.json --header magnetometer_calibration.h
```

CSV columns: `mx_uT,my_uT,mz_uT`. Supply measured/local-model field magnitude; no universal value is invented. The ellipsoid fit supplies hard-iron offset and symmetric full soft-iron correction, screens rank/conditioning/SPD, all eight octants and every-fifth-row holdout radial residuals. An interleaved holdout is not independent-session validation. Apply its calibration in **chip coordinates before** `mag_to_body`.

**Magnitude fitting cannot determine mounting rotation, magnetic declination or geographic heading.** Independently establish alignment and a reference vector in the selected navigation frame. Then set `magnetometer_enabled=true` and `magnetic_field_nav`. Startup uses the calibrated field for yaw only when checks pass. Without it, initial yaw defines an arbitrary local heading. The core additionally estimates residual magnetic bias; that does not replace calibration or solve arbitrary magnetic interference.

Norm and innovation gates reject some disturbances. Steel, magnets, changing current and local field gradients can still cause coherent wrong heading with an apparently normal field norm. Test with RF/peripherals operating and compare independent heading truth. Magnetic aiding provides **no horizontal position reference**.

## Pressure height and weather

Barometric aiding starts after an averaged initial pressure reference is acquired at declared rest. The hypsometric approximation is

```
h=(287.05*mean_air_temperature_K/local_gravity)*log(p_reference/p)
```

Default 288.15 K is an explicit standard-atmosphere assumption. Replace it with an appropriate air-column temperature model; self-heated board temperature is unsuitable. This yields approximate relative height, not surveyed absolute altitude.

The core's barometer bias state admits pressure-drift uncertainty. Height and weather bias are still not independently observable from pressure alone. Eight-hour weather, airflow, HVAC, doors and enclosure effects can dominate sensor noise. A stationary reference barometer, independent altitude fixes or known-height constraints improve observability. Pressure does not constrain horizontal position.

## Radio and version-3 protocol

RadioLib `startTransmit`, a DIO0 sent callback and `finishTransmit` avoid waiting for RF completion. A 2-second timeout disables RF while sampling continues. Example settings: SF7, 125 kHz, CR4/5, 10 dBm, preamble8, sync `0x12`, CRC, 10-second telemetry. Set the module's real frequency/antenna and matching receiver; the interval is not a determination of allowed airtime. Serial is off by default; whole packets are queued only with enough free buffer space. Short SPI/I2C operations still need hardware timing validation.

ST uses **148-byte version 3** with unsigned little-endian integers and IEEE754 binary32 physical fields. `tools/decode_telemetry.py` handles v3 and legacy v2. Bytes0–107 retain v2's fields; the extension replaces v2's CRC.

| Offset | Bytes | Field |
|---:|---:|---|
| 0 | 2 | ASCII `IN` |
| 2,3 | 1 each | Version3; length148 |
| 4,8 | 4 each | Packet sequence; origin epoch |
| 12 | 8 | Last accepted filter time, µs since boot |
| 20,24 | 4 each | Status; fault mask |
| 28,40 | 12 each | Position xyz m; velocity xyz m/s |
| 52,64 | 12 each | Position/velocity diagonal variances |
| 76 | 16 | Quaternion wxyz, body→local |
| 92 | 4 | Barometer-chip temperature proxy °C |
| 96,100,104 | 4 each | Counted lost cycles; gyro samples; max compute µs |
| 108,112,116,120 | 4 each | Accel/gyro/mag/pressure age at encoding, µs; UINT32_MAX if unavailable |
| 124,128 | 4 each | Accel/gyro polling time uncertainty, µs |
| 132 | 4 | Latest pressure hPa |
| 136 | 4 | Barometer bias variance m² |
| 140 | 4 | Auxiliary overrun/invalid-reading count |
| 144 | 4 | CRC32/ISO-HDLC of bytes0–143 |

Status: `1 Initialized`, `2 Quiet`, `4 VerifiedStationary`, `8 Calibrating`, `16 AcquisitionStopped`, `32 RadioFailed`, `64 EncodingOverflow`, `128 WaitingForCalibration`, `256 Uncalibrated`, `512 PolledTimingApproximation`, `1024 MagneticAidingConfigured`, `2048 BarometricAidingConfigured`. Configured does not mean every observation was accepted. Quiet never means rest truth. Stopped filter time freezes; receivers must reject it as a current position.

Core faults are in `src/navigation.h`; adapter bits are `1<<16 BusFault`, `1<<17 SamplingFault`, `1<<18 ConfigurationFault`, `1<<19 RadioFault`, `1<<20 Overrun`, `1<<21 Clipping`. Parse version/length/CRC/sequence/epoch/freshness explicitly. The old receiver payload is incompatible. Max compute time excludes full bus/RF execution. Covariance is conditional on modeling assumptions, not an error guarantee.

## Hardware qualification still required

Build with the real MCU SDK; inspect warnings, register readback, output rates and axes. Test freshness, FIFO order, bus faults, backlog, clipping, missing sensors, radio timeout, button debounce, clock wraps, stack margin and full-peripheral CPU load. Use a logic analyzer and timing GPIO. Fake-bus tests do not model electrical behavior or conversion clocks.

Record representative eight-hour hand/body trajectories with synchronized independent position, velocity, heading and height truth. Include stops, walking, turns, mounting movement, temperature, magnetic disturbance and pressure changes. Report endpoint and trajectory RMS/95th/max errors, outages and covariance consistency across repeated runs. See [VALIDATION_PLAN.md](VALIDATION_PLAN.md) and [REPLAY.md](REPLAY.md). Compass/barometer offer orientation/vertical constraints; **bounded horizontal error still needs independent horizontal observations or justified motion constraints**.

API references: [RadioLib SX1278](https://github.com/jgromes/RadioLib/blob/master/src/modules/SX127x/SX1278.h), [async example](https://github.com/jgromes/RadioLib/blob/master/examples/SX127x/SX127x_Transmit_Interrupt/SX127x_Transmit_Interrupt.ino), [ESP32 stack setup](https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/ArduinoStackSize/ArduinoStackSize.ino). Dependency versions remain unpinned until a real board/toolchain build is qualified.
