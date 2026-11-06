> Legacy example only. The user's actual board is Troyka IMU 10-DOF v2; use HARDWARE.md and firmware/st_imu_lora for that module.

# Legacy MPU6050 integration and real-device calibration

`firmware/imu_lora` is an ESP32 + MPU6050 + SX1278 integration example using the new `src/navigation.*` API. It replaces the inconsistent historical sender at the application boundary. The board, pin map, sensor recordings, RF receiver, antenna, and reference trajectory were not available during development. **This adapter has not been compiled with an ESP32 board SDK or tested on hardware.** Host checks cover its portable timestamp, encoding and calibration primitives only. An eight-hour synthetic benchmark cannot qualify this hardware.

## Build a concrete sketch

From the repository root:

```sh
python3 tools/stage_arduino.py /path/to/new/imu_navigation --sensor mpu6050
```

The staging tool copies the canonical `src/navigation.h/.cpp`, the adapter and support headers into a new sketch directory. It refuses to overwrite an existing directory. Edit the generated `board_config.h` with the actual SDA/SCL, MPU interrupt, button, SPI, SX1278 NSS/reset/DIO0 pins, I2C address and RF frequency. All mandatory pins start at `-1`, deliberately producing a compilation error until configured. Do not copy a pin assignment from a different ESP32 board: avoid flash, PSRAM, boot-strapping, input-only and shared pins as appropriate for that specific board. Connect MPU INT to the configured interrupt-capable input; a polled loop without that connection is insufficient.

Install the Arduino ESP32 board package, Jeff Rowberg's **I2Cdev + MPU6050** libraries and **RadioLib**. Select the real ESP32 board and compile/upload the staged sketch. The RadioLib legacy multi-argument `begin` call is still present in the verified upstream header and supports common existing installations; a future release may require its `ConfigLoRa_t` equivalent. The sketch requests a 32 KiB Arduino loop-task stack and reconstructs the global navigator in place to avoid a large temporary on that stack. Measure the actual stack high-water mark and execution time on the target; this allocation is not proof that deadlines can be met. Keep the core source snapshots synchronized by staging again after core changes.

SX1278 is not a generic 868/915 MHz radio; use the range, antenna, matching network, frequency, power and airtime constraints of the actual module and deployment. The example selects SF7, 125 kHz bandwidth, CR4/5, 10 dBm, preamble 8, sync word `0x12` and CRC. Its default telemetry interval is 10 seconds. The receiver must use the identical RF settings and the version-2 protocol below. This interval is an engineering starting point, not a determination of permitted channel usage.

## Sampling and fault behavior

The MPU is explicitly configured and read back: PLL X gyro clock, 200 Hz output (`SMPLRT_DIV=4` with enabled DLPF), gyro 42 Hz / accelerometer 44 Hz DLPF, ±8 g, ±500 degree/s, temperature enabled. Conversion is 4096 LSB/g, 65.5 LSB/(degree/s), and `raw_temperature / 340 + 36.53` °C. Raw acceleration uses standard gravity 9.80665 m/s²; local gravity belongs in the navigation model and calibration reference. Change ranges only together with conversion constants, saturation thresholds, model noise and validation data. Lower ranges trade dynamic headroom for quantization; choose them from measured motion, not an assumed universal optimum.

The data-ready ISR records a sequence number and `micros()` timestamp. The main loop reads one coherent 14-byte acceleration/temperature/gyro burst, checks the byte count, and verifies no new data-ready edge occurred during the read. It does not reread unchanged registers at a nominal loop rate. A 64-bit accumulator extends the 32-bit microsecond clock across wraps. The loop must continue executing; a pause longer than one full 32-bit wrap cannot be reconstructed from `micros()` alone. An eight-hour normal run crosses several wraps and is supported by the arithmetic.

This adapter intentionally uses direct data-ready reads, **not a FIFO backlog**. More than one unconsumed edge, a burst crossing the next edge, an old read (>3 ms), an out-of-range interval (<2.5 or >7.5 ms), missing samples for >20 ms, an I2C failure, or raw clipping freezes navigation and latches a fault. It never invents acceleration for a missing interval. A subsequent button calibration establishes a **new origin epoch**; it does not silently resume the old trajectory. Telemetry continues to report the frozen state and fault. Clients must reject stopped states as current navigation. A configuration/readback failure requires device repair/restart.

The core expects interval-average IMU measurements. These DLPF output samples are a practical approximation, not calibrated delta-angle/delta-velocity integrals. DLPF group delay, ISR latency/jitter, oscillator error, aliasing, coning/sculling and vibration rectification remain hardware error sources. Measure them. Absolute aiding timestamps must be aligned to the effective measurement time; calling a current-time update with an old GNSS/UWB fix biases the result. The core rejects delayed timestamps; a production delayed-aiding path needs buffering and replay.

No transmission wait or serial print is in the sampling path. RadioLib `startTransmit` returns after staging the packet; the DIO0 callback sets a flag; `finishTransmit` cleans up in the loop. A 2-second timeout disables further RF transmission while sampling continues. Short synchronous SPI operations still take time: qualify the worst-case budget. Serial telemetry is disabled by default; when enabled, a whole frame is written only if `availableForWrite()` reports enough free space. It can drop telemetry frames rather than block acquisition. `max_compute_us` measures calibration/filter computation only, **not** the complete I2C/SPI/interrupt deadline budget. Use a logic analyzer and the drop counter for full-loop qualification.

## Initial alignment and true stationary information

After power-up, warm the sensor until its temperature and bias stabilize. Navigation waits for a debounced falling edge of the calibration button. Put the complete sensor assembly at genuine translational and rotational rest, press once, then keep it still for ten seconds. The adapter rejects excessive acceleration/gyro magnitude and sample variation; motion resets collection. A held button does not repeat calibration. The initial attitude aligns the measured gravity direction, and the mean gyro residual initializes gyro bias. It does **not** estimate accelerometer bias from one orientation. A button press during operation deliberately resets the local origin.

These statistical checks cannot prove rest: constant translational velocity is invisible to a six-axis IMU. The button is the operator's explicit declaration of initial rest. During navigation `VERIFIED_REST_PIN=-1` disables stationary updates by default. Connect it only to an independent signal that establishes both zero linear and angular velocity for the **sensor**, and validate wiring against unintended assertion. An IMU quiet flag, a wheel contact alone, or an idle motor command does not establish this condition. An asserted external input still must pass the core's quiet checks before updates are used.

Yaw is arbitrary at initial alignment. The output is a fixed local z-up frame whose heading is defined by initialization, not geographic east/north. The default Earth-rate vector is zero. To compensate Earth rotation, establish the frame's geographic heading and latitude and express Earth rate in that same frame; do not insert an ENU vector into an unknown-heading frame. A single low-cost stationary sensor cannot distinguish all gyro bias from Earth rotation reliably. Gravity alignment also inherits accelerometer calibration error. Do not tighten the initial covariance merely because many correlated samples were averaged.

## Fit this sensor's measured calibration

The supplied `calibration_config.h` is identity calibration and explicitly reports `Uncalibrated`. No constants from a different unit are supplied. The fitter uses only the Python standard library:

```sh
python3 tools/fit_calibration.py six_faces.csv \
  --gyro-temperature gyro_temperature.csv \
  --report calibration_report.json --header calibration_config.h
```

Acquire fresh timestamped raw samples through the same hardware/ranges/filter path. Keep the full raw recording and timestamps separately for replay; the fitter's input columns are:

```csv
face,ax,ay,az,gx,gy,gz,temp_c,stationary
```

Acceleration must already be in m/s², angular rate in rad/s and temperature in °C, before applying this fitted calibration. `stationary` must be `1` from independently established rest. For six-face data, `face` is `+x,-x,+y,-y,+z,-z`, indicating the positive specific-force axis when that face is upward. Collect at least 200 samples for each precisely aligned face at a stable temperature (screening rejects a >3 °C span). A longer hold with settling time and an accurate fixture is preferable. The tool rejects missing faces, implausible units, motion and excessive residuals; it fits diagonal gains and offsets, with no fabricated cross-axis terms. Its model is:

```
corrected = M * (raw - offset - temperature_slope * (T - T_reference))
```

The optional gyro-temperature recording must hold a **fixed orientation** through a controlled, slow thermal sweep. It needs at least 600 samples, a ≥10 °C span by default, and ≥100 samples in each third of the temperature range. Record long settled holds during both heating and cooling, and validate a linear model against separate holdout runs; hysteresis often prevents a single line from being sufficient. The fit's gyro intercept includes Earth's rotation projected in the fixture frame. It is not a gyro scale-factor, cross-axis, hysteresis, or accelerometer-temperature calibration. Avoid extrapolation outside the recorded temperature range.

The output JSON includes residuals, motion screening, coverage and limitations. Screening failure exits nonzero and does not write a new C++ header. A pre-existing header is left unchanged, so check the exit status and report before copying anything into the sketch. Passing is not an uncertainty certificate. Validate on held-out face orientations, a calibrated rotation reference, and separate thermal cycles. The core supports full 3×3 matrices and accelerometer temperature slopes for a properly characterized richer calibration.

## Version-2 wire protocol

Every frame is exactly 112 bytes. Integer fields are unsigned little-endian. Real fields are IEEE754 binary32 little-endian. The core computes in double precision; float conversion is only at the RF boundary. No compiler struct packing or host endianness is assumed. `tools/decode_telemetry.py packet.bin` verifies a complete saved frame and prints JSON.

| Offset | Length | Meaning |
|---:|---:|---|
| 0 | 2 | ASCII `IN` |
| 2 | 1 | Version `2` |
| 3 | 1 | Frame length `112` |
| 4 | 4 | Packet sequence, modulo 2³² |
| 8 | 4 | Origin epoch; changes after completed button alignment |
| 12 | 8 | Filter state's timestamp in microseconds since MCU boot |
| 20 | 4 | Status bit mask |
| 24 | 4 | Core and adapter fault mask |
| 28 | 12 | Position x/y/z, m |
| 40 | 12 | Velocity x/y/z, m/s |
| 52 | 12 | Diagonal position covariance, m² |
| 64 | 12 | Diagonal velocity covariance, m²/s² |
| 76 | 16 | Quaternion w/x/y/z, body → local |
| 92 | 4 | Latest sensor temperature, °C |
| 96 | 4 | Counted dropped/ambiguous samples |
| 100 | 4 | Valid burst sample count, modulo 2³² |
| 104 | 4 | Maximum filter/alignment compute time, µs |
| 108 | 4 | CRC32/ISO-HDLC over bytes 0…107 |

Status bits: `1 Initialized`, `2 Quiet`, `4 VerifiedStationary`, `8 Calibrating`, `16 AcquisitionStopped`, `32 RadioFailed`, `64 EncodingOverflow`, `128 WaitingForCalibration`, `256 Uncalibrated`. `Quiet` alone does not mean stationary. Position/velocity must not be used without initialization or when stopped, invalid/overflowed or numerically faulted. Covariance is the model's estimate, not a guaranteed error bound. Byte 12 reports the **last accepted state time**, allowing a receiver to detect frozen data; it is not the radio emission time.

Core fault bits are defined in `src/navigation.h`: `1 InvalidInput`, `2 TimeGap`, `4 Saturation`, `8 NumericalFailure`. Adapter bits: `1<<16 BusFault`, `1<<17 SampleLoss`, `1<<18 ConfigurationFault`, `1<<19 RadioFault`. Dropped counts do not imply reconstructable missing motion. CRC, length, version, sequence, epoch, timestamps and freshness must all be handled by the receiver. This packet is incompatible with the historical sender/README format.

## Commission before any accuracy claim

Compile with the real board SDK, inspect warnings, test pin/readback faults, inject bus and radio failures, verify button debounce and rest-input deassertion, and exercise at least two `micros()` wraps. Capture data-ready, I2C, radio SPI and a filter-timing GPIO on a logic analyzer. Verify the 200 Hz sequence with maximum radio traffic and all intended peripherals, stack margin, thermal stabilization, power integrity and no clipping. The adapter's fail-closed policy deliberately exposes deadline failures; if it fails under the intended load, move to timestamped FIFO acquisition or a dedicated sampling task and revalidate the correspondence of each timestamp to each FIFO sample.

For an eight-hour navigation claim, record a representative trajectory with independent reference position/velocity, synchronized times, raw IMU, temperature, rest truth, aiding quality and outage intervals. Measure repeated runs over actual temperature, vibration, magnetic/RF environment and mounting conditions. Report trajectory RMSE, 95th/max error, return-to-origin error, path-length error, outage growth and uncertainty consistency. The repository's synthetic aided benchmark requires the specified independent measurements; this example does not magically supply GNSS/UWB/vision or implement their transport. Add a synchronized driver through `updatePosition`/`updateVelocity` and validate lever arms and delays. See `VALIDATION_PLAN.md` and `REPLAY.md` for further testing.

## Primary API references checked

- [I2Cdev MPU6050 declarations](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.h) and [bounded reads](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/I2Cdev/I2Cdev.h).
- [RadioLib SX1278 API](https://github.com/jgromes/RadioLib/blob/master/src/modules/SX127x/SX1278.h) and [interrupt-driven transmission example](https://github.com/jgromes/RadioLib/blob/master/examples/SX127x/SX127x_Transmit_Interrupt/SX127x_Transmit_Interrupt.ino).
- [Arduino-ESP32 loop stack example](https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/ArduinoStackSize/ArduinoStackSize.ino).

Dependency versions are not pinned because no board/toolchain installation was available to validate a lock. Pin the exact successfully tested versions in the deployment build.
