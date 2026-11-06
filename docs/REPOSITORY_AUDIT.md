# Source provenance and repository audit

The public URL supplied by the user was `https://github.com/Winston-Arkihit/accel-gyro-processing`. During this work (2026-11-06), direct Git access requested credentials and direct raw-file HTTP access returned 404. The web reader supplied a cached main-branch listing and several complete source files, marked as crawled the previous week. A current commit ID and complete current checkout could not be obtained. **This deliverable is a local replacement implementation based on that recoverable snapshot, not a pushed branch or a claim to have reviewed an unavailable current checkout.**

`baseline/` preserves the six recovered source files exactly as their line-numbered web views showed them, except presentation metadata/line prefixes were removed. These are benchmark/audit inputs, not deployable firmware. No license grant has been invented for the original code. Keep its ownership/provenance when integrating your repository.

| Listed original file | Inspection and action |
|---|---|
| `algorithm_core.h` | Recovered. New API moves into `src/navigation.h`, makes state per-instance, defines units and conventions, and adds residual biases and covariance. |
| `algorithm_core.cpp` | Recovered and compiled unchanged in the benchmark. Replaced for deployment by `src/navigation.cpp`; removed hard dead zones and autonomous false zero-velocity updates. |
| `data_filtration.cpp` | Recovered and compiled unchanged in the benchmark. Its gravity-direction signs contradict the body-to-navigation rotation matrix. Separate attitude and translation stages are replaced by one consistent ESKF. |
| `alpgorithm_core.c` | Recovered for audit. Old interface refers to `TacticalMovement` absent from the current recovered header. Variance-only rest detection can suppress true constant acceleration. Retained only as an archived baseline artifact. |
| `mpu_n_send.c` | Recovered for audit. Uses the obsolete navigation API, raw packed floats, no enforced radio interval, and fabricated replacement `dt` after delays. Replaced by the coherent firmware example. |
| `calibration.h` | Recovered. A single stationary hardware-offset routine is insufficient to characterize scale, cross-axis and temperature effects. New fitting tools, calibration configuration and startup validation are provided. |
| `Data_filtration.c` | Present in cached listing, contents unavailable. It has not been invented, compiled, or claimed reviewed. Do not include it alongside the new implementation without inspecting it. |
| `README.md` | Read through cached repository landing page. Refers to a missing `.ino` and a wire protocol that disagrees with `mpu_n_send.c`. Replaced by this deliverable's README and explicit protocol documentation. |
| `.gitignore` | Listed but contents unavailable. New local ignore rules cover generated builds. |

There was no visible receiver implementation or hardware-board definition in the recovered listing. Receiver compatibility and an actual board build require those inputs. The archive's build compiles an explicit file list: it does not accidentally compile old C and C++ implementations together.

## Design defects addressed

1. Inconsistent gravity projection and first-order quaternion drift: one stated Hamilton convention, exponential rotation, midpoint force and independently tested Jacobians.
2. Quiet motion mistaken for zero velocity: quiet detection is diagnostic; stationary updates require independently asserted rigid rest and internal consistency.
3. Real small accelerations discarded: no arbitrary per-axis acceleration dead zone.
4. Untracked bias and error growth: a 25-state error covariance, residual inertial and magnetic bias estimation, six symmetric residual soft-iron terms, pressure-offset uncertainty, calibration/temperature input, and independent position/velocity updates. The target was updated from the repository's MPU6050 to the user's actual ST 10-DOF module.
5. Long-run attitude covariance instability: exact frozen-rate angular transition and bias coupling instead of Euler rotation of covariance.
6. Fragile numerical correction: joint innovation gating, scalar Joseph-equivalent covariance updates, tangent-frame reset and finite-value/fault handling. These are not a formal proof of PSD under all inputs; numerical tests check PSD.
7. Timing and acquisition ambiguity: explicit timestamps, no invented elapsed time, clock-wrap handling, sample-loss/saturation fault states and separately scheduled telemetry.
8. No reproducible evidence: original code is benchmarked unchanged against the actual new C++ core, with declared synthetic assumptions, independent analytic truth, seeds, hashes and adverse scenarios.

## Not solved by this implementation

Absolute position and yaw without adequate observations; the user's unprovided sensor calibration and motion model; the unavailable current repository files; sensor timing and noise on the actual board; high-dynamics coning/sculling; global geodetic travel; robust asynchronous sensor fusion; radio-network collisions; guaranteed smallest real-world error. The included tests and simulations do not substitute for those missing measurements.
