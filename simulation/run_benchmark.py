#!/usr/bin/env python3
"""Build, run, and record exact source hashes. Python standard library only."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ["Makefile", "src/navigation.h", "src/navigation.cpp",
           "tests/navigation_tests.cpp", "tests/physics_tests.cpp", "tests/trajectory_tests.cpp", "simulation/benchmark.cpp",
           "simulation/run_benchmark.py", "baseline/algorithm_core.h",
           "baseline/algorithm_core.cpp", "baseline/data_filtration.cpp"]


def hashes():
    return {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in SOURCES}


def run(hours, dt, seeds, output, mag_systematics=1, scenario=-1, seed_start=1):
    source_hashes = hashes()
    manifest_path = ROOT / output / "manifest.json"
    # A failed/interrupted rerun must not leave an old manifest beside new CSVs.
    if manifest_path.exists():
        manifest_path.unlink()
    command = ["./work-build/benchmark", "--hours", str(hours), "--dt", str(dt),
               "--seeds", str(seeds), "--output", output,
               "--mag-systematics", str(mag_systematics), "--scenario", str(scenario),
               "--seed-start", str(seed_start)]
    started = time.time()
    subprocess.run(command, cwd=ROOT, check=True)
    if source_hashes != hashes():
        raise RuntimeError("Sources changed during the run; results are unverified")
    manifest = {
        "command": command, "source_sha256": source_hashes,
        "elapsed_wall_s": time.time() - started,
        "compiler": subprocess.check_output(["c++", "--version"], text=True).strip(),
        "platform": platform.platform(), "python": platform.python_version(),
        "seeds": list(range(seed_start, seed_start + seeds)), "simulated_hours_per_seed_scenario": hours,
        "sample_interval_s": dt,
        "scenarios": 9 if scenario == -1 else 1,
        "selected_scenario": scenario,
        "magnetic_systematics_enabled": bool(mag_systematics),
        "sensor_profile": "st10dof_calibrated_synthetic",
        "note": "ST10DOF-appropriate quantization with synthetic calibrated residuals; not measured hardware performance. std::normal_distribution is reproducible within this C++ library, not guaranteed bit-identical across libraries.",
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hours", type=float, default=8)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--seed-start", type=int, default=1)
    parser.add_argument("--output", default="simulation/results")
    parser.add_argument("--scenario", type=int, default=-1)
    parser.add_argument("--mag-systematics", type=int, choices=(0, 1), default=1)
    parser.add_argument("--sensitivity", action="store_true")
    args = parser.parse_args()
    subprocess.run(["make", "all", "test"], cwd=ROOT, check=True)
    run(args.hours, args.dt, args.seeds, args.output, args.mag_systematics, args.scenario, args.seed_start)
    if args.sensitivity:
        for step in (0.005, 0.01, 0.02):
            run(0.05, step, 1, f"simulation/sensitivity/dt_{step}", args.mag_systematics, args.scenario, args.seed_start)
