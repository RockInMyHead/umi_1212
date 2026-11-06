#!/usr/bin/env python3
"""Create an Arduino sketch containing a snapshot of the canonical core sources."""
import argparse
from pathlib import Path
import shutil

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("destination", help="New sketch directory; existing directory is never overwritten")
parser.add_argument("--sensor", choices=("st", "mpu6050"), default="st", help="Actual Troyka v2 ST module (default), or legacy MPU6050")
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
destination = Path(args.destination).resolve()
if destination.exists():
    parser.error("Destination already exists; choose a new directory to preserve your wiring/calibration")
required = [root / "src" / "navigation.h", root / "src" / "navigation.cpp"]
if not all(path.exists() for path in required):
    parser.error("Canonical src/navigation.h and src/navigation.cpp are required")
destination.mkdir(parents=True)
legacy = root / "firmware" / "imu_lora"
selected = root / "firmware" / ("st_imu_lora" if args.sensor == "st" else "imu_lora")
for source in selected.iterdir():
    if source.is_file():
        target = destination / (destination.name + ".ino" if source.suffix == ".ino" else source.name)
        shutil.copy2(source, target)
if args.sensor == "st":
    for name in ("acquisition.h", "telemetry.h", "calibration_config.h"):
        shutil.copy2(legacy / name, destination / name)
for source in required:
    shutil.copy2(source, destination / source.name)
example = destination / "board_config.example.h"
shutil.copy2(example, destination / "board_config.h")
print(f"Staged {destination}. Set board_config.h, install dependencies, then compile for your actual board.")
