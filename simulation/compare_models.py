#!/usr/bin/env python3
"""Combine paired continuous-10DOF experiments without mixing sensor profiles."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def named(value):
    name, path = value.split("=", 1)
    return name, Path(path).resolve()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", action="append", required=True,
                        help="ModelName=/path/to/summary.csv; repeat per model/ablation")
    parser.add_argument("--kernel", action="append", required=True,
                        help="ModelName=/path/to/navigation.cpp; adjacent header is hashed too")
    parser.add_argument("--output", default="simulation/results/model_comparison.csv")
    args = parser.parse_args()
    kernels = dict(named(x) for x in args.kernel)
    provenance = {"note": "Paired synthetic regression and calibration ablation; not hardware accuracy or a universal optimum.",
                  "scenario": "continuous_10dof_only", "kernels": {}, "inputs": []}
    for name, path in kernels.items():
        header = path.with_suffix(".h")
        states = int(re.search(r"constexpr int N = (\d+)", header.read_text()).group(1))
        provenance["kernels"][name] = {
            "error_states": states,
            "navigation.cpp_sha256": digest(path),
            "navigation.h_sha256": digest(header),
            "retained_source_directory": str(path.parent),
        }
    metrics = ["seed", "mag_systematics", "hours", "dt_s", "sensor_profile",
               "final_position_error_m", "rms_position_error_m", "max_position_error_m",
               "final_horizontal_error_m", "final_vertical_error_m",
               "rms_horizontal_error_m", "rms_vertical_error_m",
               "final_tilt_error_deg", "final_yaw_error_deg", "coverage_xyz_3sigma",
               "fault_bits", "final_covariance_cholesky_ok"]
    rows, baseline_by_seed = [], {}
    for name, path in (named(x) for x in args.input):
        provenance["inputs"].append({"model": name, "path": str(path), "sha256": digest(path)})
        matched = 0
        for row in csv.DictReader(path.open()):
            if row["scenario"] != provenance["scenario"]:
                continue
            if row["estimator"] == "cached_baseline":
                # Changing available aiding must not change the paired IMU realization.
                key = row["seed"]
                check = tuple(row[x] for x in ("hours", "dt_s", "sensor_profile",
                                               "final_position_error_m", "rms_position_error_m"))
                if key in baseline_by_seed and baseline_by_seed[key] != check:
                    raise ValueError("Input profiles/IMU streams are not paired for seed " + key)
                baseline_by_seed[key] = check
            elif row["estimator"] == "new_eskf":
                matched += 1
                rows.append({"model": name, "error_states": provenance["kernels"][name]["error_states"],
                             **{key: row[key] for key in metrics}})
        if not matched:
            raise ValueError("No target scenario in " + str(path))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["model", "error_states"] + metrics)
        writer.writeheader()
        writer.writerows(rows)
    provenance["paired_baseline_checks_passed"] = True
    provenance["comparison_csv_sha256"] = digest(output)
    output.with_suffix(".json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(f"Saved {len(rows)} paired rows to {output}")


if __name__ == "__main__":
    main()
