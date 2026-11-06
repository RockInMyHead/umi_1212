#!/usr/bin/env python3
"""Fit sensor-specific SI calibration. Standard library only; no synthetic defaults.

Six-face CSV: face,ax,ay,az,gx,gy,gz,temp_c,stationary
face is +x,-x,+y,-y,+z,-z; accelerometer m/s^2; gyro rad/s.
Optional gyro-temperature CSV uses the same columns except face is unnecessary.
Every used row must have stationary=1, declared from external evidence.
"""
import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean

FACES = ("+x", "-x", "+y", "-y", "+z", "-z")
NUMERIC = ("ax", "ay", "az", "gx", "gy", "gz", "temp_c")


def read_rows(path, faces=False):
    result = []
    with open(path, newline="", encoding="utf-8-sig") as stream:
        for line, row in enumerate(csv.DictReader(stream), 2):
            if row.get("stationary") != "1":
                raise ValueError(f"{path}:{line}: stationary must be explicitly 1")
            if faces and row.get("face") not in FACES:
                raise ValueError(f"{path}:{line}: face must be one of {FACES}")
            parsed = {name: float(row[name]) for name in NUMERIC}
            if not all(math.isfinite(v) for v in parsed.values()):
                raise ValueError(f"{path}:{line}: non-finite data")
            parsed["face"] = row.get("face")
            result.append(parsed)
    if not result:
        raise ValueError(f"{path}: empty data")
    return result


def vector_rms(rows, columns):
    center = [mean(r[c] for r in rows) for c in columns]
    return math.sqrt(sum(sum((r[c]-center[i])**2 for i, c in enumerate(columns)) for r in rows)/len(rows))


def fit_accel(rows, gravity=9.80665, minimum=200):
    groups = {face: [r for r in rows if r["face"] == face] for face in FACES}
    issues = []
    for face, samples in groups.items():
        if len(samples) < minimum:
            raise ValueError(f"{face}: {len(samples)} samples, need at least {minimum}")
    temperatures = [r["temp_c"] for r in rows]
    if max(temperatures)-min(temperatures) > 3:
        issues.append("Six-face temperature span exceeds 3 C; stabilize temperature first.")
    scale, offset = [], []
    for axis, column in zip("xyz", ("ax", "ay", "az")):
        plus = mean(r[column] for r in groups["+"+axis])
        minus = mean(r[column] for r in groups["-"+axis])
        span = plus-minus
        if not 1.5*gravity < span < 2.5*gravity:
            raise ValueError(f"{axis}: implausible ±face span {span:g}; verify units and face labels")
        scale.append(2*gravity/span)
        offset.append((plus+minus)/2)
    diagnostics = {}
    for face, samples in groups.items():
        expected = [0., 0., 0.]
        expected["xyz".index(face[1])] = gravity if face[0] == "+" else -gravity
        residuals = [[scale[i]*(r[c]-offset[i])-expected[i] for i, c in enumerate(("ax", "ay", "az"))] for r in samples]
        rms = math.sqrt(sum(sum(x*x for x in v) for v in residuals)/len(samples))
        bias = [mean(v[i] for v in residuals) for i in range(3)]
        accel_rms = vector_rms(samples, ("ax", "ay", "az"))
        gyro_rms = vector_rms(samples, ("gx", "gy", "gz"))
        gyro_norm = math.sqrt(sum(mean(r[c] for r in samples)**2 for c in ("gx", "gy", "gz")))
        diagnostics[face] = dict(samples=len(samples), residual_rms_m_s2=rms,
                                 residual_mean_m_s2=bias, accel_variation_rms_m_s2=accel_rms,
                                 gyro_variation_rms_rad_s=gyro_rms, gyro_mean_norm_rad_s=gyro_norm)
        if accel_rms > 0.15 or gyro_rms > 0.015 or gyro_norm > 0.05:
            issues.append(f"{face}: motion/vibration or excessive gyro bias detected.")
        if rms > 0.30:
            issues.append(f"{face}: fit residual >0.30 m/s^2; fixture alignment/cross-axis errors need investigation.")
    return dict(scale=scale, offset=offset, reference_temperature_c=mean(temperatures),
                diagnostics=diagnostics, issues=issues)


def fit_gyro_temperature(rows, minimum_span=10):
    temps = [r["temp_c"] for r in rows]
    span = max(temps)-min(temps)
    if len(rows) < 600 or span < minimum_span:
        raise ValueError("Gyro temperature fit needs >=600 stationary samples and the requested temperature span")
    # Require coverage, not just two outlying temperatures.
    bins = [sum(min(2, int(3*(t-min(temps))/span)) == b for t in temps) for b in range(3)]
    if min(bins) < 100:
        raise ValueError("Gyro temperature data needs >=100 samples in each third of the temperature range")
    reference = mean(temps)
    centered = [t-reference for t in temps]
    denominator = sum(t*t for t in centered)
    offset, slope, rms = [], [], []
    for column in ("gx", "gy", "gz"):
        intercept = mean(r[column] for r in rows)
        gradient = sum(t*(r[column]-intercept) for t, r in zip(centered, rows))/denominator
        errors = [r[column]-intercept-gradient*t for t, r in zip(centered, rows)]
        offset.append(intercept); slope.append(gradient)
        rms.append(math.sqrt(sum(e*e for e in errors)/len(errors)))
    issues = []
    if max(rms) > 0.015:
        issues.append("Gyro temperature residual >0.015 rad/s; reject motion/hysteresis or use a validated richer model.")
    accel_rms = vector_rms(rows, ("ax", "ay", "az"))
    if accel_rms > 0.30:
        issues.append("Gyro temperature acquisition changed orientation or acceleration; fixture must remain fixed.")
    return dict(offset=offset, slope=slope, reference_temperature_c=reference,
                range_c=[min(temps), max(temps)], residual_rms_rad_s=rms,
                samples=len(rows), temperature_bin_counts=bins, issues=issues)


def cpp_header(report):
    accel, gyro = report["accelerometer"], report.get("gyroscope_temperature")
    ref = gyro["reference_temperature_c"] if gyro else accel["reference_temperature_c"]
    vector = lambda values: "{" + ", ".join(format(v, ".17g") for v in values) + "}"
    lines = ["#pragma once", '#include "navigation.h"',
             "// Generated from measured data; see its JSON report and validity range.",
             "inline imu::Calibration measuredCalibration() {", "    imu::Calibration c;",
             f"    c.reference_temperature_c = {ref:.17g};",
             f"    c.accel_offset = {vector(accel['offset'])};"]
    for i, value in enumerate(accel["scale"]):
        lines.append(f"    c.accel_matrix[{i}][{i}] = {value:.17g};")
    if gyro:
        lines.extend([f"    c.gyro_offset = {vector(gyro['offset'])};",
                      f"    c.gyro_temp_slope = {vector(gyro['slope'])};"])
    lines.extend(["    return c;", "}", "inline bool measuredCalibrationProvided() { return true; }", ""])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("six_face_csv")
    parser.add_argument("--gyro-temperature")
    parser.add_argument("--gravity", type=float, default=9.80665)
    parser.add_argument("--minimum-face-samples", type=int, default=200)
    parser.add_argument("--minimum-temperature-span", type=float, default=10.)
    parser.add_argument("--report", required=True)
    parser.add_argument("--header", help="Write calibration_config.h only when checks pass")
    args = parser.parse_args()
    if not 9.7 < args.gravity < 9.9 or args.minimum_face_samples < 20 or args.minimum_temperature_span < 5:
        parser.error("Gravity must be 9.7..9.9, face sample count >=20, temperature span >=5 C")
    try:
        report = {"accelerometer": fit_accel(read_rows(args.six_face_csv, True), args.gravity, args.minimum_face_samples)}
        if args.gyro_temperature:
            report["gyroscope_temperature"] = fit_gyro_temperature(read_rows(args.gyro_temperature), args.minimum_temperature_span)
        issues = report["accelerometer"]["issues"] + report.get("gyroscope_temperature", {}).get("issues", [])
        report["passed_screening"] = not issues
        report["limitations"] = [
            "Screening is not a calibration uncertainty certificate; validate on separate measurements.",
            "Diagonal accelerometer fit assumes accurately aligned faces; it does not identify cross-axis terms.",
            "No accelerometer temperature model or gyro scale/cross-axis model is identified.",
            "Stationary gyro offset includes local Earth rotation; unknown heading prevents separation.",
            "Linear temperature fits must not be extrapolated; assess heating/cooling hysteresis separately.",
            "DLPF correlation makes sample count different from independent observation count."]
        Path(args.report).write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
        if issues:
            for issue in issues:
                print("REJECT:", issue)
            return 2
        if args.header:
            Path(args.header).write_text(cpp_header(report), encoding="utf-8")
        print("Calibration screening passed; validate with independent data before deployment.")
        return 0
    except (ValueError, KeyError, OSError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
