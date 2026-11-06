#!/usr/bin/env python3
"""Fit a measured magnetometer ellipsoid with held-out screening (requires NumPy).

Input CSV: mx_uT,my_uT,mz_uT, covering many static orientations in a uniform field.
Use a measured local field magnitude; no nominal sensor spec determines it.
This fit cannot identify the rotation between magnetometer and IMU body axes.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def ellipsoid(points, field):
    origin = np.mean(points, axis=0)
    unit = np.sqrt(np.mean((points-origin)**2))
    if not np.isfinite(unit) or unit < 1e-6:
        raise ValueError("No three-dimensional field variation")
    p = (points-origin)/unit
    x,y,z = p.T
    design = np.column_stack((x*x,y*y,z*z,2*x*y,2*x*z,2*y*z,2*x,2*y,2*z))
    condition = float(np.linalg.cond(design))
    if condition > 1e5:
        raise ValueError("Degenerate orientation coverage or ill-conditioned ellipsoid")
    c,_,rank,_ = np.linalg.lstsq(design, np.ones(len(p)), rcond=None)
    if rank < 9:
        raise ValueError("Insufficient independent orientations")
    a = np.array([[c[0],c[3],c[4]],[c[3],c[1],c[5]],[c[4],c[5],c[2]]])
    center = -np.linalg.solve(a,c[6:9])
    radius2 = 1 + center@a@center
    shape = a/radius2
    eigenvalues,eigenvectors = np.linalg.eigh(shape)
    if np.min(eigenvalues) <= 0 or np.max(eigenvalues)/np.min(eigenvalues) > 100:
        raise ValueError("Fit is not a plausible positive-definite ellipsoid")
    matrix = np.einsum('ik,jk->ij',eigenvectors*np.sqrt(eigenvalues),eigenvectors) * field/unit
    offset = origin+unit*center
    return offset,matrix,condition


def fit(points, field, max_relative_residual=.05):
    p = np.asarray(points,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<600 or not np.all(np.isfinite(p)):
        raise ValueError("Need at least 600 finite three-axis samples")
    if not 10 <= field <= 100 or not 0 < max_relative_residual < .5:
        raise ValueError("Check field magnitude (10..100 uT) and residual threshold")
    # Deterministic interleaved hold-out: separate measurements, not an independent
    # session. Validate again in a different session and the actual final mounting.
    held = np.arange(len(p))%5==0
    offset,matrix,condition = ellipsoid(p[~held],field)
    corrected=np.einsum('nj,ij->ni',p-offset,matrix)
    radii=np.linalg.norm(corrected,axis=1)
    if np.min(radii)<1e-6:
        raise ValueError("A sample collapses to the field center")
    directions=corrected/radii[:,None]
    coverage=np.linalg.eigvalsh(np.einsum('ni,nj->ij',directions,directions)/len(p))
    octants=np.sum((directions>=0)*np.array([1,2,4]),axis=1)
    octant_counts=[int(np.sum(octants==i)) for i in range(8)]
    if np.min(coverage)<.12 or np.linalg.norm(np.mean(directions,axis=0))>.35:
        raise ValueError("Orientations do not cover the full sphere sufficiently")
    if min(octant_counts)<10:
        raise ValueError("Each of the eight directional octants needs at least 10 samples")
    errors=radii[held]-field
    p95=float(np.quantile(np.abs(errors),.95))
    if p95>field*max_relative_residual:
        raise ValueError("Held-out radial error too large; field may be disturbed or time-varying")
    return {
        "hard_iron_uT":offset.tolist(),"soft_iron_matrix":matrix.tolist(),
        "reference_field_magnitude_uT":field,"samples":len(p),"held_out_samples":int(sum(held)),
        "held_out_radial_rms_uT":float(np.sqrt(np.mean(errors**2))),
        "held_out_radial_p95_uT":p95,"orientation_coverage_eigenvalues":coverage.tolist(),
        "design_condition_number":condition,"passed_radial_screening":True,
        "octant_counts":octant_counts,
        "model":"corrected_uT = soft_iron_matrix @ (raw_uT - hard_iron_uT)",
        "limitations":[
            "Magnitude fitting leaves an unobservable rotation: independently align sensor axes to the IMU.",
            "Low radial residual does not certify accurate heading or correct magnetic declination.",
            "Uniform stable field and broad orientation coverage are required; ferrous disturbances can bias the fit.",
            "Interleaved hold-out is not independent environmental validation; repeat after final mounting.",
            "This fit does not characterize temperature drift or transient magnetic disturbances."]}


def header(report):
    vector=lambda v:"{"+", ".join(format(x,".17g") for x in v)+"}"
    return "\n".join(["#pragma once",'#include "navigation.h"',
                      "namespace magnetic_calibration {", "constexpr bool provided=true;",
                      "constexpr imu::Vec3 hard_iron_uT="+vector(report["hard_iron_uT"])+";",
                      "constexpr double soft_iron[3][3]={"+", ".join(vector(r) for r in report["soft_iron_matrix"])+"};",
                      "}",""])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path");parser.add_argument("--field-uT",required=True,type=float)
    parser.add_argument("--report","--output",dest="output",required=True)
    parser.add_argument("--header",help="Generate magnetometer_calibration.h only after successful screening")
    parser.add_argument("--max-relative-residual",type=float,default=.05)
    args=parser.parse_args()
    try:
        with open(args.csv_path,newline="",encoding="utf-8-sig") as f:
            points=[[float(r[k]) for k in ("mx_uT","my_uT","mz_uT")] for r in csv.DictReader(f)]
        report=fit(points,args.field_uT,args.max_relative_residual)
        Path(args.output).write_text(json.dumps(report,indent=2)+"\n")
        if args.header: Path(args.header).write_text(header(report))
        print(f"Radial screening passed: held-out p95 {report['held_out_radial_p95_uT']:.4f} uT; axis alignment still required.")
    except (ValueError,KeyError,OSError,np.linalg.LinAlgError) as e:
        parser.error(str(e))

if __name__=="__main__": main()
