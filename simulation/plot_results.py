#!/usr/bin/env python3
"""Render the actual CSV results; requires matplotlib. Run from project root."""
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parents[1]
with (root/'simulation/results/summary.csv').open() as f: rows=list(csv.DictReader(f))
with (root/'simulation/results/timeseries.csv').open() as f: series=list(csv.DictReader(f))
manifest=json.loads((root/'simulation/results/manifest.json').read_text())
new=[r for r in rows if r['estimator']=='new_eskf']
labels={
 'stationary_verified':'Stationary, verified rest (6-axis)',
 'stop_go_verified':'Stop/go, verified rest (6-axis)',
 'continuous_imu_only':'Continuous, accel + gyro only',
 'continuous_position_1hz':'6-axis + position at 1 Hz',
 'continuous_position_outages_outliers':'6-axis + position, outages/outliers',
 'continuous_10dof_only':'10-DOF module only',
 'stop_go_10dof_verified':'10-DOF + verified full stops',
 'continuous_10dof_position_1hz':'10-DOF + position at 1 Hz',
 'walking_10dof_no_stance':'Hand/body-like motion, 10-DOF only'}
colors=['#177e89','#db6d27','#6956a8']
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(2,2,figsize=(13.6,8.8),layout='constrained')
fig.suptitle('Eight simulated hours • ST 10-DOF wearable model',fontsize=18,fontweight='bold')
def trace(ax,scenario,column,scale=1):
    for seed,c in zip(('1','2','3'),colors):
        rr=sorted((r for r in series if r['scenario']==scenario and r['seed']==seed),key=lambda r:float(r['time_s']))
        ax.plot([float(r['time_s'])/3600 for r in rr],[float(r[column])*scale for r in rr],color=c,lw=.85,alpha=.85,label='Seed '+seed)
    ax.set_xlabel('Time (hours)');ax.set_xlim(0,8);ax.grid(alpha=.2)
trace(axes[0,0],'continuous_10dof_position_1hz','new_position_error_m')
axes[0,0].set_title('With independent position fixes: 3D position error',loc='left',fontweight='bold')
axes[0,0].set_ylabel('Position error (m)');axes[0,0].legend(loc='upper right',frameon=False,ncol=3,fontsize=8)
axes[0,0].text(.02,.97,'Fixes: 1 Hz, 0.20 m noise per axis\nTraces sampled each minute; maxima use every sample',transform=axes[0,0].transAxes,va='top',fontsize=8,bbox={'facecolor':'white','alpha':.8,'edgecolor':'none'})
trace(axes[0,1],'continuous_10dof_only','new_yaw_deg')
axes[0,1].set_title('Module only: heading can look stable',loc='left',fontweight='bold');axes[0,1].set_ylabel('Yaw error (degrees)')
selected=['continuous_imu_only','continuous_10dof_only','walking_10dof_no_stance','stop_go_10dof_verified','continuous_10dof_position_1hz']
short=['Accel + gyro','10-DOF only','Hand/body-like\n10-DOF only','10-DOF +\nverified stops','10-DOF +\nposition fixes']
ax=axes[1,0]
for i,name in enumerate(selected):
    rr=[r for r in new if r['scenario']==name]
    for j,r in enumerate(rr):ax.scatter(i+(j-1)*.12,float(r['rms_horizontal_error_m']),color=colors[j],s=28,zorder=3)
ax.set_xticks(range(len(short)),short,fontsize=8);ax.set_yscale('log');ax.set_ylabel('Horizontal RMS error (m, log scale)');ax.grid(axis='y',alpha=.2)
ax.set_title('Horizontal position still needs information',loc='left',fontweight='bold')
ax.text(.98,.97,'Huge unaided values indicate model failure,\nnot real-world distance forecasts.',transform=ax.transAxes,va='top',ha='right',fontsize=8,bbox={'facecolor':'white','alpha':.9,'edgecolor':'none'})
ax=axes[1,1]
for seed,c in zip(('1','2','3'),colors):
    rr=sorted((r for r in series if r['scenario']=='continuous_10dof_only' and r['seed']==seed),key=lambda r:float(r['time_s']))
    ax.plot([float(r['time_s'])/3600 for r in rr],[abs(float(r['new_z'])-float(r['true_z'])) for r in rr],color=c,lw=.9)
ax.set_title('Barometer: altitude includes weather uncertainty',loc='left',fontweight='bold');ax.set_xlabel('Time (hours)');ax.set_ylabel('Vertical position error (m)');ax.set_xlim(0,8);ax.grid(alpha=.2)
fig.supxlabel('Synthetic residual calibration, thermal drift, noise, quantization and interference; 3 seeds. No hardware accuracy guarantee.',fontsize=9)
out=root/'docs';out.mkdir(exist_ok=True)
fig.savefig(out/'eight_hour_results.png',dpi=160);fig.savefig(out/'eight_hour_results.pdf');plt.close(fig)

def span(rr,key):
    values=[float(r[key]) for r in rr]
    if key=='coverage_xyz_3sigma':
        lo,hi=100*min(values),100*max(values)
        return f'{lo:.4g}%' if lo==hi else f'{lo:.4g}–{hi:.4g}%'
    return f'{min(values):.4g}–{max(values):.4g}'
lines=['# Eight-hour model results','',
       'These results use the actual C++ implementation and the synthetic ST sensor profile described in [the simulation methodology](../simulation/README.md). They are not measured hardware performance or a proof of minimum error. Each range spans three random seeds. Full source hashes and commands are in the [manifest](../simulation/results/manifest.json).',
       '', '![Eight-hour model results](eight_hour_results.png)','',
       '| Scenario | Final 3D error, m | RMS 3D error, m | Worst sample error, m | 3σ XYZ coverage |',
       '|---|---:|---:|---:|---:|']
for name,label in labels.items():
    rr=[r for r in new if r['scenario']==name]
    if rr:lines.append(f"| {label} | {span(rr,'final_position_error_m')} | {span(rr,'rms_position_error_m')} | {max(float(r['max_position_error_m']) for r in rr):.4g} | {span(rr,'coverage_xyz_3sigma')} |")
lines += ['', '## Held-out noise realizations', '',
          'Seeds 4–6 were run after freezing the model and priors. They vary noise under the same trajectory and systematic-error assumptions; they are not new hardware or new environments.', '',
          '| Scenario | Final 3D error, m | RMS 3D error, m | 3σ XYZ coverage |',
          '|---|---:|---:|---:|']
for folder, label in [('holdout_continuous','10-DOF module only'), ('holdout_walking','Hand/body-like, 10-DOF only'), ('holdout_aided','10-DOF + independent position at 1 Hz')]:
    path=root/'simulation'/folder/'summary.csv'
    if path.exists():
        with path.open() as f: rr=[r for r in csv.DictReader(f) if r['estimator']=='new_eskf']
        lines.append(f"| {label} | {span(rr,'final_position_error_m')} | {span(rr,'rms_position_error_m')} | {span(rr,'coverage_xyz_3sigma')} |")
comparison=root/'simulation/results/model_comparison.csv'
if comparison.exists():
    with comparison.open() as f: compared=list(csv.DictReader(f))
    lines += ['', '## Why persistent magnetic uncertainty matters', '',
              'The paired continuous-motion experiments retain identical inertial/noise streams. Earlier candidates were selected using seeds 1–3, so this is a development ablation. Full per-seed results and provenance are in [model_comparison.csv](../simulation/results/model_comparison.csv) and [its metadata](../simulation/results/model_comparison.json). The archive contains the final core; intermediate source snapshots are identified by hashes in the metadata.', '',
              '| Error states | Magnetic calibration residuals | 3σ XYZ coverage | Final yaw error, deg |',
              '|---:|---|---:|---:|']
    for n in sorted({int(r['error_states']) for r in compared}):
        for enabled in ('1','0'):
            rr=[r for r in compared if int(r['error_states'])==n and r['mag_systematics']==enabled]
            if rr:lines.append(f"| {n} | {'Included' if enabled=='1' else 'Removed (ablation)'} | {span(rr,'coverage_xyz_3sigma')} | {span(rr,'final_yaw_error_deg')} |")
    lines += ['', 'The 16-state candidate treats residual magnetic calibration as white noise. The 19-state candidate adds persistent offset uncertainty. The final 25-state model also retains symmetric scale/cross-axis uncertainty. Improved coverage means the filter better represents uncertainty in this model; it does not make the divergent horizontal position useful. A 100% sample coverage result is not a certified error bound.']
lines += ['', '## Interpretation','',
          '- The module measures acceleration, angular velocity, magnetic field and pressure. It does not provide independent horizontal position. Continuous unaided horizontal estimates diverge in this experiment. Their very large numerical errors are failure indicators outside the meaningful local-frame regime.',
          '- Magnetometer and barometer corrections can improve heading and height while horizontal position still fails. A stable-looking compass is not evidence of accurate coordinates.',
          '- The position-aided case assumes an additional independent source at 1 Hz with Gaussian 0.20 m per-axis noise. This capability is not supplied by the four chips on the user’s module.',
          '- Verified-stop cases receive independent knowledge of true rigid rest. A hand or torso during walking must not receive a foot-contact zero-velocity flag. The walking-like synthetic route does not receive one.',
          '- Coverage measures model consistency on correlated synthetic samples, not a probability guarantee. Persistent unmodeled magnetic, pressure or calibration errors can violate the reported covariance.',
          '- Initial position, velocity, orientation and the magnetic/Earth reference are favorably known in the model. Actual alignment, gyro scale, clock delay, vibration and sensor timing have not been measured.',
          '- Path length and final position error are separate metrics. This filter sums predicted displacement magnitudes; corrections do not retroactively repair accumulated distance.',
          '', '## Reproduce','', '```sh', 'python3 simulation/run_benchmark.py --sensitivity', 'python3 simulation/plot_results.py', '```','',
          'The plotting command requires Matplotlib. The estimator, tests and benchmark require a C++17 compiler, Make and Python; the numerical C++ implementation has no third-party matrix library dependency.',
          '', 'See [hardware commissioning](HARDWARE.md), [mathematical derivation](MATHEMATICS.md), and [validation requirements](VALIDATION_PLAN.md) before interpreting these results as an engineering error budget.']
(out/'RESULTS.md').write_text('\n'.join(lines)+'\n')
print('Wrote docs/eight_hour_results.png, .pdf and RESULTS.md from CSVs.')
