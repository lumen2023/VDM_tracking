"""Validate original-core integrity, test results and recorded actuator bounds."""
from __future__ import annotations
import hashlib,json,subprocess,sys,csv,math
from pathlib import Path
import numpy as np
from vdm_lab.student.experiments import write_json,OUT,ROOT


SKIP_PREFIXES = (
    ".git/",
    "outputs/",
    "Revised/",
    ".cursor/",
    "__pycache__/",
)


def _ignored(relative):
    relative = relative.replace("\\", "/")
    return any(
        relative.startswith(prefix) or f"/{prefix}" in f"/{relative}"
        for prefix in SKIP_PREFIXES
    ) or relative.endswith(".pyc")


def main():
    here=Path(__file__).resolve().parent
    expected=json.loads((here/'provenance/original_sha256.json').read_text())
    outside={k:v for k,v in expected.items() if not k.startswith('vdm_lab/student/')}
    changed=[k for k,v in outside.items() if not (ROOT/k).exists() or hashlib.sha256((ROOT/k).read_bytes()).hexdigest()!=v]
    added=[str(p.relative_to(ROOT)) for p in ROOT.rglob('*') if p.is_file()
           and not str(p.relative_to(ROOT)).startswith('vdm_lab/student/')
           and str(p.relative_to(ROOT)) not in expected
           and not _ignored(str(p.relative_to(ROOT)))]
    proof=dict(original_files_outside_student=len(outside),changed_original_files=changed,added_files_outside_student=added,
               all_original_files_outside_student_identical=not changed,algorithm='SHA-256',expected=outside)
    write_json(OUT/'validation/core_integrity.json',proof)
    errors=[];rows_checked=0;calls=0
    for config_path in sorted((OUT/'runs').glob('*/config.json')):
        stored=json.loads(config_path.read_text());c=stored['case'];v=stored['plant_config']['vehicle']
        with (config_path.parent/'trajectory.csv').open() as f:rows=list(csv.DictReader(f))
        steer=np.array([float(r['steer']) for r in rows]);speed=np.array([float(r['speed']) for r in rows]);a=np.array([float(r['acceleration']) for r in rows])
        checks={'finite states and error':all(all(math.isfinite(float(r[k])) for k in ('x','y','yaw','speed','projection_error')) for r in rows),
            'steering angle':np.max(abs(steer))<=v['max_steer']+1e-6,
            'steering rate':np.max(abs(np.diff(np.r_[0.,steer])))/c['dt']<=v['max_steer_rate']+1e-5,
            'speed':np.min(speed)>=v['min_speed']-1e-6 and np.max(speed)<=v['max_speed']+1e-6,
            'acceleration':np.max(a)<=v['max_accel']+1e-6 and np.min(a)>=-v['max_decel']-1e-6}
        for name,passed in checks.items():
            if not passed:errors.append({'case':c['case_id'],'check':name})
        rows_checked+=len(rows);calls+=1
    write_json(OUT/'validation/trajectory_checks.json',dict(runs_checked=calls,rows_checked=rows_checked,violations=errors,
            note='Numerical actuator/state checks only; not road-safety certification.'))
    result=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(here/'tests'),'-v'],cwd=ROOT,capture_output=True,text=True)
    (OUT/'validation').mkdir(parents=True,exist_ok=True)
    (OUT/'validation/unit_tests.txt').write_text(result.stdout+result.stderr,encoding='utf-8')
    fingerprint={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in here.rglob('*') if p.is_file() and p.suffix in {'.py','.js'} and 'results' not in p.parts and 'results_smoke' not in p.parts}
    write_json(OUT/'validation/student_source_sha256.json',fingerprint)
    print(f'Core: {len(outside)} original files unchanged={not changed}; {calls} runs / {rows_checked} samples checked; violations={len(errors)}; tests exit={result.returncode}')
    if changed or added or errors or result.returncode:raise SystemExit(1)
    return proof
if __name__=='__main__':main()
