"""Run the complete student-side study, identification, plots and validation.

Run from repository root: python -m vdm_lab.student.reproduce
Use --resume only for an interrupted run with unchanged simulation code/configs.
Presentation regeneration is separate: npm install; node build_presentation.js
(from vdm_lab/student, with the shipped package.json).
"""
import argparse,os,subprocess,sys
from pathlib import Path

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args();root=Path(__file__).resolve().parents[2]
    env=os.environ.copy();env['PYTHONDONTWRITEBYTECODE']='1';env.setdefault('OPENBLAS_NUM_THREADS','1')
    for module,extra in [('experiments',['--suite','full']+(['--resume'] if args.resume else [])),
                         ('identify_models',[]),('visualizations',[]),('validate_delivery',[]),('build_report',[])]:
        subprocess.run([sys.executable,'-m','vdm_lab.student.'+module,*extra],cwd=root,env=env,check=True)
    print('Study rebuilt; see vdm_lab/student/README_REFINED.md for presentation regeneration.')
if __name__=='__main__':main()
