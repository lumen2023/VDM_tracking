"""Recompute all derived metrics from saved state/input logs without resimulation."""
import csv,json
from pathlib import Path
from vdm_lab.student.experiments import Case,OUT,config_for,prepared_path,local_projection,analyze,write_csv,write_json
from vdm_lab.common.types import VehicleState

def main(out=OUT):
    summary=[]
    for p in sorted((Path(out)/'runs').glob('*/config.json')):
        stored=json.loads(p.read_text());case=Case(**stored['case'])
        config,_=config_for(case);path=prepared_path(config)
        rows=list(csv.DictReader((p.parent/'trajectory.csv').open()))
        for row in rows:
            for key,value in row.items():
                if key not in ('controller_status','solver_status'):row[key]=float(value)
            row['target_index']=int(row['target_index'])
            state=VehicleState(row['x'],row['y'],row['yaw'],row['speed'])
            ey,progress,px,py,seg,f=local_projection(path,state,row['target_index'])
            row.update(projection_error=ey,projection_distance=((state.x-px)**2+(state.y-py)**2)**.5)
        old=json.loads((p.parent/'metrics.json').read_text())
        metric=analyze(case,path,rows,config,old['termination'])
        write_csv(p.parent/'trajectory.csv',rows);write_json(p.parent/'metrics.json',metric)
        summary.append(metric)
    write_json(Path(out)/'summary.json',summary);write_csv(Path(out)/'summary.csv',summary)
    print(f'Reanalyzed {len(summary)} cases from raw logs.')
if __name__=='__main__':main()
