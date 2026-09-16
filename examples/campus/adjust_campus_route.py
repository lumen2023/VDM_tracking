"""Locally widen tight turns with C2 quintic Hermite joins; retain road sequence."""
import csv
import json
import hashlib
import xml.etree.ElementTree as ET
import numpy as np
from scipy.interpolate import CubicHermiteSpline, BPoly
from scipy.integrate import cumulative_trapezoid
from campus_route import ROOT, to_lonlat, speed_profile


def main():
    src=ROOT/'data/route_before_adjustment.csv'
    old=np.genfromtxt(src,delimiter=',',names=True)
    u=old['s_m']; xy=np.column_stack([old['x_m'],old['y_m']])
    tangent=np.column_stack([np.cos(old['yaw_rad']),np.sin(old['yaw_rad'])])
    original=CubicHermiteSpline(u,xy,tangent)
    # Three principal turns plus the two closely spaced high-curvature transitions.
    regions=[(268.4,274.4),(571.2,580.0),(671.2,676.4),(827.4,832.8)]
    patches=[]
    for low,high in regions:
        center=(low+high)/2
        for halfwidth in np.arange(max((high-low)/2+2,6),26,.5):
            a,b=center-halfwidth,center+halfwidth
            spline=BPoly.from_derivatives([a,b],[[original(a),original(a,1),original(a,2)],
                                                [original(b),original(b,1),original(b,2)]])
            grid=np.linspace(a,b,4001);d=spline(grid,1);dd=spline(grid,2)
            curvature=(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/np.linalg.norm(d,axis=1)**3
            if max(abs(curvature))<=.19 and min(np.linalg.norm(d,axis=1))>.2:
                patches.append((a,b,spline));break
        else:raise RuntimeError('No feasible local join')
    for (a,b,_),(c,d,_) in zip(patches,patches[1:]):assert b<c
    dense=np.unique(np.concatenate([np.arange(0,u[-1],.01),[u[-1]],[v for a,b,_ in patches for v in (a,b)]]))
    def evaluate(t,derivative=0):
        result=original(t,derivative)
        for a,b,spline in patches:
            mask=(t>=a)&(t<=b);result[mask]=spline(t[mask],derivative)
        return result
    d=evaluate(dense,1);speed=np.linalg.norm(d,axis=1)
    station=np.r_[0,cumulative_trapezoid(speed,dense)]
    s=np.r_[np.arange(0,station[-1],.2),station[-1]]
    t=np.interp(s,station,dense)
    new=evaluate(t);d=evaluate(t,1);dd=evaluate(t,2)
    yaw=np.unwrap(np.arctan2(d[:,1],d[:,0]));k=(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/np.linalg.norm(d,axis=1)**3
    dense_dd=evaluate(dense,2)
    dense_k=(evaluate(dense,1)[:,0]*dense_dd[:,1]-evaluate(dense,1)[:,1]*dense_dd[:,0])/speed**3
    assert max(abs(dense_k))<.20
    assert np.allclose(new[[0,-1]],xy[[0,-1]],atol=1e-9)
    lon,lat=to_lonlat(new[:,0],new[:,1]);v=speed_profile(s,k,3)
    with (ROOT/'data/planned_path.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(old.dtype.names);w.writerows(zip(s,new[:,0],new[:,1],yaw,k,v,lon,lat))
    ns='http://www.topografix.com/GPX/1/1';ET.register_namespace('',ns)
    tag=lambda name:'{'+ns+'}'+name
    g=ET.Element(tag('gpx'),version='1.1',creator='Campus curvature-feasible smooth plan')
    meta=ET.SubElement(g,tag('metadata'));ET.SubElement(meta,tag('name')).text='Campus: Start to Destination'
    ET.SubElement(meta,tag('desc')).text='Smooth planned route with locally widened turns; not a measured GNSS track.'
    seg=ET.SubElement(ET.SubElement(g,tag('trk')),tag('trkseg'))
    for lo,la in zip(lon,lat):ET.SubElement(seg,tag('trkpt'),lat=f'{la:.10f}',lon=f'{lo:.10f}')
    ET.indent(g);ET.ElementTree(g).write(ROOT/'data/gpx/campus_route.gpx',encoding='utf-8',xml_declaration=True)
    delta=np.deg2rad(35);L=2.5;lr=1.25;kmax=np.tan(delta)/np.sqrt(L**2+(lr*np.tan(delta))**2)
    req=np.arctan(L*abs(dense_k)/np.sqrt(1-(lr*dense_k)**2))
    joins=[]
    for a,b,spline in patches:
        joins.append(dict(old_station_start_m=a,old_station_end_m=b,
            position_join_error_m=float(max(np.linalg.norm(spline(x)-original(x)) for x in (a,b))),
            tangent_join_error=float(max(np.linalg.norm(spline(x,1)-original(x,1)) for x in (a,b))),
            second_derivative_join_error=float(max(np.linalg.norm(spline(x,2)-original(x,2)) for x in (a,b)))))
    meta=dict(origin_lon_lat=[118.8145,31.8885],projection='Spherical equirectangular R=6378137 m',
        source='Same campus road sequence with locally widened smooth turns',route_length_m=float(s[-1]),points=len(s),ds_m=.2,
        source_plan_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),
        before_length_m=float(u[-1]),before_max_curvature_1pm=float(max(abs(old['curvature_1pm']))),
        max_abs_curvature_1pm=float(max(abs(k))),dense_check_max_curvature_1pm=float(max(abs(dense_k))),
        minimum_radius_m=float(1/max(abs(dense_k))),vehicle_curvature_limit_1pm=float(kmax),
        max_geometric_steer_deg=float(np.rad2deg(max(req))),steer_limit_deg=35,
        maximum_same_parameter_displacement_m=float(max(np.linalg.norm(evaluate(dense)-original(dense),axis=1))),
        endpoint_difference_m=float(max(np.linalg.norm(new[[0,-1]]-xy[[0,-1]],axis=1))),
        method='Local quintic Hermite curves matched through second derivatives; unchanged curves outside four intervals. Dense 0.01 m parameter check; arc-length resampling to 0.2 m.',
        joins=joins,local_access_verified=False)
    (ROOT/'data/campus_route_metadata.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
    print(json.dumps(meta,indent=2))


if __name__=='__main__':main()
