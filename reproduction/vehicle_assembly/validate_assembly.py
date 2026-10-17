"""Actual, bounded GPU tests for beads, suspension and complete assembly."""
from common import ROOT,RESEARCH
import argparse,json,time
import numpy as np
import trimesh as tm
from scene import Assembly,build_vehicle,pose,load_proxy,Logger

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--case',choices=['bead','suspension','vehicle'],required=True)
    parser.add_argument('--steps',type=int,default=60)
    parser.add_argument('--obstacles',type=int,choices=range(13),default=0)
    parser.add_argument('--profile',choices=['assembly','paper_stiffness'],default='assembly')
    parser.add_argument('--motor-speed',type=float,default=0.)
    parser.add_argument('--start-x',type=float,default=-3.)
    parser.add_argument('--label')
    args=parser.parse_args()
    if args.steps<1:parser.error('--steps must be positive')
    label=args.label or args.case
    if not label.replace('_','').replace('-','').isalnum():parser.error('label must be alphanumeric with _ or -')
    out=ROOT/'results'/label;out.mkdir(parents=True,exist_ok=True)
    Logger.set_level(Logger.Level.Warn)
    start=time.perf_counter()
    if args.case=='vehicle':
        sim=build_vehicle(args.profile,args.obstacles,args.motor_speed,label,args.start_x)
    else:
        sim=Assembly(label,args.profile)
        if args.case=='bead':
            sim.add_wheel('test_wheel',pose([0.,0.,.75]),True,True)
        else:
            center=np.array([0.,0.,.75])
            box=tm.creation.box(extents=[.3,.15,.1])
            support=sim.add_body('support',box.vertices,box.faces,pose(center+[0,-.5,.35]),524.13,anchors=[[0,0,0]],fixed=True)
            v,f=load_proxy('spindle_proxy')
            spindle=sim.add_body('slider_spindle',v,f,pose(center),34.155,anchors=[[0,-.5,0]])
            wheel=sim.add_wheel('test_wheel',pose(center),True)
            sim.add_suspension('test_suspension',support,spindle,wheel,center,
                int(support.anchor_ids[0]),int(spindle.anchor_ids[0]),motor_speed=args.motor_speed)
    sim.export_manifest(out/'scene_manifest.json')
    np.savez_compressed(out/'geometry.npz',
         **{f'body_v_{i}':b.vertices for i,b in enumerate(sim.bodies)},
         **{f'body_f_{i}':b.faces for i,b in enumerate(sim.bodies)},
         tire_faces=sim.wheels[0].faces,tire_tets=sim.wheels[0].tets)
    status={'status':'INITIALIZING','case':args.case,'profile':args.profile,'requested_steps':args.steps}
    (out/'status.json').write_text(json.dumps(status,indent=2))
    sim.init()
    frames=[sim.metrics()];snapshots=[]
    def snapshot():
        snapshots.append({'frame':sim.world.frame(),
          'bodies':np.array([b.transform() for b in sim.bodies]),
          'tires':np.array([w.vertices() for w in sim.wheels])})
    snapshot()
    def checkpoint():
        # Preserve useful evidence even if a later step hits the external timeout.
        (out/'trajectory_partial.json').write_text(json.dumps(frames,indent=2))
        np.savez_compressed(out/'snapshots_partial.npz',frames=np.array([s['frame'] for s in snapshots]),
             body_transforms=np.array([s['bodies'] for s in snapshots]),
             tire_vertices=np.array([s['tires'] for s in snapshots]))
    checkpoint()
    print('Initialized',len(sim.bodies),'ABD bodies,',len(sim.wheels),'FEM tires,',len(sim.suspensions),'suspensions',flush=True)
    for step in range(args.steps):
        sim.advance()
        metrics=sim.metrics();frames.append(metrics)
        if (step+1)%5==0 or step+1==args.steps:snapshot()
        status.update(status='RUNNING',completed_steps=step+1,elapsed_s=time.perf_counter()-start,
                      current_max_bead_error_m=max(w['max_bead_error_m'] for w in metrics['wheels']))
        (out/'status.json').write_text(json.dumps(status,indent=2))
        if (step+1)%10==0 or step+1==args.steps:
            checkpoint()
            print('Frame',step+1,'bead error',status['current_max_bead_error_m'],'elapsed',round(status['elapsed_s'],2),flush=True)
    ws=[w for frame in frames for w in frame['wheels']]
    ss=[s for frame in frames for s in frame['suspensions']]
    checks={'world_valid':bool(sim.world.is_valid()),
        'bead_error_under_0_5mm':max(w['max_bead_error_m'] for w in ws)<.0005,
        'positive_fem_tet_volumes':min(w['min_tet_volume_ratio'] for w in ws)>0,
        'tire_ground_clearance':min(w['min_z_m'] for w in ws)>=-1e-6}
    if ss:
        checks.update(prismatic_lateral_error_under_1mm=max(s['prismatic_lateral_error_m'] for s in ss)<.001,
            revolute_anchor_error_under_1mm=max(s['revolute_anchor_error_m'] for s in ss)<.001,
            suspension_travel_within_limits=max(abs(s['travel_m']) for s in ss)<.151)
    force_balance=None
    if args.case=='suspension' and args.steps*sim.dt>=.6:
        s=sim.suspensions[0]
        weight=(s['spindle'].mass+s['wheel'].rim.mass+s['wheel'].ref_volumes.sum()*sim.materials['density'])*9.8
        force=frames[-1]['suspensions'][0]['spring_force_N']
        force_balance={'suspended_weight_N':float(weight),'final_spring_force_N':force,
                       'relative_error':float(abs(force-weight)/weight)}
        checks['suspension_static_balance_within_3_percent']=force_balance['relative_error']<.03
    rotation=None;angle_error=None
    if sim.suspensions and args.motor_speed:
        s=sim.suspensions[0]
        relative=np.linalg.solve(s['spindle'].transform()[:3,:3],s['wheel'].rim.transform()[:3,:3])
        rotation=float(np.arctan2(relative[0,2],relative[0,0]))
        target=args.motor_speed*args.steps*sim.dt
        angle_error=float(abs(np.angle(np.exp(1j*(rotation-target)))))
        checks['wheel_rotation_transmitted']=angle_error<.02
    chassis_dx=None
    if args.case=='vehicle':
        chassis_dx=float(sim.bodies[0].transform()[0,3]-snapshots[0]['bodies'][0,0,3])
    summary={'status':'PASS' if all(checks.values()) else 'CHECK_FAILED',
        'case':args.case,'profile':args.profile,'steps':args.steps,'dt':sim.dt,
        'checks':checks,'max_bead_error_m':max(w['max_bead_error_m'] for w in ws),
        'min_tet_volume_ratio':min(w['min_tet_volume_ratio'] for w in ws),
        'minimum_tire_z_m':min(w['min_z_m'] for w in ws),
        'max_prismatic_lateral_error_m':max((s['prismatic_lateral_error_m'] for s in ss),default=0.),
        'max_revolute_anchor_error_m':max((s['revolute_anchor_error_m'] for s in ss),default=0.),
        'measured_wheel_angle_rad':rotation,'motor_angle_error_rad':angle_error,
        'suspension_force_balance':force_balance,'chassis_displacement_x_m':chassis_dx,
        'elapsed_s':time.perf_counter()-start,
        'not_validated':['HSC solver','paper scene equivalence','full obstacle traversal','all continuous collision pairs independently'],
        'last_frame':frames[-1]}
    (out/'result.json').write_text(json.dumps(summary,indent=2))
    (out/'trajectory.json').write_text(json.dumps(frames,indent=2))
    np.savez_compressed(out/'snapshots.npz',frames=np.array([s['frame'] for s in snapshots]),
         body_transforms=np.array([s['bodies'] for s in snapshots]),tire_vertices=np.array([s['tires'] for s in snapshots]))
    (out/'status.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2),flush=True)
    if not all(checks.values()):raise SystemExit(2)

if __name__=='__main__':main()
