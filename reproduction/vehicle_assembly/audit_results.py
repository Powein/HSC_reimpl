"""Offline checks of saved states, source hashes and native solver warnings."""
from common import ROOT, RESEARCH
import ast, hashlib, json, re
import numpy as np
import trimesh


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def audit_case(label):
    out=ROOT/'results'/label
    result=read(out/'result.json');manifest=read(out/'scene_manifest.json')
    frames=read(out/'trajectory.json');states=np.load(out/'snapshots.npz')
    log=(ROOT/'logs'/(label+'.log'))
    if label=='bead':log=ROOT/'logs/bead_bound.log'
    lines=log.read_text(encoding='utf-8',errors='replace').splitlines()
    warnings=[line for line in lines if '[warning]' in line.lower()]
    unexpected=[line for line in warnings if ' is clamped to ' not in line]
    errors=[line for line in lines if '[error]' in line.lower() or '[critical]' in line.lower()]
    clamped=[]
    for line in warnings:
        match=re.search(r'clamped to \[?([0-9.eE+\-]+)',line)
        if match:clamped.append(float(match.group(1)))
    bodies=states['body_transforms']
    body_names=[b['name'] for b in manifest['bodies']]
    all_finite=all(bool(np.isfinite(states[k]).all()) for k in states.files)
    checks={'recorded_dynamic_checks':all(result['checks'].values()),
            'latest_status_is_pass':read(out/'status.json')['status']=='PASS',
            'no_native_errors':not errors,'no_unreviewed_native_warnings':not unexpected,
            'saved_states_finite':all_finite,
            'saved_affine_determinants_positive':bool((np.linalg.det(bodies[:,:,:3,:3])>0).all())}
    summary={'label':label,'checks':checks,'requested_contact_kappa':1e10,
             'effective_initial_contact_kappa':sorted(set(clamped)) or [1e10],
             'native_errors':errors,'unreviewed_warnings':unexpected,
             'max_bead_error_um':result['max_bead_error_m']*1e6,
             'minimum_tire_ground_gap_mm':result['minimum_tire_z_m']*1000,
             'minimum_fem_volume_ratio':result['min_tet_volume_ratio'],
             'simulated_seconds':result['steps']*result['dt']}
    motor_errors=[]
    for i,s in enumerate(manifest['suspensions']):
        if not s['motor_rad_s']:continue
        spindle_name='slider_spindle' if result['case']=='suspension' else f'spindle_{i}'
        rim_name='test_wheel_rim' if result['case']=='suspension' else f'wheel_{i}_rim'
        a=bodies[-1,body_names.index(spindle_name),:3,:3]
        b=bodies[-1,body_names.index(rim_name),:3,:3]
        R=np.linalg.solve(a,b)
        angle=np.arctan2(R[0,2],R[0,0])
        target=s['motor_rad_s']*summary['simulated_seconds']
        motor_errors.append(float(abs(np.angle(np.exp(1j*(angle-target))))))
    if motor_errors:
        summary['all_motor_angle_errors_rad']=motor_errors
        checks['all_motor_angles_within_0_02rad']=max(motor_errors)<.02
    if 'chassis' in body_names:
        idx=body_names.index('chassis')
        summary['chassis_translation_m']=(bodies[-1,idx,:3,3]-bodies[0,idx,:3,3]).tolist()
        if motor_errors:checks['positive_vehicle_forward_motion']=summary['chassis_translation_m'][0]>0
    if result['case']=='suspension':
        tire=np.load(RESEARCH/'prepared/tire_tet4.npz');v=tire['vertices'];t=tire['tetrahedra']
        vol=np.linalg.det(np.stack([v[t[:,1]]-v[t[:,0]],v[t[:,2]]-v[t[:,0]],v[t[:,3]]-v[t[:,0]]],axis=2)).sum()/6
        weight=(vol*manifest['materials']['density']+34.155+18.8)*9.8
        spring=frames[-1]['suspensions'][0]['spring_force_N']
        error=float(abs(spring-weight)/weight)
        summary['suspension_balance']={'weight_N':float(weight),'spring_N':spring,'relative_error':error,
            'static_extension_m':float(weight/manifest['suspensions'][0]['spring_k_N_m']),
            'measured_extension_m':-frames[-1]['suspensions'][0]['travel_m']}
        if summary['simulated_seconds']>=.6:checks['spring_weight_balance_within_3_percent']=error<.03
    pairs=[(s['travel_m'],s['reported_joint_distance_m']) for f in frames for s in f['suspensions'] if 'reported_joint_distance_m' in s]
    if pairs:
        summary['max_reported_distance_vs_geometric_travel_error_m']=max(abs(a-b) for a,b in pairs)
        checks['reported_joint_distance_matches_geometry']=summary['max_reported_distance_vs_geometric_travel_error_m']<1e-5
    summary['loose_rim_translations_m']={name:(bodies[-1,i,:3,3]-bodies[0,i,:3,3]).tolist()
                                         for i,name in enumerate(body_names) if name.startswith('loose_')}
    independent=out/'independent_overlap_check.json'
    if independent.exists():
        saved=read(independent)
        summary['independent_tire_overlap_check']={k:saved[k] for k in ['status','frames','tire_pair_checks','narrow_phase_tet_pairs','positive_volume_overlaps','excluded_scope']}
        checks['independent_saved_frame_tire_overlap_check']=saved['status']=='PASS'
    summary['status']='PASS' if all(checks.values()) else 'CHECK_FAILED'
    return summary


def main():
    sources=read(ROOT/'references_manifest.json')
    hashes=[hashlib.sha256((ROOT/f['path']).read_bytes()).hexdigest()==f['sha256'] for f in sources['files']]
    for path in ROOT.glob('*.py'):ast.parse(path.read_text(encoding='utf-8'),filename=str(path))
    proxies={}
    for name in ['chassis_proxy','rim_proxy','spindle_proxy','chassis_chrono_primitive']:
        d=np.load(ROOT/'assets'/(name+'.npz'));mesh=trimesh.Trimesh(d['vertices'],d['faces'],process=False)
        proxies[name]={'closed':bool(mesh.is_watertight),'consistent_normals':bool(mesh.is_winding_consistent),
                       'positive_volume':bool(mesh.volume>0),'nondegenerate':bool((mesh.area_faces>1e-14).all())}
    labels=['bead','suspension_drive','vehicle_roll','obstacle_paper_stiffness','obstacle_entry']
    cases=[audit_case(label) for label in labels if (ROOT/'results'/label/'result.json').exists()]
    checks={'all_reference_hashes_match':bool(hashes) and all(hashes),'reference_downloads_complete':not sources['errors'],
            'closed_valid_proxies':all(all(p.values()) for p in proxies.values()),
            'all_five_required_cases_present':len(cases)==len(labels),
            'all_completed_cases_pass':all(c['status']=='PASS' for c in cases)}
    report={'status':'PASS' if all(checks.values()) else 'CHECK_FAILED','checks':checks,
            'reference_file_count':len(hashes),'proxies':proxies,'cases':cases,
            'scope':'Saved-state/component verification; not a proof of every continuous collision pair or a HSC benchmark.'}
    (ROOT/'verification.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    if not all(checks.values()):raise SystemExit(2)


if __name__=='__main__':main()
