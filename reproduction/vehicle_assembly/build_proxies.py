"""Closed collision proxies derived from Chrono meshes using CoACD/Manifold."""
from common import ROOT,RESEARCH
import json,time,argparse
import numpy as np
import trimesh
import coacd

OUT=ROOT/'assets'
SOURCE=RESEARCH/'upstream/projectchrono__chrono/data/vehicle/hmmwv'

def metrics(mesh):
    return {'vertices':len(mesh.vertices),'triangles':len(mesh.faces),
            'watertight':bool(mesh.is_watertight),'winding_consistent':bool(mesh.is_winding_consistent),
            'volume_m3':float(mesh.volume),'bounds_m':mesh.bounds.tolist(),
            'degenerate_faces':int(np.count_nonzero(mesh.area_faces<1e-14))}

def export(name,mesh,**extra):
    assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume>0
    mesh.export(OUT/(name+'.obj'))
    np.savez_compressed(OUT/(name+'.npz'),vertices=mesh.vertices,faces=mesh.faces,**extra)
    return metrics(mesh)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--coacd',action='store_true');args=parser.parse_args()
    OUT.mkdir(exist_ok=True)
    start=time.perf_counter()
    raw=trimesh.load(SOURCE/'hmmwv_chassis_col.obj',force='mesh',process=True)
    raw.update_faces(raw.nondegenerate_faces());raw.remove_unreferenced_vertices()
    coacd.set_log_level('warn')
    params=dict(threshold=.04,real_metric=True,max_convex_hull=24,preprocess_mode='auto',
                preprocess_resolution=40,resolution=1000,mcts_nodes=12,mcts_iterations=40,
                mcts_max_depth=3,merge=True,decimate=True,max_ch_vertex=64,seed=42)
    if args.coacd:
        parts=coacd.run_coacd(coacd.Mesh(np.asarray(raw.vertices),np.asarray(raw.faces)),**params)
        hulls=[trimesh.Trimesh(v,f,process=True) for v,f in parts]
        for h in hulls:h.fix_normals()
        chassis=trimesh.boolean.union(hulls,engine='manifold')
        method='CoACD convex decomposition + Manifold union'
    else:
        # The CoACD attempt exceeded its 240 s cap on the defective input.
        # Use an explicit, reproducible source hull with cut wheel clearances.
        hulls=[raw.convex_hull]
        vehicle=json.loads((SOURCE/'vehicle/HMMWV_Vehicle.json').read_text())
        data=[json.loads((SOURCE/'suspension/HMMWV_DoubleWishboneFront.json').read_text()),
              json.loads((SOURCE/'suspension/HMMWV_DoubleWishboneRear.json').read_text())]
        cuts=[]
        for axle,suspension in zip(vehicle['Axles'],data):
            p=np.array(axle['Suspension Location'])+np.array(suspension['Spindle']['COM'])
            for side in [-1,1]:
                cut=trimesh.creation.cylinder(radius=.65,height=.90,sections=48)
                cut.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]))
                cut.apply_translation([p[0],side*1.05,p[2]])
                cuts.append(cut)
        chassis=trimesh.boolean.difference([hulls[0],*cuts],engine='manifold')
        method='Chrono collision mesh convex hull with four Boolean wheel-well cuts (Manifold)'
    chassis.remove_unreferenced_vertices()
    chassis.fix_normals()
    result={'source':'Project Chrono HMMWV collision mesh','method':method,
            'coacd_attempt':'240 s timeout; retained logs; optional --coacd retry path',
            'coacd_version':'1.0.14','trimesh_version':trimesh.__version__,'manifold3d_version':'3.5.4',
            'parameters':params,'hull_count':len(hulls),'parts':[metrics(h) for h in hulls]}
    result['chassis']=export('chassis_proxy',chassis)
    # Official Chrono PRIMITIVES fallback retained as a named reduced proxy.
    simple=trimesh.creation.box(extents=[2.,1.,.2]);simple.apply_translation([0.,0.,.1])
    result['chrono_primitive']=export('chassis_chrono_primitive',simple)
    tire=np.load(RESEARCH/'prepared/tire_tet4.npz')
    v=tire['vertices'];beads=tire['BC_CONN'];points=v[beads]
    minrad=float(np.linalg.norm(v[:,[0,2]],axis=1).min())
    faces=tire['surface_triangles']
    edges=np.unique(np.sort(np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]),axis=1),axis=0)
    ea=v[edges[:,0]][:,[0,2]];eb=v[edges[:,1]][:,[0,2]];direction=eb-ea
    u=np.clip(-(ea*direction).sum(1)/np.maximum((direction*direction).sum(1),1e-30),0,1)
    min_edge_radius=float(np.linalg.norm(ea+u[:,None]*direction,axis=1).min())
    # Linearized faces lie inside the circle through their corner nodes. Use
    # the polygonal hole radius, not merely the minimum node radius.
    radius=min_edge_radius-.006
    halfwidth=float(np.abs(points[:,1]).max())-.008
    rim=trimesh.creation.cylinder(radius=radius,height=2*halfwidth,sections=48)
    rim.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]))
    result['rim']=export('rim_proxy',rim,bead_anchor_positions=points,bead_tire_ids=beads)
    result['rim']['radius_m']=radius;result['rim']['halfwidth_m']=halfwidth
    result['bead']={'source':'Chrono BC_CONN','count':len(beads),'radius_m':float(np.linalg.norm(points[:,[0,2]],axis=1).mean()),
                    'rows_y_m':np.unique(np.round(points[:,1],6)).tolist(),
                    'minimum_projected_edge_radius_m':min_edge_radius,
                    'minimum_polygonal_radial_clearance_m':min_edge_radius-radius}
    # Axis/spindle proxies stay inside the hub cavity and never substitute for tires.
    spindle=trimesh.creation.box(extents=[.10,.12,.12])
    result['spindle']=export('spindle_proxy',spindle)
    result['elapsed_s']=time.perf_counter()-start
    (OUT/'proxy_audit.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='parts'},indent=2),flush=True)

if __name__=='__main__':main()
