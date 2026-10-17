"""Small actual CUDA ABD/FEM contact check, separate from the static car scene.

The stock libuipc solver is used. This is not an HSC verification and does not
validate the tyre scene or friction quantitatively.
"""
from pathlib import Path
import json
import sys
import time

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent))
import bootstrap
from probe_cuda import prepare_runtime
runtime=Path(prepare_runtime())
sys.path.insert(0,str(runtime))
import numpy as np
from uipc import Engine, World, Scene, Logger, view
from uipc.geometry import tetmesh, ground, label_surface, label_triangle_orient, flip_inward_triangles
from uipc.constitution import AffineBodyConstitution, StableNeoHookean, ElasticModuli


def cube(bottom):
    v=np.array([[-.15,-.15,0],[.15,-.15,0],[.15,.15,0],[-.15,.15,0],
                [-.15,-.15,.3],[.15,-.15,.3],[.15,.15,.3],[-.15,.15,.3]])
    v[:,2]+=bottom
    t=np.array([[0,1,2,6],[0,2,3,6],[0,3,7,6],[0,7,4,6],[0,4,5,6],[0,5,1,6]])
    sc=tetmesh(v,t)
    label_surface(sc)
    label_triangle_orient(sc)
    return flip_inward_triangles(sc),v,t


def overlapping_tet_pairs(a,b,t,tolerance=1e-9):
    """Independent separating-axis test for positive-volume tet overlap.

    AABB overlap is only broad phase: tilted/deformed cubes may have overlapping
    boxes while their actual volumes are disjoint. All face normals and all
    cross products of one edge from each tet form the complete SAT axis set.
    """
    faces=np.array([[0,1,2],[0,1,3],[0,2,3],[1,2,3]])
    edges=np.array([[0,1],[0,2],[0,3],[1,2],[1,3],[2,3]])
    count=0
    for ta in a[t]:
        for tb in b[t]:
            if np.any(np.minimum(ta.max(0),tb.max(0))-np.maximum(ta.min(0),tb.min(0))<=tolerance):
                continue
            na=np.cross(ta[faces[:,1]]-ta[faces[:,0]],ta[faces[:,2]]-ta[faces[:,0]])
            nb=np.cross(tb[faces[:,1]]-tb[faces[:,0]],tb[faces[:,2]]-tb[faces[:,0]])
            ea=ta[edges[:,1]]-ta[edges[:,0]]
            eb=tb[edges[:,1]]-tb[edges[:,0]]
            cross=np.cross(ea[:,None,:],eb[None,:,:]).reshape(-1,3)
            axes=np.concatenate([na,nb,cross])
            norm=np.linalg.norm(axes,axis=1)
            axes=axes[norm>1e-14]/norm[norm>1e-14,None]
            pa,pb=ta@axes.T,tb@axes.T
            gap=np.maximum(pa.min(0)-pb.max(0),pb.min(0)-pa.max(0))
            if np.all(gap < -tolerance):count+=1
    return count


def main():
    start=time.perf_counter()
    out=ROOT/'prepared'
    Logger.set_level(Logger.Level.Warn)
    config=Scene.default_config()
    config['dt']=.005
    config['gravity']=[[0.],[0.],[-9.8]]
    config['contact']['constitution']='ipc'
    config['contact']['d_hat']=.001
    config['contact']['friction']['enable']=True
    config['collision_detection']['method']='stackless_bvh'
    config['newton']['semi_implicit']['enable']=0
    config['newton']['velocity_tol']=.001
    config['line_search']['max_iter']=32
    (out/'contact_probe_config.json').write_text(json.dumps(config,indent=2))
    engine=Engine('cuda',str(runtime.parent/'contact_probe_output'))
    world=World(engine)
    scene=Scene(config)
    scene.contact_tabular().default_model(.4,1e8)
    fem,ref,tets=cube(.004)
    StableNeoHookean().apply_to(fem,ElasticModuli.youngs_poisson(1e5,.3),1000.)
    fs=scene.objects().create('fem').geometries().create(fem)[0]
    abd,abd_ref,_=cube(.404)
    AffineBodyConstitution().apply_to(abd,1e8,1000.)
    rs=scene.objects().create('abd').geometries().create(abd)[0]
    scene.objects().create('ground').geometries().create(ground(0.,np.array([0.,0.,1.])))
    world.init(scene)
    records=[]
    for i in range(120):
        world.advance()
        world.retrieve()
        fv=np.asarray(fs.geometry().positions().view()).reshape(-1,3).copy()
        transform=np.asarray(rs.geometry().transforms().view()).reshape(-1,4,4)[0]
        rv=abd_ref@transform[:3,:3].T+transform[:3,3]
        overlap_count=overlapping_tet_pairs(fv,rv,tets)
        volumes=np.linalg.det(np.stack([fv[tets[:,1]]-fv[tets[:,0]],fv[tets[:,2]]-fv[tets[:,0]],fv[tets[:,3]]-fv[tets[:,0]]],axis=2))/6
        records.append({'frame':i+1,'fem_min_z':float(fv[:,2].min()),
                        'abd_min_z':float(rv[:,2].min()),
                        'vertical_aabb_gap':float(rv[:,2].min()-fv[:,2].max()),
                        'overlapping_tet_pairs':overlap_count,
                        'min_fem_signed_tet_volume_m3':float(volumes.min()),
                        'fem_height':float(np.ptp(fv[:,2])),
                        'abd_com_z':float(rv[:,2].mean())})
        if (i+1)%30==0:print('completed',i+1,'frames',flush=True)
    minimum_floor=min(x['fem_min_z'] for x in records)
    minimum_gap=min(x['vertical_aabb_gap'] for x in records)
    final=records[-1]
    overlaps=sum(x['overlapping_tet_pairs'] for x in records)
    min_volume=min(x['min_fem_signed_tet_volume_m3'] for x in records)
    passed=(minimum_floor>=-1e-8 and overlaps==0 and min_volume>0 and
            final['abd_com_z']<.50 and abs(final['fem_height']-.3)>1e-5)
    result={'status':'PASS' if passed else 'CHECK_FAILED','engine':'pyuipc 0.0.28 CUDA',
            'scope':'Two aligned 6-tet cubes: one ABD falling onto one FEM cube above a plane; stock solver, not HSC.',
            'friction_enabled':True,'friction_quantitatively_validated':False,
            'vehicle_scene_simulated':False,'steps':len(records),'dt':config['dt'],
            'minimum_fem_ground_distance_m':minimum_floor,
            'minimum_abd_fem_vertical_aabb_gap_m':minimum_gap,
            'aabb_gap_is_not_a_penetration_test':True,
            'overlapping_tet_pairs_all_frames':overlaps,
            'overlap_check':'Independent tetrahedron separating-axis test, tolerance 1e-9 m; touching permitted.',
            'minimum_fem_signed_tet_volume_m3':min_volume,
            'final_fem_compression_m':.3-final['fem_height'],
            'elapsed_seconds':time.perf_counter()-start,'final':final,'frames':records}
    (out/'contact_probe_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='frames'},indent=2))
    if not passed:raise SystemExit(1)


if __name__=='__main__':main()
