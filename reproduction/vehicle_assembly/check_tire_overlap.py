"""Independent discrete Tet4 overlap audit of the obstacle-entry snapshots.

Conservative AABB/sphere broad phase; exact convex separating-axis narrow phase.
This tests inter-tire volume overlap at saved frames, not continuous trajectories,
self contact, or ABD surfaces. No libuipc imports or collision routines are used.
"""
from common import ROOT
from itertools import combinations
import json,time
import numpy as np
from scipy.spatial import cKDTree

EDGES=np.array(list(combinations(range(4),2)))
FACES=np.array([[0,1,2],[0,1,3],[0,2,3],[1,2,3]])
TOL=1e-9


def overlap_count(a,b):
    alo,ahi=a.min(1),a.max(1);blo,bhi=b.min(1),b.max(1)
    selected=np.flatnonzero(((ahi>blo.min(0)+TOL)&(alo<bhi.max(0)-TOL)).all(1))
    if not len(selected):return 0,0
    ac=(alo+ahi)/2;bc=(blo+bhi)/2
    ar=np.linalg.norm((ahi-alo)/2,axis=1);br=np.linalg.norm((bhi-blo)/2,axis=1)
    candidates=cKDTree(bc).query_ball_point(ac[selected],ar[selected]+br.max()+TOL)
    aa=[];bb=[]
    for i,js in zip(selected,candidates):
        if not js:continue
        js=np.asarray(js)
        keep=((np.minimum(ahi[i],bhi[js])-np.maximum(alo[i],blo[js]))>TOL).all(1)
        js=js[keep];aa.extend([i]*len(js));bb.extend(js)
    if not aa:return 0,0
    aa=np.asarray(aa);bb=np.asarray(bb);count=0
    for start in range(0,len(aa),2048):
        ta=a[aa[start:start+2048]];tb=b[bb[start:start+2048]]
        na=np.cross(ta[:,FACES[:,1]]-ta[:,FACES[:,0]],ta[:,FACES[:,2]]-ta[:,FACES[:,0]])
        nb=np.cross(tb[:,FACES[:,1]]-tb[:,FACES[:,0]],tb[:,FACES[:,2]]-tb[:,FACES[:,0]])
        ea=ta[:,EDGES[:,1]]-ta[:,EDGES[:,0]];eb=tb[:,EDGES[:,1]]-tb[:,EDGES[:,0]]
        cross=np.cross(ea[:,:,None,:],eb[:,None,:,:]).reshape(-1,36,3)
        axes=np.concatenate([na,nb,cross],axis=1);norm=np.linalg.norm(axes,axis=2)
        valid=norm>1e-14;axes/=np.maximum(norm[:,:,None],1e-30)
        pa=np.einsum('nvi,nai->nav',ta,axes);pb=np.einsum('nvi,nai->nav',tb,axes)
        extent=np.minimum(pa.max(2),pb.max(2))-np.maximum(pa.min(2),pb.min(2))
        extent[~valid]=np.inf
        count+=int((extent>TOL).all(1).sum())
    return count,len(aa)


def main():
    start=time.perf_counter();out=ROOT/'results/obstacle_entry'
    unit=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])[None,:,:]
    examples={'identical':(unit,1),'contained':(unit*.1+.1,1),
              'overlapping':(unit+.1,1),'touching':(unit+[1.,0.,0.],0),
              'separated':(unit+[2.,0.,0.],0),
              'aabb_overlap_but_tets_separate':(unit+[.8,.8,0.],0)}
    sanity={name:overlap_count(unit,other)[0]==expected for name,(other,expected) in examples.items()}
    assert all(sanity.values()),sanity
    states=np.load(out/'snapshots.npz');geo=np.load(out/'geometry.npz')
    manifest=json.loads((out/'scene_manifest.json').read_text())
    entries=[]
    for frame,tires in zip(states['frames'],states['tire_vertices']):
        for i,j in combinations(range(len(tires)),2):
            a,b=tires[i],tires[j]
            if not (np.minimum(a.max(0),b.max(0))-np.maximum(a.min(0),b.min(0))>TOL).all():
                count,candidates=0,0
            else:count,candidates=overlap_count(a[geo['tire_tets']],b[geo['tire_tets']])
            entries.append({'frame':int(frame),'a':manifest['wheels'][i]['name'],'b':manifest['wheels'][j]['name'],
                            'aabb_tet_candidates':candidates,'positive_volume_overlaps':count})
    report={'status':'PASS' if not any(e['positive_volume_overlaps'] for e in entries) else 'OVERLAP_FOUND',
            'test':'Independent tetrahedron SAT (4+4 face normals and 36 edge-cross axes)',
            'tolerance_m':TOL,'frames':states['frames'].tolist(),'tire_pair_checks':len(entries),
            'analytic_sanity_cases':sanity,
            'narrow_phase_tet_pairs':sum(e['aabb_tet_candidates'] for e in entries),
            'positive_volume_overlaps':sum(e['positive_volume_overlaps'] for e in entries),
            'excluded_scope':['between saved frames','self contact','ABD meshes'],
            'elapsed_s':time.perf_counter()-start,'entries':entries}
    (out/'independent_overlap_check.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='entries'},indent=2))
    if report['status']!='PASS':raise SystemExit(2)


if __name__=='__main__':main()
