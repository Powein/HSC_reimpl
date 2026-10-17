"""Audit Chrono geometry and prepare a documented static replacement scene.

This does NOT simulate contact. C3D10 -> Tet4 uses the four corner nodes and
discards quadratic shape functions; the original mesh is retained unchanged.
"""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent))
import bootstrap
import numpy as np

ASSETS = ROOT/'upstream/projectchrono__chrono/data/vehicle/hmmwv'
OUT = ROOT/'prepared'


def read_obj(path):
    vertices, triangles = [], []
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        words = line.split()
        if not words:
            continue
        if words[0] == 'v':
            vertices.append([float(x) for x in words[1:4]])
        elif words[0] == 'f':
            ids = [int(x.split('/')[0]) for x in words[1:]]
            ids = [i-1 if i > 0 else len(vertices)+i for i in ids]
            triangles.extend([[ids[0],ids[j],ids[j+1]] for j in range(1,len(ids)-1)])
    v = np.asarray(vertices, dtype=float)
    f = np.asarray(triangles, dtype=np.int64)
    return v, f


def surface_stats(v, f):
    # OBJ files sometimes split identical vertices at material/normal seams.
    wv, inv = np.unique(np.round(v, 10), axis=0, return_inverse=True)
    wf = inv[f]
    edges = np.sort(np.concatenate([wf[:,[0,1]],wf[:,[1,2]],wf[:,[2,0]]]), axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    areas = np.linalg.norm(np.cross(v[f[:,1]]-v[f[:,0]], v[f[:,2]]-v[f[:,0]]),axis=1)/2
    return {'vertices':len(v), 'triangles':len(f), 'welded_vertices':len(wv),
            'bounds_min_m':v.min(axis=0).tolist(),'bounds_max_m':v.max(axis=0).tolist(),
            'boundary_edges_after_weld':int(np.count_nonzero(counts==1)),
            'nonmanifold_edges_after_weld':int(np.count_nonzero(counts>2)),
            'zero_area_triangles':int(np.count_nonzero(areas<1e-14)),
            'watertight_edge_incidence_only':bool(np.all(counts==2)),
            'self_intersections_checked':False}


def read_abaqus(path):
    nodes, elements, sets = {}, [], {}
    mode, set_name = '', None
    for line in path.read_text().splitlines():
        line=line.strip()
        if not line or line.startswith('**'):
            continue
        if line.startswith('*'):
            header=line.upper()
            if header.startswith('*NODE,'):
                mode='node'
            elif header.startswith('*ELEMENT,'):
                assert 'TYPE=C3D10' in header
                mode='element'
            elif header.startswith('*NSET,'):
                mode='set'
                set_name=next(x.split('=')[1] for x in header.split(',') if x.strip().startswith('NSET='))
                sets.setdefault(set_name, [])
            else:
                mode=''
            continue
        words=[x.strip() for x in line.split(',') if x.strip()]
        if mode=='node':
            nodes[int(words[0])]=[float(x) for x in words[1:4]]
        elif mode=='element':
            assert len(words)==11, 'Unexpected multiline C3D10 record'
            elements.append([int(x) for x in words[1:]])
        elif mode=='set':
            sets[set_name].extend(int(x) for x in words)
    return nodes, np.asarray(elements,dtype=np.int64),sets


def boundary(tets):
    # Positive tet orientation -> outward oriented boundary faces.
    f=np.concatenate([tets[:,[1,2,3]],tets[:,[0,3,2]],tets[:,[0,1,3]],tets[:,[0,2,1]]])
    _, indices, counts=np.unique(np.sort(f,axis=1),axis=0,return_index=True,return_counts=True)
    assert counts.max()<=2
    return f[indices[counts==1]]


def write_obj(path,objects):
    with path.open('w',encoding='utf-8') as stream:
        stream.write('# Static replacement scene; not a simulated frame. Units: metres.\n')
        stream.write('# Derived from Project Chrono assets; see THIRD_PARTY_NOTICES.md.\n')
        offset=1
        for name,v,f in objects:
            stream.write('o '+name+'\n')
            for p in v:stream.write('v %.9g %.9g %.9g\n'%tuple(p))
            for t in f:stream.write('f %d %d %d\n'%tuple(t+offset))
            offset+=len(v)


def write_msh(path,v,t):
    with path.open('w') as stream:
        stream.write('$MeshFormat\n2.2 0 8\n$EndMeshFormat\n$Nodes\n%d\n'%len(v))
        for i,p in enumerate(v,1):stream.write('%d %.12g %.12g %.12g\n'%(i,*p))
        stream.write('$EndNodes\n$Elements\n%d\n'%len(t))
        for i,e in enumerate(t,1):stream.write('%d 4 0 %d %d %d %d\n'%(i,*(e+1)))
        stream.write('$EndElements\n')


def main():
    OUT.mkdir(exist_ok=True)
    audit={'scope':'Geometry structure only; no contact simulation or self-intersection certificate.', 'obj':{}}
    meshes={}
    for file in ASSETS.glob('*.obj'):
        v,f=read_obj(file)
        audit['obj'][file.name]=surface_stats(v,f)
        meshes[file.stem]=(v,f)
    nodes,elements,sets=read_abaqus(ASSETS/'tire/HMMWV_lugged_coarse.inp')
    corner_ids=np.unique(elements[:,:4])
    mapping={int(k):i for i,k in enumerate(corner_ids)}
    v=np.array([nodes[int(i)] for i in corner_ids])
    t=np.array([[mapping[int(i)] for i in row[:4]] for row in elements],dtype=np.int64)
    vol=np.linalg.det(np.stack([v[t[:,1]]-v[t[:,0]],v[t[:,2]]-v[t[:,0]],v[t[:,3]]-v[t[:,0]]],axis=2))/6
    neg=vol<0
    t[neg,0],t[neg,1]=t[neg,1].copy(),t[neg,0].copy()
    assert np.min(np.abs(vol))>1e-15
    f=boundary(t)
    corner_sets={k:np.array([mapping[int(i)] for i in values if int(i) in mapping],dtype=np.int64) for k,values in sets.items()}
    audit['fea']={'source_node_count':len(nodes),'source_element_type':'C3D10',
        'source_elements':len(elements),'tet4_corner_vertices':len(v),'tet4_count':len(t),
        'flipped_tet_orientations':int(np.count_nonzero(neg)),
        'min_absolute_volume_before_scaling_m3':float(np.min(np.abs(vol))),
        'original_total_linearized_volume_m3':float(np.sum(np.abs(vol))),
        'original_bounds_min_m':v.min(axis=0).tolist(),'original_bounds_max_m':v.max(axis=0).tolist(),
        'node_sets_original':{k:len(s) for k,s in sets.items()},
        'node_sets_retained_corners':{k:len(s) for k,s in corner_sets.items()},
        'conversion':'First four C3D10 nodes -> linear Tet4; quadratic nodes discarded; orientation repaired.',
        'surface':surface_stats(v,f)}
    # Native FE tire axle is X. Rotate around +Z to align axle with Y.
    rotation=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
    radius=float(np.linalg.norm(v[:,[1,2]],axis=1).max())
    scale=0.47/radius
    v=v@rotation.T*scale
    audit['fea']['prepared_radius_m']=0.47
    audit['fea']['uniform_scale']=scale
    np.savez_compressed(OUT/'tire_tet4.npz',vertices=v,tetrahedra=t,surface_triangles=f,
                        source_corner_node_ids=corner_ids,**corner_sets)
    write_msh(OUT/'tire_tet4.msh',v,t)
    write_obj(OUT/'tire_tet4_surface.obj',[('tire',v,f)])

    # Vehicle geometry: Chrono axle + spindle offsets; body height chosen for
    # unloaded wheels 4 mm above the plane. This is an assembly assumption.
    front=json.loads((ASSETS/'suspension/HMMWV_DoubleWishboneFront.json').read_text())
    rear=json.loads((ASSETS/'suspension/HMMWV_DoubleWishboneRear.json').read_text())
    car=json.loads((ASSETS/'vehicle/HMMWV_Vehicle.json').read_text())
    car_pos=np.array([-3.0,0.,0.50])
    instances=[]
    objects=[]
    cv,cf=meshes['hmmwv_chassis_simple']
    objects.append(('chassis_visual',cv+car_pos,cf))
    instances.append({'name':'chassis','visual_source':'hmmwv_chassis_simple.obj',
                      'collision_candidate':'hmmwv_chassis_col.obj', 'translation_m':car_pos.tolist(),
                      'dynamics_intent':'ABD; collision/mass proxy requires preparation'})
    rv,rf=meshes['hmmwv_rim']
    # Rim meshes are used for appearance only; connection/proxy needs checking.
    rim_radius=float(np.linalg.norm(rv[:,[0,2]],axis=1).max())
    rv=rv*(0.27/rim_radius)
    for a,susp in zip(car['Axles'],[front,rear]):
        p=np.array(a['Suspension Location'])+np.array(susp['Spindle']['COM'])+car_pos
        for side in [-1,1]:
            q=p.copy();q[1]=side*abs(p[1])
            name=f'mounted_{len(instances)}'
            objects.extend([(name+'_tire',v+q,f),(name+'_rim',rv+q,rf)])
            instances.append({'name':name,'geometry':'tire_tet4.npz','translation_m':q.tolist(),
                              'rotation_matrix':np.eye(3).tolist(),'young_modulus_Pa':1e10,
                              'dynamics_intent':'FEM tire + ABD rim; joint attachment pending'})
    # Twelve horizontal loose wheels form an obstacle patch in front of the car.
    # Rotate about X: tire axle Y -> vertical Z; maintain positive orientation.
    flat=np.array([[1,0,0],[0,0,-1],[0,1,0.]])
    fv=v@flat.T
    fr=rv@flat.T
    height=-float(min(fv[:,2].min(),fr[:,2].min()))+0.004
    for row in range(4):
        for col in range(3):
            q=np.array([0.0+row*1.01,(col-1)*1.01,height])
            name=f'loose_r{row}_c{col}'
            objects.extend([(name+'_tire',fv+q,f),(name+'_rim',fr+q,rf)])
            instances.append({'name':name,'geometry':'tire_tet4.npz','translation_m':q.tolist(),
                              'rotation_matrix':flat.tolist(),'young_modulus_Pa':1e8,
                              'dynamics_intent':'FEM tire + ABD rim; bead attachment pending'})
    scene={'status':'STATIC_ASSET_ASSEMBLY_NOT_SIMULATED','exact_paper_scene':False,
           'units':'m','up_axis':'Z','vehicle_forward_axis':'+X',
           'source':'Project Chrono HMMWV; modified Tet4 tire mesh',
           'original_paper_reference':'HC.pdf page 14 Fig.19; technical supplement page 3 Fig.3',
           'assumptions':['Chassis translation and obstacle arrangement chosen locally.',
                          'FE tire uniformly scaled to 0.47 m maximum radius.',
                          'Chrono suspension describes double wishbones; paper uses reduced prismatic+spring mechanism.',
                          'No Poisson ratio/density/friction values have been inferred as paper-exact.',
                          'Rims are visualization only; full spokes/hubs and bead coupling not yet prepared.',
                          'Initial intersections between separate tire obstacles excluded by AABB spacing; other pairs not certified.'],
           'instances':instances,'ground':{'type':'half_plane','z':0},
           'intended_contact_engine':'libuipc CUDA IPC + ACCD + friction',
           'required_before_simulation':['Mass/collision proxy preparation for ABD meshes',
               'Tire bead to rim attachment and contact exclusions',
               '4 prismatic/spring suspension assemblies + 4 revolute wheel joints',
               'Material, drive, friction parameters and collision-free initial geometry check']}
    (OUT/'scene_manifest.json').write_text(json.dumps(scene,ensure_ascii=False,indent=2),encoding='utf-8')
    write_obj(OUT/'vehicle_over_tires_static.obj',objects)
    audit['assembly']={'mounted_tires':4,'loose_tires':12,'total_tire_tets':16*len(t),
                       'state':'STATIC_NOT_SIMULATED','complete_collision_check':False}
    (OUT/'asset_audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf-8')
    render(objects)
    print(json.dumps({'obj':audit['obj'],'fea':audit['fea'],'assembly':audit['assembly']},indent=2))


def render(objects):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    fig=plt.figure(figsize=(13,7.5),facecolor='#f3f5f6')
    ax=fig.add_axes([.035,.13,.93,.72])
    ax.set_facecolor('#f3f5f6')
    all_triangles,all_colors=[],[]
    for name,v,f in objects:
        color='#657c64' if 'chassis' in name else '#be8c51' if 'rim' in name else '#333a40'
        triangles=v[f]
        normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
        normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-20)
        light=np.array([-.2,-.6,1.]);light/=np.linalg.norm(light)
        intensity=.45+.55*np.clip(np.abs(normals@light),0,1)
        base=np.array(matplotlib.colors.to_rgb(color))
        colors=np.minimum(1,base[None,:]*intensity[:,None])
        all_triangles.append(triangles)
        all_colors.append(colors)
    # Explicit orthographic projection gives the wide scene its full canvas,
    # without the square clipping region of Matplotlib's 3D axes.
    az,el=np.deg2rad(-57),np.deg2rad(30)
    camera=np.array([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)])
    right=np.array([-np.sin(az),np.cos(az),0.])
    up=np.cross(camera,right)
    projection=np.stack([right,up],axis=1)
    triangles=np.concatenate(all_triangles)
    polygons=triangles@projection
    order=np.argsort((triangles@camera).mean(axis=1))
    ax.add_collection(PolyCollection(polygons[order],facecolors=np.concatenate(all_colors)[order],linewidths=0))
    arrow=np.array([[-4.5,-1.8,.05],[-2.3,-1.8,.05]])@projection
    ax.annotate('',xy=arrow[1],xytext=arrow[0],arrowprops={'arrowstyle':'->','color':'#197a99','lw':2})
    ax.text(arrow[0,0],arrow[0,1]-.14,'Drive direction',fontsize=10,color='#197a99',ha='center')
    lo=polygons.reshape(-1,2).min(axis=0)
    hi=polygons.reshape(-1,2).max(axis=0)
    ax.set(xlim=(lo[0]-.25,hi[0]+.25),ylim=(lo[1]-.45,hi[1]+.25),aspect='equal')
    ax.set_axis_off()
    fig.text(.04,.94,'HMMWV + 12 loose wheels',fontsize=24,weight='bold',color='#243339')
    fig.text(.04,.895,'Static replacement asset assembly | NOT a simulation result',fontsize=13,color='#52656e')
    fig.text(.04,.075,'Source: Project Chrono (BSD-3-Clause) | Tire: C3D10 corners converted to Tet4',fontsize=10,color='#52656e')
    fig.text(.04,.045,'4 mounted stiff FEM tires + 12 softer FEM tires; chassis / rims intended as affine bodies',fontsize=10,color='#52656e')
    fig.savefig(OUT/'scene_preview.png',dpi=160,facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__=='__main__':main()
