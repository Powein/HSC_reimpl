"""HMMWV assembly using libuipc ABD/FEM, native joints and SoftVertexStitch.

Geometry from Chrono, closed hull/Boolean collision proxies using Manifold. This is a
documented replacement assembly, not the unavailable original HSC scene.
"""
from common import ROOT,RESEARCH,import_uipc
from pathlib import Path
from dataclasses import dataclass
import json,os
import numpy as np
import trimesh as tm

uipc,RUNTIME_ROOT=import_uipc()
from uipc import Engine,World,Scene,view,Logger
import uipc.builtin as builtin
from uipc.geometry import trimesh,tetmesh,ground,label_surface,label_triangle_orient,flip_inward_triangles
from uipc.constitution import (AffineBodyConstitution,StableNeoHookean,ElasticModuli,
    SoftVertexStitch,AffineBodyPrismaticJoint,AffineBodyPrismaticJointLimit,
    AffineBodyRevoluteJoint,AffineBodyDrivingRevoluteJoint,AffineBodyPrismaticJointExternalForce)

ASSETS=ROOT/'assets'
CHRONO=RESEARCH/'upstream/projectchrono__chrono/data/vehicle/hmmwv'

def read_json(path):return json.loads(path.read_text())
def pose(p,rotation=None):
    T=np.eye(4);T[:3,3]=p
    if rotation is not None:T[:3,:3]=rotation
    return T

def load_proxy(name):
    data=np.load(ASSETS/(name+'.npz'))
    return data['vertices'],data['faces']

@dataclass
class Body:
    name:str
    slot:object
    vertices:np.ndarray
    faces:np.ndarray
    anchor_ids:np.ndarray
    contact:object
    mass:float
    def transform(self):return np.asarray(self.slot.geometry().transforms().view()).reshape(-1,4,4)[0].copy()
    def points(self,points=None):
        T=self.transform();v=self.vertices if points is None else points
        return np.asarray(v)@T[:3,:3].T+T[:3,3]

@dataclass
class Wheel:
    name:str
    rim:Body
    tire_slot:object
    ref_vertices:np.ndarray
    tets:np.ndarray
    faces:np.ndarray
    bead_ids:np.ndarray
    ref_volumes:np.ndarray
    def vertices(self):return np.asarray(self.tire_slot.geometry().positions().view()).reshape(-1,3).copy()
    def errors(self):
        v=self.vertices();target=self.rim.points()[self.rim.anchor_ids]
        return np.linalg.norm(v[self.bead_ids]-target,axis=1)

def signed_volumes(v,t):
    return np.linalg.det(np.stack([v[t[:,1]]-v[t[:,0]],v[t[:,2]]-v[t[:,0]],v[t[:,3]]-v[t[:,0]]],axis=2))/6

class Assembly:
    def __init__(self,name,profile='assembly',dt=.005,*,material_overrides=None,
                 solver_options=None,native_workspace=None,strict_mode=False,
                 newton_max_iter=200,motor_ramp_time=0.,material_model='stable_neo_hookean'):
        self.name=name;self.profile=profile;self.dt=dt
        self.motor_ramp_time=float(motor_ramp_time)
        self.material_model=material_model
        if material_model == 'log_neo_hookean':
            from uipc.constitution import LogNeoHookean
            self.tire_constitution=LogNeoHookean()
        elif material_model == 'stable_neo_hookean':
            self.tire_constitution=StableNeoHookean()
        else:
            raise ValueError(f'Unknown tire material model: {material_model}')
        self.bodies=[];self.wheels=[];self.suspensions=[];self.motors=[]
        self.exclusions=[];self.bindings=[]
        config=Scene.default_config()
        config['dt']=dt;config['gravity']=[[0.],[0.],[-9.8]]
        config['contact']['constitution']='ipc'
        config['contact']['d_hat']=.001
        config['contact']['friction']['enable']=True
        config['collision_detection']['method']='stackless_bvh'
        config['newton']['semi_implicit']['enable']=0
        config['newton']['velocity_tol']=.003
        config['newton']['max_iter']=newton_max_iter
        config['line_search']['max_iter']=32
        config['linear_system']['fem_preconditioner']='mas'
        if os.environ.get('HC_LINEAR_SOLVER')=='linear_hsc':
            config['linear_system']['solver']='linear_hsc'
            config['linear_system']['tol_rate']=1e-4
            config['linear_system']['fem_preconditioner']='diag'
            config['linear_system']['hsc']['max_rank']=int(os.environ.get('HC_HSC_RANK','128'))
            config['linear_system']['hsc']['neumann_order']=5
            config['linear_system']['hsc']['omega']=.7
            config['linear_system']['hsc']['adaptive_abd_inverse']=int(os.environ.get('HC_HSC_ADAPTIVE_ABD','1'))
        if solver_options:
            config['linear_system']['solver']='linear_hsc'
            config['linear_system']['tol_rate']=1e-4
            config['linear_system']['fem_preconditioner']='diag'
            config['linear_system']['hsc'].update(solver_options)
        config['extras']['strict_mode']['enable']=int(strict_mode)
        self.config=config
        self.scene=Scene(config)
        self.scene.contact_tabular().default_model(.6,1e10)
        self.virtual_ce=self.scene.contact_tabular().create('virtual_attachment_vertices')
        self.scene.contact_tabular().insert(self.virtual_ce,self.scene.contact_tabular().default_element(),0.,1e10,False)
        self.scene.contact_tabular().insert(self.virtual_ce,self.virtual_ce,0.,1e10,False)
        self.native_workspace=Path(native_workspace) if native_workspace else RUNTIME_ROOT/('hc_assembly_'+name)
        self.engine=Engine('cuda',str(self.native_workspace))
        self.world=World(self.engine)
        self.ground_slot=self.scene.objects().create('ground').geometries().create(ground(0.,np.array([0.,0.,1.])))[0]
        self.materials={'mounted_E':1e10 if profile=='paper_stiffness' else 1e8,
                        'loose_E':1e8 if profile=='paper_stiffness' else 1e6,
                        'poisson':.3,'density':1000.,'abd_kappa':1e9,'bead_k':1e8}
        if material_overrides:
            unknown=set(material_overrides)-set(self.materials)
            if unknown:raise ValueError(f'Unknown material properties: {unknown}')
            self.materials.update(material_overrides)
        self.callbacks=[]

    def add_body(self,name,v,f,transform,mass,anchors=None,fixed=False):
        anchors=np.asarray(anchors if anchors is not None else [],dtype=float).reshape(-1,3)
        anchor_ids=np.arange(len(v),len(v)+len(anchors),dtype=np.int32)
        all_v=np.concatenate([v,anchors])
        mesh=trimesh(all_v,np.asarray(f,dtype=np.int32))
        label_surface(mesh)
        # Virtual attachment vertices contribute no surface/contact or volume.
        if len(anchors):
            attr=mesh.vertices().find(builtin.is_surf)
            view(attr)[anchor_ids]=0
        volume=tm.Trimesh(v,f,process=False).volume
        assert volume>0
        AffineBodyConstitution().apply_to(mesh,self.materials['abd_kappa'],mass/volume)
        view(mesh.transforms())[0]=transform
        if fixed:view(mesh.instances().find(builtin.is_fixed))[0]=1
        ce=self.scene.contact_tabular().create(name)
        ce.apply_to(mesh)
        self.scene.contact_tabular().insert(self.virtual_ce,ce,0.,1e10,False)
        if len(anchors):
            ids=mesh.vertices().create(builtin.contact_element_id,ce.id())
            view(ids)[anchor_ids]=self.virtual_ce.id()
        slot=self.scene.objects().create(name).geometries().create(mesh)[0]
        body=Body(name,slot,all_v,np.asarray(f),anchor_ids,ce,mass)
        self.bodies.append(body)
        return body

    def exclude(self,a,b,reason):
        self.scene.contact_tabular().insert(a.contact,b.contact,0.,1e10,False)
        self.exclusions.append({'a':a.name,'b':b.name,'reason':reason})

    def stitch(self,name,a_slot,b_slot,pairs,k,rest_length=0.):
        geom=SoftVertexStitch().create_geometry((a_slot,b_slot),np.asarray(pairs,dtype=np.int32),k,rest_length)
        slot=self.scene.objects().create(name).geometries().create(geom)[0]
        self.bindings.append({'name':name,'pairs':len(pairs),'stiffness_N_per_m':k,'rest_length_m':rest_length})
        return slot

    def add_wheel(self,name,transform,mounted=True,fixed_rim=False):
        d=np.load(RESEARCH/'prepared/tire_tet4.npz')
        v,t,f,beads=d['vertices'],d['tetrahedra'],d['surface_triangles'],d['BC_CONN']
        rv,rf=load_proxy('rim_proxy')
        wheel_mass=float(read_json(CHRONO/'wheel/HMMWV_Wheel.json')['Mass'])
        rim=self.add_body(name+'_rim',rv,rf,transform,wheel_mass,anchors=v[beads],fixed=fixed_rim)
        world_v=v@transform[:3,:3].T+transform[:3,3]
        tire=tetmesh(world_v,t.astype(np.int32))
        label_surface(tire);label_triangle_orient(tire);tire=flip_inward_triangles(tire)
        E=self.materials['mounted_E'] if mounted else self.materials['loose_E']
        self.tire_constitution.apply_to(tire,ElasticModuli.youngs_poisson(E,self.materials['poisson']),self.materials['density'])
        tire_ce=self.scene.contact_tabular().create(name+'_tire');tire_ce.apply_to(tire)
        self.scene.contact_tabular().insert(self.virtual_ce,tire_ce,0.,1e10,False)
        slot=self.scene.objects().create(name+'_tire').geometries().create(tire)[0]
        self.stitch(name+'_bead',rim.slot,slot,np.column_stack([rim.anchor_ids,beads]),self.materials['bead_k'])
        wheel=Wheel(name,rim,slot,world_v,t,f,beads,signed_volumes(world_v,t))
        self.wheels.append(wheel)
        return wheel

    def add_suspension(self,name,chassis,spindle,wheel,axis_world,spring_chassis_id,spring_spindle_id,
                       spring_k=45374.0375,spring_rest=.35,damping=4753.8923,motor_speed=0.):
        p=np.asarray(axis_world)
        pr=AffineBodyPrismaticJoint().create_geometry(np.array([p]),np.array([p+[0,0,.2]]),
                 [chassis.slot],[0],[spindle.slot],[0],[1e5])
        AffineBodyPrismaticJointLimit().apply_to(pr,-.15,.15,100.)
        AffineBodyPrismaticJointExternalForce().apply_to(pr,0.)
        view(pr.edges().find('external_force/is_constrained'))[:]=1
        pr_obj=self.scene.objects().create(name+'_prismatic')
        pr_slot=pr_obj.geometries().create(pr)[0]
        self.stitch(name+'_spring',chassis.slot,spindle.slot,[[spring_chassis_id,spring_spindle_id]],spring_k,spring_rest)
        center=wheel.rim.transform()[:3,3]
        a,b=center+[0,-.10,0],center+[0,.10,0]
        rev=AffineBodyRevoluteJoint().create_geometry(np.array([a]),np.array([b]),
                    [spindle.slot],[0],[wheel.rim.slot],[0],[1e5])
        if motor_speed:
            AffineBodyDrivingRevoluteJoint().apply_to(rev,1e4)
            view(rev.edges().find('driving/is_constrained'))[:]=1
        rev_obj=self.scene.objects().create(name+'_revolute')
        rev_slot=rev_obj.geometries().create(rev)[0]
        def local(body,points):
            T=body.transform();return (np.asarray(points)-T[:3,3])@np.linalg.inv(T[:3,:3]).T
        record={'name':name,'chassis':chassis,'spindle':spindle,'wheel':wheel,
                'pr_slot':pr_slot,'rev_slot':rev_slot,'k':spring_k,'rest':spring_rest,'c':damping,
                'spring_ids':(spring_chassis_id,spring_spindle_id),
                'pr_l':local(chassis,[p,p+[0,0,.2]]),'pr_r':local(spindle,[p,p+[0,0,.2]]),
                'rev_l':local(spindle,[a,b]),'rev_r':local(wheel.rim,[a,b]),
                'last_d':0.,'motor_speed':motor_speed}
        self.suspensions.append(record)
        def damp(info):
            geo=info.geo_slots()[0].geometry()
            d=float(np.asarray(geo.edges().find('distance').view()).ravel()[0])
            vel=(d-record['last_d'])/info.dt()
            view(geo.edges().find('external_force'))[:]=-damping*vel
            record['last_d']=d
        self.scene.animator().insert(pr_obj,damp);self.callbacks.append(damp)
        if motor_speed:
            def motor(info):
                geo=info.geo_slots()[0].geometry()
                t=info.dt()*info.frame();ramp=self.motor_ramp_time
                phase=t if ramp<=0 else t*t/(2*ramp) if t<ramp else t-ramp/2
                view(geo.edges().find('aim_angle'))[:]=motor_speed*phase
            self.scene.animator().insert(rev_obj,motor);self.callbacks.append(motor)
        self.exclude(chassis,spindle,'prismatic guide and slider are joined, with intentional proxy overlap')
        self.exclude(spindle,wheel.rim,'revolute spindle occupies the hub cavity')
        return record

    def init(self):
        self.world.init(self.scene)
        if not self.world.is_valid():raise RuntimeError('libuipc initialization failed; inspect sanity-check report')

    def advance(self):
        self.world.advance();self.world.retrieve()
        if not self.world.is_valid():raise RuntimeError('libuipc world became invalid')

    def metrics(self):
        data={'frame':self.world.frame(),'wheels':[],'suspensions':[]}
        for w in self.wheels:
            v=w.vertices()
            data['wheels'].append({'name':w.name,'max_bead_error_m':float(w.errors().max()),
                'min_z_m':float(v[:,2].min()),'min_tet_volume_ratio':float((signed_volumes(v,w.tets)/w.ref_volumes).min()),
                'rim_center':w.rim.transform()[:3,3].tolist()})
        for s in self.suspensions:
            c,b=s['chassis'],s['spindle'];lp=c.points(s['pr_l']);rp=b.points(s['pr_r'])
            axis=lp[1]-lp[0];axis/=np.linalg.norm(axis)
            delta=rp[0]-lp[0];perp=delta-np.dot(delta,axis)*axis
            i,j=s['spring_ids'];length=float(np.linalg.norm(c.points()[i]-b.points()[j]))
            rev_delta=b.points(s['rev_l'])-s['wheel'].rim.points(s['rev_r'])
            data['suspensions'].append({'name':s['name'],'travel_m':float(np.dot(delta,axis)),
                'reported_joint_distance_m':float(np.asarray(s['pr_slot'].geometry().edges().find('distance').view()).ravel()[0]),
                'prismatic_lateral_error_m':float(np.linalg.norm(perp)),
                'revolute_anchor_error_m':float(np.linalg.norm(rev_delta,axis=1).max()),
                'spring_length_m':length,'spring_force_N':s['k']*(length-s['rest']),
                'spindle_center':b.transform()[:3,3].tolist()})
        return data

    def export_manifest(self,path):
        result={'model':'Chrono-derived replacement with explicit bead attachment; NOT paper-exact',
            'profile':self.profile,'config':self.config,'materials':self.materials,'material_model':self.material_model,
            'bodies':[{'name':b.name,'mass_kg':b.mass,'transform':b.transform().tolist(),
                       'vertices':len(b.vertices),'surface_triangles':len(b.faces),'virtual_anchors':len(b.anchor_ids)} for b in self.bodies],
            'wheels':[{'name':w.name,'vertices':len(w.ref_vertices),'tets':len(w.tets),'bead_nodes':len(w.bead_ids),
                       'tire_mass_kg':float(w.ref_volumes.sum()*self.materials['density'])} for w in self.wheels],
            'suspensions':[{'name':s['name'],'spring_k_N_m':s['k'],'spring_rest_m':s['rest'],
                            'damping_N_s_m':s['c'],'travel_limits_m':[-.15,.15],
                            'motor_rad_s':s['motor_speed']} for s in self.suspensions],
            'contact_exclusions':self.exclusions,'bindings':self.bindings}
        path.write_text(json.dumps(result,indent=2))

def build_vehicle(profile='assembly',obstacles=0,motor_speed=0.,name='vehicle',start_x=-3.,**options):
    sim=Assembly(name,profile,**options)
    vehicle=read_json(CHRONO/'vehicle/HMMWV_Vehicle.json')
    axle_data=[read_json(CHRONO/'suspension/HMMWV_DoubleWishboneFront.json'),read_json(CHRONO/'suspension/HMMWV_DoubleWishboneRear.json')]
    points=[]
    for axle,data in zip(vehicle['Axles'],axle_data):
        p=np.array(axle['Suspension Location'])+np.array(data['Spindle']['COM'])
        for side in [-1,1]:q=p.copy();q[1]=side*abs(p[1]);points.append(q)
    p0=np.array([float(start_x),0.,.505])
    cv,cf=load_proxy('chassis_proxy')
    mass=read_json(CHRONO/'chassis/HMMWV_Chassis.json')['Components'][0]['Mass']
    anchors=np.array(points)+[0,0,.35]
    chassis=sim.add_body('chassis',cv,cf,pose(p0),mass,anchors=anchors)
    sv,sf=load_proxy('spindle_proxy')
    for i,q in enumerate(points):
        center=q+p0
        wheel=sim.add_wheel('wheel_'+str(i),pose(center),True)
        # Slider and spindle share one affine body, as in the supplement.
        spindle=sim.add_body('spindle_'+str(i),sv,sf,pose(center),34.155,anchors=[[0,0,0]])
        k=45374.0375
        rest=.35+(mass*9.8/4)/k
        sim.add_suspension('suspension_'+str(i),chassis,spindle,wheel,center,
                           int(chassis.anchor_ids[i]),int(spindle.anchor_ids[0]),
                           spring_k=k,spring_rest=rest,motor_speed=motor_speed)
    flat=np.array([[1,0,0],[0,0,-1],[0,1,0.]])
    tire=np.load(RESEARCH/'prepared/tire_tet4.npz')['vertices']
    h=-float((tire@flat.T)[:,2].min())+.004
    for i in range(obstacles):
        row,col=divmod(i,3)
        sim.add_wheel('loose_'+str(i),pose([row*1.01,(col-1)*1.01,h],flat),False)
    return sim
