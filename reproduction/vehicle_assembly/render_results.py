"""Render actual saved FEM/ABD states and diagnostic plots; no GPU simulation."""
from common import ROOT,RESEARCH
import json,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
from mesh_render import draw, objects, transform, COLORS, SOURCE

OUT=ROOT/'preview';OUT.mkdir(exist_ok=True)


def read(path):return json.loads(path.read_text())


def load_case(name):
    base=ROOT/'results'/name
    return read(base/'scene_manifest.json'),np.load(base/'geometry.npz'),np.load(base/'snapshots.npz')


def summary_figure():
    case=load_case('obstacle_paper_stiffness')
    fig=plt.figure(figsize=(14,10),facecolor='#f2f5f7')
    fig.text(.04,.955,'HMMWV assembly: collision + suspension + bead coupling',fontsize=21,weight='bold',color='#223742')
    fig.text(.04,.918,'Saved GPU state at 0.060 s | 16 FEM tires, 21 affine bodies | libuipc stock solver',fontsize=12,color='#526b78')
    top=fig.add_axes([.03,.40,.94,.49]);draw(top,objects(case))
    fig.text(.04,.405,'4 mounted tires: E = 1e10 Pa    |    12 loose tires: E = 1e8 Pa    |    178,736 Tet4 elements',fontsize=11,color='#526b78')
    a=fig.add_axes([.035,.075,.48,.28]);draw(a,objects(case,visual=False,mounted_only=True),az=-65,el=18)
    a.set_title('Closed collision proxies + actual FEM surfaces',fontsize=12,loc='left',pad=8)
    b=fig.add_axes([.56,.07,.40,.29])
    tire=np.load(RESEARCH/'prepared/tire_tet4.npz');v=tire['vertices'];f=tire['surface_triangles'];beads=tire['BC_CONN']
    rim=np.load(ROOT/'assets/rim_proxy.npz')
    # Cutaway removes the front half only in this illustration, never in physics.
    mask=v[f][:,:,1].mean(1)>=0
    P,_=draw(b,[(v,f[mask],COLORS['tire']),(rim['vertices'],rim['faces'],COLORS['proxy'])],az=-100,el=15)
    p=v[beads]@P;b.scatter(p[:,0],p[:,1],s=12,c='#f0a32b',zorder=5)
    b.set_title('Cutaway: closed rim + 39 virtual bead anchors',fontsize=12,loc='left',pad=8)
    fig.text(.04,.025,'Chrono-derived replacement; short assembly test, not a completed obstacle traversal or HSC reproduction.',fontsize=10,color='#526b78')
    fig.savefig(OUT/'assembly_overview.png',dpi=150);plt.close(fig)


def diagnostics():
    quarter=read(ROOT/'results/suspension_drive/trajectory.json')
    roll=read(ROOT/'results/vehicle_roll/trajectory.json')
    verify=read(ROOT/'verification.json')
    balance=next(c['suspension_balance'] for c in verify['cases'] if c['label']=='suspension_drive')
    fig,axes=plt.subplots(2,2,figsize=(11,7.5),layout='constrained')
    t=np.array([f['frame']*.005 for f in quarter])
    axes[0,0].plot(t,[-f['suspensions'][0]['travel_m']*1000 for f in quarter],color='#148c9c',lw=2,label='GPU suspension')
    axes[0,0].axhline(balance['static_extension_m']*1000,color='#d5913a',ls='--',label='mg / k')
    axes[0,0].set(title='Suspended wheel: gravity / spring balance',ylabel='Downward travel [mm]');axes[0,0].legend()
    axes[0,1].plot(t,[max(w['max_bead_error_m'] for w in f['wheels'])*1e6 for f in quarter],color='#148c9c',lw=2)
    axes[0,1].set(title='Driven wheel: bead attachment error',ylabel='Maximum error [micrometres]')
    tr=np.array([f['frame']*.005 for f in roll])
    for i in range(4):axes[1,0].plot(tr,[f['suspensions'][i]['travel_m']*1000 for f in roll],label=f'Wheel {i}')
    axes[1,0].set(title='Four-wheel ground roll: suspension travel',ylabel='Travel [mm]');axes[1,0].legend(ncol=2)
    axes[1,1].plot(tr,[min(w['min_z_m'] for w in f['wheels'])*1000 for f in roll],color='#148c9c',lw=2)
    axes[1,1].axhline(0,color='#bd4854',ls='--');axes[1,1].set(title='Four-wheel ground roll: ground clearance',ylabel='Minimum FEM vertex z [mm]')
    for ax in axes.ravel():ax.set_xlabel('Simulated time [s]');ax.grid(alpha=.22)
    fig.suptitle('Measurements from saved dynamic states',fontsize=16)
    fig.savefig(OUT/'validation_curves.png',dpi=150);plt.close(fig)


def suspension_animation():
    case=load_case('suspension_drive');manifest,geo,states=case
    all_frames=[];fixed_limits=None
    for index,frame in enumerate(states['frames']):
        fig,ax=plt.subplots(figsize=(6,5),dpi=90,facecolor='#f2f5f7')
        P,fixed_limits=draw(ax,objects(case,index),az=-105,el=12,limits=fixed_limits)
        # Spring is drawn between the actual virtual attachment positions.
        top=transform(geo['body_v_0'][-1:],states['body_transforms'][index,0])[0]
        bottom=transform(geo['body_v_1'][-1:],states['body_transforms'][index,1])[0]
        u=np.linspace(0,1,120);spring=(1-u[:,None])*bottom+u[:,None]*top
        spring[:,0]+=.035*np.sin(u*20*np.pi)*np.sin(np.pi*u)**.4
        p=spring@P;ax.plot(p[:,0],p[:,1],color='#ce7a2e',lw=1.8)
        ax.set_title(f'Single suspension + driven FEM tire | t = {frame*.005:.3f} s',fontsize=10)
        fig.text(.08,.04,'Actual saved states. Spring curve is a visual guide.',fontsize=9,color='#526b78')
        fig.canvas.draw();all_frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy()))
        plt.close(fig)
    all_frames[0].save(OUT/'suspension_test.gif',save_all=True,append_images=all_frames[1:],duration=90,loop=0)


def contact_figure():
    manifest,geo,states=load_case('obstacle_entry')
    fig,axes=plt.subplots(1,2,figsize=(12,5),facecolor='#f2f5f7')
    limits=None
    for ax,index,title in zip(axes,[0,-1],['Initial state','Contact onset at 0.200 s']):
        meshes=[]
        for i,b in enumerate(manifest['bodies']):
            if b['name'] not in {'wheel_0_rim','loose_0_rim'}:continue
            meshes.append((transform(geo[f'body_v_{i}'],states['body_transforms'][index,i]),geo[f'body_f_{i}'],COLORS['rim']))
        for i in [0,4]:meshes.append((states['tire_vertices'][index,i],geo['tire_faces'],COLORS['tire']))
        P,limits=draw(ax,meshes,az=-90,el=12,limits=limits)
        floor=np.array([[-1.5,-1.55,0.],[.65,-1.55,0.],[.65,-.45,0.],[-1.5,-.45,0.]])@P
        ax.fill(floor[:,0],floor[:,1],color='#dce4e8',zorder=-5)
        ax.set_title(title,fontsize=14)
    fig.suptitle('Mounted tire loading a loose FEM tire',fontsize=18,weight='bold')
    fig.text(.055,.05,'Assembly profile: mounted E = 1e8 Pa, loose E = 1e6 Pa. Actual saved geometry; no displacement scaling.',fontsize=10,color='#526b78')
    fig.subplots_adjust(top=.80,bottom=.15,wspace=.12)
    fig.savefig(OUT/'first_contact.png',dpi=150);plt.close(fig)


if __name__=='__main__':
    summary_figure();diagnostics();suspension_animation();contact_figure()
    print('Wrote',OUT)
