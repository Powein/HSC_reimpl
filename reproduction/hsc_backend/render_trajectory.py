"""Render saved native HSC geometry without running any simulation."""
from pathlib import Path
import argparse, json, sys

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'vehicle_assembly'))
from render_results import draw, objects, plt, Image, np


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('result',type=Path)
    args=parser.parse_args()
    base=args.result.resolve()
    manifest=json.loads((base/'scene_manifest.json').read_text())
    status=json.loads((base/'status.json').read_text())
    case=(manifest,np.load(base/'geometry.npz'),np.load(base/'snapshots.npz'))
    states=case[2]; dt=manifest['config']['dt']
    preview=base/'preview'; preview.mkdir(exist_ok=True)
    fig,axes=plt.subplots(1,2,figsize=(13,6),facecolor='#f2f5f7')
    limits=None
    for ax,index in zip(axes,[0,-1]):
        _,limits=draw(ax,objects(case,index),az=-76,el=19,limits=limits)
        ax.set_title(f"Frame {states['frames'][index]} | {states['frames'][index]*dt:.3f} s",fontsize=14)
    fig.suptitle('Native HSC + IPC: saved vehicle trajectory',fontsize=19,weight='bold')
    fig.text(.04,.10,f"Mounted tires E={manifest['materials']['mounted_E']:.0e} Pa | loose tires E={manifest['materials']['loose_E']:.0e} Pa",fontsize=11)
    fig.text(.04,.055,f"{status['status']} | Chrono replacement assets | actual deformation, no displacement scaling",fontsize=10,color='#526b78')
    fig.subplots_adjust(top=.83,bottom=.16,wspace=.04)
    fig.savefig(preview/'trajectory.png',dpi=150);plt.close(fig)
    images=[]; limits=None
    for index,frame in enumerate(states['frames']):
        fig,ax=plt.subplots(figsize=(9,5.5),dpi=100,facecolor='#f2f5f7')
        _,limits=draw(ax,objects(case,index),az=-76,el=19,limits=limits)
        ax.set_title(f'Native HSC + IPC | frame {frame} | {frame*dt:.3f} s',fontsize=13)
        fig.text(.06,.04,'Saved physical states. Chrono replacement assets.',fontsize=10,color='#526b78')
        fig.canvas.draw()
        images.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy()))
        plt.close(fig)
    images[0].save(preview/'trajectory.gif',save_all=True,append_images=images[1:],duration=180,loop=0)
    print(preview)


if __name__=='__main__':main()
