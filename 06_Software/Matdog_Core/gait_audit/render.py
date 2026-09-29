"""Deterministic canonical collision-mesh engineering views; no mesh edits."""
import csv,json,sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import numpy as np
from core import ROOT
sys.path.insert(0,str(ROOT/'06_Software/Matdog_Core/pose_audit'))
from model import Model,LEGS
OUT=ROOT/'09_Logs/Validation_Reports/G4_Gait_Envelope'
COLORS=['#167b92','#3b94a8','#825fbb','#a280c5']
def main():
 data=json.loads((OUT/'full_cases.json').read_text())['cases'];model=Model();(OUT/'views').mkdir(exist_ok=True)
 for kind,name in ((0,'walk'),(1,'trot')):
  r=next(r for r in data if r['parameters']['type']==kind and r['parameters']['height_m']==.1 and r['parameters']['advance_x_m']==.01 and r['parameters']['lift_m']==.01)
  frames=r['frames'];chosen=[min(frames,key=lambda f:abs(f['phase']-s)) for s in (1.125,1.375,1.625,1.875)]
  for axes,label in (((0,2),'side'),((0,1),'top')):
   fig,axs=plt.subplots(2,2,figsize=(11,7),layout='constrained');fig.suptitle(f'MATDOG {name.upper()} — canonical collision meshes · {label} view\n100 mm body · 10 mm advance/cycle · 10 mm lift · OFFLINE MODEL',fontsize=13)
   for ax,f in zip(axs.flat,chosen):
    tf=model.fk(f['q'],np.array(f['body']))
    for link,mesh in model.meshes.items():
     v=mesh.vertices@tf[link][:3,:3].T+tf[link][:3,3];faces=(v[:,axes]*1000)[mesh.faces]
     leg=next((l for l in LEGS if link.startswith(l+'_')),None);color='#aeb9c1' if leg is None else COLORS[LEGS.index(leg)]
     if leg and leg not in f['active_feet']:color='#dc7d2d'
     ax.add_collection(PolyCollection(faces,facecolors=color,edgecolors='none',alpha=.7,rasterized=True))
    contact=np.array(f['contacts_world_m'])*1000;com=np.array(f['com_world_m'])*1000
    ax.scatter(contact[:,axes[0]],contact[:,axes[1]],s=25,c=['#188348' if l in f['active_feet'] else '#dc7d2d' for l in LEGS],zorder=4)
    ax.scatter(com[axes[0]],com[axes[1]],marker='x',color='black',s=45,zorder=5,label='CAD/URDF COM')
    if label=='top' and f['support_polygon']:
     poly=np.array(f['support_polygon'])*1000;poly=np.vstack([poly,poly[0]]);ax.plot(poly[:,0],poly[:,1],color='#188348',lw=1)
    if label=='side':ax.axhline(0,color='black',lw=.7)
    ax.set_xlim(-230,240);ax.set_ylim((-25,200) if label=='side' else (-145,145));ax.set_aspect('equal');ax.grid(alpha=.15)
    ax.set_xlabel('World X [mm]');ax.set_ylabel('World '+('Z' if label=='side' else 'Y')+' [mm]')
    ax.set_title(f"Phase {f['phase']%1:.3f} · support {'/'.join(l.upper() for l in f['active_feet'])}",fontsize=10)
   fig.savefig(OUT/'views'/f'{name}_{label}.png',dpi=150,metadata={'Software':'MATDOG G4 deterministic render'});plt.close(fig)
  fig,axs=plt.subplots(3,1,figsize=(10,7),sharex=True,layout='constrained');phase=np.array([f['phase']-1 for f in frames]);mask=np.array([[l in f['active_feet'] for l in LEGS] for f in frames])
  for i,l in enumerate(LEGS):axs[0].step(phase,i+.7*mask[:,i],where='post',color=COLORS[i],label=l.upper())
  axs[0].set_yticks(range(4),[l.upper() for l in LEGS]);axs[0].set_ylabel('Contact phase');axs[0].legend(ncol=4)
  axs[1].plot(phase,[f['support_margin_m']*1000 if f['support_margin_m'] is not None else np.nan for f in frames],color='#167b92');axs[1].axhline(0,color='black',lw=.5);axs[1].set_ylabel('Model support margin [mm]')
  axs[2].plot(phase,mask.sum(axis=1),color='#167b92');axs[2].set_ylabel('Declared support count');axs[2].set_xlabel('Normalized cycle phase')
  for ax in axs:ax.grid(alpha=.2)
  fig.suptitle(f'{name.upper()} — '+('quasi-static support study' if kind==0 else 'dynamic stability NOT YET PROVEN'));fig.savefig(OUT/'views'/f'{name}_phase.png',dpi=150,metadata={'Software':'MATDOG G4 deterministic render'});plt.close(fig)
  with (OUT/f'{name}_cycle.csv').open('w') as file:
   writer=csv.writer(file);writer.writerow(['phase',*[f'{l}_{j}_{quantity}' for quantity in ('rad','rad_s','rad_s2') for l in LEGS for j in ('hip','upper','lower')],*[f'{l}_{axis}_world_m' for l in LEGS for axis in ('x','y','z')],'body_x_m','body_y_m','body_z_m','support_margin_m','classification'])
   for f in frames:writer.writerow([f['phase'],*f['q'],*f['qdot'],*f['qddot'],*np.array(f['contacts_world_m']).ravel(),*np.array(f['body'])[:3,3],f['support_margin_m'],';'.join(f['classification'])])
 print('Rendered six engineering views and two complete cycle CSV files')
if __name__=='__main__':main()
