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
def snapshots(frames,duty,kind):
 # Frames chosen from the leg phases, so the views show swings for any duty factor.
 # WALK: mid-swing of RH, RF, LH, LF. TROT: mid-swing of each diagonal pair and an all-stance frame between them.
 def mid(i):
  swing=[f for f in frames if not f['leg_phase'][i][1]]
  return min(swing,key=lambda f:abs((f['leg_phase'][i][0]-duty)/(1-duty)-.5))
 near=lambda s:min(frames,key=lambda f:abs(f['phase']-s))
 if kind==0:return [mid(LEGS.index(l)) for l in ('rh','rf','lh','lf')]
 b,a=mid(LEGS.index('rf')),mid(LEGS.index('lf'))  # RF+LH then LF+RH within one cycle
 after=(a['phase']+b['phase']+1)/2
 return [b,near((a['phase']+b['phase'])/2),a,near(after-1 if after>2 else after)]
def representative(entry):
 # Only an explicitly recorded, audited representative may be rendered; a failed candidate is refused.
 case=next(c for c in json.loads((OUT/entry['frames_source']).read_text())['cases'] if c['id']==entry['id'])
 assert case['complete'] and case['classification']==['KINEMATICALLY_VALID'] and case['first_failure'] is None and entry['status'] in ('FULL_MESH_VALIDATED','FULL_MESH_SAMPLED_PASS_DENSITY_FRAGILE'),entry
 return case
def main():
 reps=json.loads((OUT/'representatives.json').read_text())['representatives'];model=Model();(OUT/'views').mkdir(exist_ok=True)
 for entry in reps:
  kind=0 if entry['type']=='WALK' else 1;name=entry['name'];r=representative(entry);p=r['parameters']
  frames=r['frames'];chosen=snapshots(frames,p['duty'],kind)
  caption=f"{p['height_m']*1000:.0f} mm body · {p['advance_x_m']*1000:+.0f} mm/cycle · {p['lift_m']*1000:.0f} mm lift · duty {p['duty']:g} · id{entry['id']}\n{entry['status']}"
  for axes,label in (((0,2),'side'),((0,1),'top')):
   fig,axs=plt.subplots(2,2,figsize=(11,7),layout='constrained');fig.suptitle(f'MATDOG {name.upper()} — canonical collision meshes · {label} view\n{caption} · OFFLINE MODEL',fontsize=13)
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
  with (OUT/f'{name}_cycle.csv').open('w',newline='') as file:
   writer=csv.writer(file,lineterminator='\n');writer.writerow(['phase',*[f'{l}_{j}_{quantity}' for quantity in ('rad','rad_s','rad_s2') for l in LEGS for j in ('hip','upper','lower')],*[f'{l}_{axis}_world_m' for l in LEGS for axis in ('x','y','z')],'body_x_m','body_y_m','body_z_m','support_margin_m','classification'])
   for f in frames:writer.writerow([f['phase'],*f['q'],*f['qdot'],*f['qddot'],*np.array(f['contacts_world_m']).ravel(),*np.array(f['body'])[:3,3],f['support_margin_m'],';'.join(f['classification'])])
 print(f'Rendered {3*len(reps)} engineering views and {len(reps)} complete cycle CSV files')
if __name__=='__main__':main()
