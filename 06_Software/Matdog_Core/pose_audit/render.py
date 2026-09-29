"""Deterministic orthographic engineering views from canonical collision triangles."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import numpy as np
from model import Model,sha
from survey import OUT,save

def main():
 m=Model();lib=json.loads((OUT/'pose_library.json').read_text());dest=OUT/'views';dest.mkdir(exist_ok=True);outputs={}
 axes=[(np.array([1,0,0]),np.array([0,0,1]),'SIDE X/Z'),(np.array([0,1,0]),np.array([0,0,1]),'FRONT Y/Z'),(np.array([.7071067811865475,-.7071067811865475,0]),np.array([.408248290463863,.408248290463863,.816496580927726]),'ISOMETRIC')]
 for pose in lib['poses']:
  tf=m.fk(pose['q'],np.array(pose['body']));fig,axs=plt.subplots(1,3,figsize=(15,4),dpi=130)
  for ax,(right,up,label) in zip(axs,axes):
   depth=np.cross(right,up);faces=[];colors=[];zs=[]
   for n,mesh in m.meshes.items():
    t=tf[n];v=mesh.vertices@t[:3,:3].T+t[:3,3];tri=v[mesh.faces];xy=np.stack([tri@right,tri@up],axis=-1);faces.extend(xy);zs.extend(tri.mean(axis=1)@depth)
    color=('#9da9b8' if n=='base_link' else '#e5a344' if 'foot' in n else '#4298b5' if 'upper' in n else '#627589');colors.extend([color]*len(tri))
   order=np.argsort(zs,kind='stable');ax.add_collection(PolyCollection(np.asarray(faces)[order],facecolors=np.array(colors)[order],edgecolors='none',rasterized=True))
   poly=np.c_[pose['support_polygon'],np.zeros(len(pose['support_polygon']))];poly=np.vstack([poly,poly[0]]);ax.plot(poly@right,poly@up,color='#228833',lw=2,label='Support hull')
   com=np.array(pose['com_world_m']);ax.plot(com@right,com@up,'o',color='#aa3377',ms=5,label='CAD COM');proj=com.copy();proj[2]=0;ax.plot([com@right,proj@right],[com@up,proj@up],':',color='#aa3377')
   if label!='ISOMETRIC':ax.axhline(0,color='#bbbbbb',lw=.5)
   projected=np.asarray(faces).reshape(-1,2);lo=projected.min(axis=0)-.025;hi=projected.max(axis=0)+.025
   ax.set_aspect('equal');ax.set_xlim(lo[0],hi[0]);ax.set_ylim(min(lo[1],-.025),hi[1]);ax.set_title(label);ax.set_xlabel('projected metres');ax.grid(alpha=.15)
  axs[0].legend(loc='upper left',fontsize=7);fig.suptitle(pose['name']+' | '+pose['evidence']+' | '+pose['regime']+'\nbase Z %.3f mm | CAD support margin %.2f mm | rigid mesh evidence only'%(pose['body'][2][3]*1000,pose['support_margin_m']*1000),fontsize=10)
  fig.tight_layout(rect=[0,0,1,.9]);p=dest/(pose['name']+'.png');fig.savefig(p,bbox_inches='tight',metadata={'Software':'MATDOG G3.5 canonical mesh renderer'});plt.close(fig);outputs[str(p.relative_to(OUT))]=sha(p)
 env=json.loads((OUT/'envelope.json').read_text());fig,axs=plt.subplots(2,3,figsize=(12,6),dpi=130)
 for ax,dim in zip(axs.flat,['height','x','y','yaw','roll','pitch']):
  rows=[r for r in env['rows'] if r['dimension']==dim];x=[r['value']*(180/np.pi if dim in ('yaw','roll','pitch') else 1000) for r in rows];y=[int(r['valid']) for r in rows];ax.scatter(x,y,c=['#228833' if v else '#cc6677' for v in y],s=12);ax.set_title(dim+(' (deg)' if dim in ('yaw','roll','pitch') else ' (mm)'));ax.set_yticks([0,1],['rejected','valid']);ax.grid(alpha=.2)
 fig.suptitle('C4 footprint: independent one-axis slices at STAND; sampled feasibility, not global bounds');fig.tight_layout();p=dest/'envelope.png';fig.savefig(p,metadata={'Software':'MATDOG G3.5 canonical survey'});plt.close(fig);outputs[str(p.relative_to(OUT))]=sha(p)
 save('visual_manifest',{'provenance':{'canonical_sources':m.sources,'library_sha256':sha(OUT/'pose_library.json'),'generator_sha256':sha(__file__)},'outputs':outputs,'scope':'actual triangle geometry; support hull and CAD COM overlay; failed SIT/Beg targets are not rendered as achieved poses'})
 print('RENDERED',len(outputs),flush=True)
if __name__=='__main__':main()
