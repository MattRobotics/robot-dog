"""Parse source-pinned XGO-family geometry; never relabel it exact Lite."""
import json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh
from model import Model,ROOT,sha,vec
from survey import save
CACHE=Path('/home/matteo-manicardi/robotics-reverse/work/milestone_h2_sources_20260728')
def main():
 m=Model();p=CACHE/'JoseManuelLuque_DOGZILLA/urdf/xgo_rviz.xacro';root=ET.parse(p).getroot();j={x.get('name'):x for x in root.findall('joint')};rows=[]
 def add(name,xgo,matdog,meaning='direct geometric distance',scope='GENERIC_TEMPLATE'):
  rows.append({'dimension':name,'xgo_m':xgo,'matdog_m':matdog,'ratio':matdog/xgo if xgo else None,'xgo_identity':scope,'meaning':meaning})
 hip=vec(j['lf_hip_joint'].find('origin'),'xyz');rear=vec(j['lh_hip_joint'].find('origin'),'xyz');rf=vec(j['rf_hip_joint'].find('origin'),'xyz')
 upper=vec(j['lf_upper_leg_joint'].find('origin'),'xyz');lower=vec(j['lf_lower_leg_link'].find('origin'),'xyz')
 add('front_rear_hip_spacing',float(hip[0]-rear[0]),.225)
 add('left_right_front_hip_spacing',float(hip[1]-rf[1]),.095)
 add('hip_upper_axis_lateral_offset',abs(float(upper[1])),.048)
 add('hip_upper_origin_distance',float(np.linalg.norm(upper)),.048,'origins use different frame conventions; vector components also reported')
 add('upper_link_joint_axis_distance',float(np.linalg.norm(lower)),.09)
 add('front_hip_frame_height',float(hip[2]),.0465,'frame-dependent, not scale-invariant')
 add('rear_hip_frame_height',float(rear[2]),.0265,'frame-dependent, not scale-invariant')
 lowerpath=CACHE/'JoseManuelLuque_DOGZILLA/meshes/XGO/lf_lower_leg_link.STL'
 mesh=trimesh.load_mesh(lowerpath)
 add('distal_mesh_extent_below_knee',float(-mesh.bounds[0,2]),.0499,'mesh extent versus URDF foot origin Z; unlike endpoints, not a usable similarity ratio')
 rows[-1]['ratio']=None
 for name in ['foot_cylinder_radius','foot_support_width','physical_joint_ranges','exact_lite_link_lengths']:
  rows.append({'dimension':name,'xgo_identity':'UNKNOWN','ratio':None,'reason':'No source-bound exact Lite geometry/limits recovered; family CAD and host command ranges cannot supply this'})
 save('dimensions',{'provenance':{'xgo_source':str(p),'xgo_commit':'674e3f2703b69ef182017eb72a4e3e2ee7dab72c','xgo_sha256':sha(p),'xgo_lower_mesh_sha256':sha(lowerpath),'matdog_sources':m.sources,'generator_sha256':sha(__file__)},'rows':rows,'source_correction':{'canonical_h2_inventory':'urdf_joint_inventory.csv','issue':'Summary CSV reports hip Z=0, upper origin=0 and lower Z=-0.049313; those disagree with the hash-matching pinned xacro and expanded URDF. This audit uses parsed primary XML, not that summary.','primary_values':{'lf_hip':hip.tolist(),'lf_upper_origin':upper.tolist(),'lf_lower_origin':lower.tolist()}},'conclusion':'1.5 is supported for template front/rear spacing and upper-link length; it is not a single scale across dimensions. Exact Lite similarity remains unproved.','exact_lite_host_ranges':{'translation_z':[60,110],'unit':'documented host millimetres, not proven ground/body frame datum','source':'LuwuDynamics/xgo_doglib cf72514273dc703284d3c47e46c67ce238caae11 xgolib/xgolib_dog.py changePara xgolite','classification':'EXACT_LITE_HOST_API_ONLY'}})
 print([(r['dimension'],r.get('ratio')) for r in rows])
if __name__=='__main__':main()
