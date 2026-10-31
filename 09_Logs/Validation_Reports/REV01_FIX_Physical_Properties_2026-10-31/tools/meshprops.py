"""Uniform-density mass properties of a closed triangle mesh (mm) -> volume, centroid, inertia about centroid per unit volume."""
import numpy as np, struct, sys
def load_stl(path):
    b=open(path,"rb").read()
    n=struct.unpack_from("<I",b,80)[0]
    if 84+50*n==len(b):
        dt=np.dtype([("n","<f4",3),("v","<f4",(3,3)),("a","<u2")])
        a=np.frombuffer(b,dtype=dt,count=n,offset=84)
        return a["v"].astype(np.float64)
    # ascii
    import re
    v=np.array([[float(x) for x in m.groups()] for m in re.finditer(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)",b)])
    return v.reshape(-1,3,3)
def props(tri):
    a,b,c=tri[:,0],tri[:,1],tri[:,2]
    cr=np.cross(b,c); det=np.einsum("ij,ij->i",a,cr)        # 6*signed tet volume
    vol=det.sum()/6.0
    cen=((a+b+c)*det[:,None]).sum(0)/(24.0*vol)               # sum (V_i * centroid_i)/V ; centroid_i=(a+b+c)/4 ; V_i=det/6
    # second moments about origin via tetra integral formula
    def mom(i,j):
        s=0
        for (p,q) in ((a,a),(b,b),(c,c)):
            pass
        # integral over tet(0,a,b,c) of x_i x_j = det/120 * (sum_k a_k_i*a_k_j + (sum a_i)(sum a_j))
        sij=a[:,i]*a[:,j]+b[:,i]*b[:,j]+c[:,i]*c[:,j]+(a[:,i]+b[:,i]+c[:,i])*(a[:,j]+b[:,j]+c[:,j])
        return (det*sij).sum()/120.0
    M=np.array([[mom(i,j) for j in range(3)] for i in range(3)])   # int x_i x_j dV about origin
    Mc=M-vol*np.outer(cen,cen)
    I=np.trace(Mc)*np.eye(3)-Mc                                  # inertia about centroid per unit density, mm^5
    return vol,cen,I
if __name__=="__main__":
    for p in sys.argv[1:]:
        v,c,I=props(load_stl(p)); print(p.split("/")[-1],"vol=%.3f mm3"%v,"cen=",np.round(c,3))
