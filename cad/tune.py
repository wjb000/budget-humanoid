import json, time, os
from collider import *
P,SV,J,solids,meshes=build_parts(); links=link_table(P,J)
C=Collider(P,J,meshes,links)
t=time.time(); print('zero',C.hits({}), time.time()-t)
out={}
for j in J:
    t=time.time()
    lo,hi=C.safe_range(j['name'],j.get('lim_nom',j['lim']))
    out[j['name']]=(lo,hi)
    print(f"{j['name']:16s} nominal {j['lim']} safe ({lo:+.2f},{hi:+.2f})  {time.time()-t:.1f}s",flush=True)
import os
lim={k:(round(int(v[0]*0.95*100)/100,2),round(int(v[1]*0.95*100)/100,2)) for k,v in out.items()}
json.dump(out,open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'.cache','safe_limits.json'),'w'),indent=1)
json.dump(lim,open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'limits.json'),'w'),indent=1)
print('wrote limits.json')
