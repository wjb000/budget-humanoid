import sys
from assembly import *
def sweep(P,J,meshes,links,test=None,fr=(0.5,1.0)):
    out={}
    for j in (test or J):
        lo,hi=j['lim']
        for sign,lim in (('lo',lo),('hi',hi)):
            for f in fr:
                q={j['name']:lim*f}
                h=collision_report(P,J,meshes,links,q)
                if h: out.setdefault(j['name'],[]).append((sign,f,h))
    return out
if __name__=='__main__':
    P,SV,J,solids,meshes=build_parts(); links=link_table(P,J)
    print('zero:',collision_report(P,J,meshes,links))
    only=[j for j in J if j['name'].startswith('r_') or j['name'].startswith('neck')]
    r=sweep(P,J,meshes,links,only)
    for k,v in r.items():
        for sign,f,h in v: print(k,sign,f,h[:4],flush=True)
    print('done; joints with hits:',list(r))
