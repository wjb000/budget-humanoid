import numpy as np, fcl
from assembly import *

class Collider:
    def __init__(self, P, J, meshes, links, servo_tol=0.15):
        self.J, self.links = J, links
        sm = servo_mesh()
        self.objs = []
        def mk(mesh):
            bvh = fcl.BVHModel(); bvh.beginModel(len(mesh.vertices), len(mesh.faces))
            bvh.addSubModel(mesh.vertices, mesh.faces); bvh.endModel()
            return fcl.CollisionObject(bvh)
        for lk, d in links.items():
            for pn in d['parts']:
                self.objs.append(dict(name=pn, link=lk, kind='part', obj=mk(meshes[pn]), M=np.eye(4)))
            for sv in d['servos']:
                self.objs.append(dict(name='servo:' + sv.name, link=lk, kind='servo', obj=mk(sm), M=sv.step_to_world()))
        self.pairs = [(i, j) for i in range(len(self.objs)) for j in range(i + 1, len(self.objs))
                      if not (self.objs[i]['link'] == self.objs[j]['link'])]
    def hits(self, q, ignore_neighbors=False):
        T = link_transforms(self.J, q)
        for o in self.objs:
            M = T[o['link']] @ o['M']
            o['obj'].setTransform(fcl.Transform(M[:3, :3], M[:3, 3]))
        out = []
        for i, j in self.pairs:
            a, b = self.objs[i], self.objs[j]
            res = fcl.CollisionResult()
            if fcl.collide(a['obj'], b['obj'], fcl.CollisionRequest(num_max_contacts=1, enable_contact=True), res):
                out.append((a['name'], b['name'], round(max((c.penetration_depth for c in res.contacts), default=0), 2)))
        return out
    def safe_range(self, jname, lim, step=0.02):
        """largest [lo,hi] within lim with zero collisions (others at 0); scan outward from 0."""
        res = []
        for sgn, bound in ((-1, lim[0]), (1, lim[1])):
            q = 0.0
            lastok = 0.0
            while abs(q) < abs(bound) - 1e-9:
                q += sgn * step
                if abs(q) > abs(bound): q = bound
                if self.hits({jname: q}):
                    break
                lastok = q
            res.append(lastok)
        return tuple(res)
