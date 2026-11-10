"""Assemble parts+servos into kinematic links, FCL collision checks."""
import os, numpy as np, trimesh, cadquery as cq
from robot import *

HERE_ = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE_, '.cache')
os.makedirs(BUILD, exist_ok=True)
STEP_SERVO = os.environ.get('STS3215_STEP', os.path.join(HERE_, 'vendor', 'STS3215_03a.step'))

def tess_to_mesh(shape, tol=0.1, ang=0.3):
    v, f = shape.tessellate(tol, ang)
    m = trimesh.Trimesh(np.array([[p.x, p.y, p.z] for p in v]), np.array(f), process=False)
    m.merge_vertices()
    m.update_faces(m.nondegenerate_faces())      # zero-area slivers from fillet/union faces would leave open edges
    m.remove_unreferenced_vertices()
    return m

_servo_mesh = None
def servo_mesh(tol=0.15):
    global _servo_mesh
    if _servo_mesh is None:
        s = cq.importers.importStep(STEP_SERVO).val()
        _servo_mesh = tess_to_mesh(s, tol, 0.4)
    return _servo_mesh

def build_parts(styling=False):
    P, SV, J = build_all()
    solids, meshes = {}, {}
    for k, p in P.items():
        s = p.build()
        solids[k] = s
        meshes[k] = tess_to_mesh(s)
    if styling:
        from shells import add_styling
        add_styling(P, solids, meshes)
    return P, SV, J, solids, meshes

def link_table(P, J):
    """link name -> dict(parts=[...], servos=[Servo...]); base link = pelvis+torso"""
    host = lambda k: COVER_HOST.get(k) or getattr(P[k], 'host', None) or ('base' if k in ('pelvis', 'torso') else k)
    links = {'base': dict(parts=[], servos=[])}
    for k, p in P.items():
        lk = host(k)
        links.setdefault(lk, dict(parts=[], servos=[]))
        links[lk]['parts'].append(k)
        links[lk]['servos'] += p.servos
    return links

def tree(J):
    kids = {}
    for j in J:
        kids.setdefault(j['parent'], []).append(j)
    return kids

def rot_about(axis, q, point):
    a = np.array(axis, float); a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = point - R @ point
    return T

def link_transforms(J, q):
    kids = tree(J)
    T = {'base': np.eye(4)}
    def rec(l):
        for j in kids.get(l, []):
            T[j['child']] = T[l] @ rot_about(j['servo'].a, q.get(j['name'], 0.0), j['servo'].S)
            rec(j['child'])
    rec('base')
    return T

def collision_report(P, J, meshes, links, q=None, margin=0.0, skip_same_link=True):
    import fcl
    q = q or {}
    T = link_transforms(J, q)
    sm = servo_mesh()
    objs = []   # (name, link, kind, fclobj)
    def mk(mesh, M):
        m = mesh.copy(); m.apply_transform(M)
        bvh = fcl.BVHModel(); bvh.beginModel(len(m.vertices), len(m.faces))
        bvh.addSubModel(m.vertices, m.faces); bvh.endModel()
        return fcl.CollisionObject(bvh)
    for lk, d in links.items():
        for pn in d['parts']:
            objs.append((pn, lk, 'part', mk(meshes[pn], T[lk])))
        for sv in d['servos']:
            objs.append((f"servo:{sv.name}", lk, 'servo', mk(sm, T[lk] @ sv.step_to_world())))
    hits = []
    for i in range(len(objs)):
        for j in range(i + 1, len(objs)):
            a, b = objs[i], objs[j]
            if skip_same_link and a[1] == b[1] and a[2] == b[2]:
                continue
            req = fcl.CollisionRequest(num_max_contacts=1, enable_contact=True)
            res = fcl.CollisionResult()
            n = fcl.collide(a[3], b[3], req, res)
            if n:
                depth = max((c.penetration_depth for c in res.contacts), default=0)
                hits.append((a[0], b[0], round(depth, 2)))
    return hits

if __name__ == '__main__':
    P, SV, J, solids, meshes = build_parts()
    links = link_table(P, J)
    h = collision_report(P, J, meshes, links)
    print('zero-pose collisions:', len(h))
    for x in h: print(x)
