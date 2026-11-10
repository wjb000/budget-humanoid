"""Build everything: STEP + STL per part, print analysis, mass properties, URDF, MJCF, glTF. Run: python export.py"""
import os, json, math, sys, numpy as np, trimesh, cadquery as cq
from assembly import *
from electronics import electronics

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PLA_DENSITY = 1.24e-6      # kg/mm^3
FILL = 0.60                # effective solid fraction (2.4 mm walls + ~20% gyroid infill)
SERVO_MASS = 0.055 + 0.004 # STS3215 55 g + 4 g screws/horn hardware (estimate)
BED = (220.0, 220.0)
DEC = os.path.join(HERE, '.cache')   # built by vendor_meshes.py

def load_dec():
    out = {}
    for n in ('pi5', 'bno085', 'max98357', 'speaker40', 'sts3215'):
        f = f'{DEC}/{n}_dec.npz'
        if os.path.exists(f):
            z = np.load(f); out[n] = (z['v'], z['f'])
    return out

def mkdir(*p):
    d = os.path.join(ROOT, *p); os.makedirs(d, exist_ok=True); return d

# ----------------------------------------------------------- print analysis
UPS = {'+z': np.eye(3), '-z': np.diag([1, -1, -1.]), '+x': np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0.]]),
       '-x': np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0.]]), '+y': np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.]]),
       '-y': np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0.]])}

def orient(mesh):
    best = None
    rows = []
    for k, R in UPS.items():
        m = mesh.copy(); M = np.eye(4); M[:3, :3] = R; m.apply_transform(M)
        m.apply_translation(-m.bounds[0] * np.array([0, 0, 1]) - np.array([0, 0, 0]))
        zmin = m.bounds[0][2]
        m.apply_translation((0, 0, -zmin))
        n = m.face_normals; a = m.area_faces
        cz = m.triangles_center[:, 2]
        over = a[(n[:, 2] < -0.7071) & (cz > 0.05)].sum()
        bed = a[(n[:, 2] < -0.99) & (cz < 0.05)].sum()
        ext = m.extents
        fits = ext[0] <= BED[0] and ext[1] <= BED[1] and ext[2] <= 250
        score = over + 0.05 * ext[2] - 0.01 * bed + (1e6 if not fits else 0)
        rows.append((score, k, over, bed, ext, m, R))
    rows.sort(key=lambda r: r[0])
    s, k, over, bed, ext, m, R = rows[0]
    tot = mesh.area
    return dict(up=k, overhang_mm2=round(float(over), 1), overhang_pct=round(float(over / tot * 100), 1), bed_contact_mm2=round(float(bed), 1),
                extents=[round(float(e), 1) for e in ext], mesh=m, R=R)

def min_thickness(mesh, n=2500):
    pts, fi = trimesh.sample.sample_surface(mesh, n * 3, seed=1)
    nrm = mesh.face_normals[fi]
    flat = (np.abs(nrm).max(1) > 0.999)          # flat faces only: curved hole walls give false thin ligaments
    pts, nrm = pts[flat][:n], nrm[flat][:n]
    n = len(pts)
    org = pts - nrm * 0.02
    dirs = -nrm
    loc, ir, _ = mesh.ray.intersects_location(org, dirs, multiple_hits=False)
    d = np.full(n, np.nan); d[ir] = np.linalg.norm(loc - org[ir], axis=1)
    d = d[~np.isnan(d)]
    return dict(p1=round(float(np.percentile(d, 1)), 2), p5=round(float(np.percentile(d, 5)), 2), min=round(float(d.min()), 2))

def fast_ok(v):
    return v

def write_stl_compact(mesh, path):
    """ASCII STL with 3-decimal coordinates (1 micron): about 3x smaller than trimesh's full-precision text, same geometry."""
    mesh = trimesh.Trimesh(np.round(mesh.vertices, 3), mesh.faces, process=False); mesh.merge_vertices()
    mesh.update_faces(mesh.nondegenerate_faces()); mesh.remove_unreferenced_vertices()
    v = mesh.vertices; n = mesh.face_normals
    out = ['solid part']
    for f, nn in zip(mesh.faces, n):
        out.append('facet normal %.3f %.3f %.3f\nouter loop' % tuple(nn))
        for i in f: out.append('vertex %.3f %.3f %.3f' % tuple(v[i]))
        out.append('endloop\nendfacet')
    out.append('endsolid part')
    open(path, 'w').write('\n'.join(out) + '\n')

def export_parts(P, solids, meshes, step_dir, stl_dir):
    report = {}
    for k, p in P.items():
        sh = solids[k]
        cq.exporters.export(cq.Workplane().add(sh), os.path.join(step_dir, f'{k}.step'))
        fine = tess_to_mesh(sh, 0.08, 0.4)
        o = orient(fine)
        pm = o.pop('mesh')
        write_stl_compact(pm, os.path.join(stl_dir, f'{k}.stl'))
        thick = min_thickness(fine)
        vol = float(sh.Volume())
        report[k] = dict(up=o['up'], overhang_pct=o['overhang_pct'], overhang_mm2=o['overhang_mm2'], bed_contact_mm2=o['bed_contact_mm2'],
                         print_extents_mm=o['extents'], volume_cm3=round(vol / 1000, 1), wall_min_est=thick, faces=int(len(pm.faces)),
                         mass_g_est=round(vol * PLA_DENSITY * FILL * 1000, 1), servos=[s.name for s in p.servos],
                         stl_bytes=os.path.getsize(os.path.join(stl_dir, f'{k}.stl')), watertight=bool(fine.is_watertight))
        print(f"{k:16s} up={o['up']} over={o['overhang_pct']:5.1f}% ext={o['extents']} wall p1={thick['p1']} mass={report[k]['mass_g_est']}g stl={report[k]['stl_bytes']//1024}KB wt={report[k]['watertight']}", flush=True)
    return report

# ----------------------------------------------------------- mass properties
def comb(items):
    """items: list of (trimesh (m, link-local), mass kg) -> mass, com, inertia about com (3x3 kg m^2)"""
    M = 0; C = np.zeros(3); parts = []
    for m, mass in items:
        mm = m.copy(); mm.density = mass / abs(mm.volume)
        com = np.array(mm.center_mass); I = np.array(mm.moment_inertia)
        parts.append((mass, com, I)); M += mass; C += mass * com
    C /= M
    I = np.zeros((3, 3))
    for mass, com, Ii in parts:
        d = com - C
        I += Ii + mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return M, C, I

def quat_from_R(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2; w = s / 4; x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2; w = (R[2, 1] - R[1, 2]) / s; x = s / 4; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2; w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; y = s / 4; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2; w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s; z = s / 4
    return np.array([w, x, y, z])

def rpy_from_R(R):
    sy = -R[2, 0]
    pitch = math.asin(max(-1, min(1, sy)))
    if abs(sy) < 0.999999:
        roll = math.atan2(R[2, 1], R[2, 2]); yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1]); yaw = 0.0
    return roll, pitch, yaw

def write_obj(path, m):
    with open(path, 'w') as f:
        for v in m.vertices: f.write('v %.5f %.5f %.5f\n' % tuple(v))
        for t in m.faces: f.write('f %d %d %d\n' % (t[0] + 1, t[1] + 1, t[2] + 1))

def main():
    dec = load_dec()
    P, SV, J, solids, meshes = build_parts()
    links = link_table(P, J)
    E = electronics(dec)
    step_dir = mkdir('cad', 'step'); stl_dir = mkdir('cad', 'stl')
    rep = export_parts(P, solids, meshes, step_dir, stl_dir)
    json.dump(rep, open(os.path.join(ROOT, 'cad', 'print_report.json'), 'w'), indent=1)
    json.dump(dict(joints=[dict(name=j['name'], id=j['id'], parent=j['parent'], child=j['child'], axis=[float(x) for x in j['servo'].a], pos_mm=[float(x) for x in j['servo'].S],
                                lim=list(j['lim']), lim_nom=list(j.get('lim_nom', j['lim']))) for j in J]),
              open(os.path.join(ROOT, 'cad', 'joints.json'), 'w'), indent=1)
    # electronics overlap check vs torso/head + each other (fcl)
    import fcl
    def bvh(m):
        b = fcl.BVHModel(); b.beginModel(len(m.vertices), len(m.faces)); b.addSubModel(m.vertices, m.faces); b.endModel(); return fcl.CollisionObject(b)
    obj = [(e['name'], e['link'], bvh(e['mesh'])) for e in E]
    base_objs = [(k, bvh(meshes[k])) for k in ('torso', 'head', 'pelvis', 'torso_front_cover', 'battery_door', 'head_lid')]
    bad = []
    for i in range(len(obj)):
        for nm, o in base_objs:
            if (nm == 'head') != (obj[i][1] == 'head'): continue
            r = fcl.CollisionResult()
            if fcl.collide(obj[i][2], o, fcl.CollisionRequest(num_max_contacts=1, enable_contact=True), r):
                bad.append((obj[i][0], nm, round(max(c.penetration_depth for c in r.contacts), 2)))
        for j in range(i + 1, len(obj)):
            if obj[i][1] != obj[j][1]: continue
            r = fcl.CollisionResult()
            if fcl.collide(obj[i][2], obj[j][2], fcl.CollisionRequest(num_max_contacts=1, enable_contact=True), r):
                bad.append((obj[i][0], obj[j][0], round(max(c.penetration_depth for c in r.contacts), 2)))
    print('electronics overlaps:', bad)
    json.dump(bad, open(os.path.join(ROOT, 'cad', 'electronics_overlaps.json'), 'w'))
    # ---------------- link frames, mass properties
    origin = {'base': np.zeros(3)}
    for j in J: origin[j['child']] = j['servo'].S.copy()
    sm = servo_mesh() if 'sts3215' not in dec else trimesh.Trimesh(*dec['sts3215'], process=False)
    smass = {}
    props = {}
    for lk, d in links.items():
        items = []
        o = origin[lk]
        for pn in d['parts']:
            m = meshes[pn].copy(); m.apply_translation(-o); m.apply_scale(1e-3)
            vol = abs(m.volume) * 1e9  # mm3
            items.append((m, vol * PLA_DENSITY * FILL))
        for sv in d['servos']:
            m = sm.copy(); M = sv.step_to_world(); m.apply_transform(M); m.apply_translation(-o); m.apply_scale(1e-3)
            items.append((m, SERVO_MASS))
        for e in E:
            if e['link'] == lk or (e['link'] == 'head' and lk == 'head'):
                m = e['mesh'].copy(); m.apply_translation(-o); m.apply_scale(1e-3)
                if not m.is_watertight: m = m.convex_hull
                items.append((m, e['mass'] / 1000))
        if lk == 'base':
            b = trimesh.creation.box(extents=(0.07, 0.07, 0.07)); b.apply_translation((-0.025 - 0.0, 0, 0.40)); items.append((b, 0.090))  # wiring + fasteners estimate
        Mx, C, I = comb(items)
        props[lk] = dict(mass=float(Mx), com=C.tolist(), inertia=I.tolist(), origin_mm=o.tolist())
    tot = sum(p['mass'] for p in props.values())
    allc = sum(np.array(p['com']) * p['mass'] + np.array(p['origin_mm']) * 1e-3 * p['mass'] for p in props.values()) / tot
    print('total mass %.3f kg  COM (m) %s' % (tot, np.round(allc, 4)))
    json.dump(dict(total_mass_kg=tot, com_m=allc.tolist(), links=props, assumptions=dict(pla_density_g_cc=1.24, fill_factor=FILL, servo_mass_g=SERVO_MASS * 1000)),
              open(os.path.join(ROOT, 'sim', 'mass_properties.json'), 'w') if os.path.isdir(os.path.join(ROOT, 'sim')) else open(os.path.join(mkdir('sim'), 'mass_properties.json'), 'w'), indent=1)
    return dict(P=P, SV=SV, J=J, meshes=meshes, links=links, E=E, props=props, origin=origin, sm=sm, rep=rep, dec=dec)

if __name__ == '__main__':
    main()
