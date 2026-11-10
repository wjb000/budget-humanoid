"""Budget-22 humanoid layout: servo poses (zero pose), printed parts, kinematic tree. Units mm.
World: x forward, y left, z up, ground z=0. Right-side numbers are authored with y<0; left side mirrors y."""
import numpy as np
from lib import *

XT = -22.4                  # torso centre x
Z_BASE = 358.5              # top of pelvis yaw sleeves / torso floor
TORSO_X = (-62.4, 17.6)
TORSO_Y = (-52.0, 52.0)
TORSO_TOP = 500.0
XN = -40.0                  # neck axis x

class Side:
    def __init__(self, sg):
        self.sg = sg                      # -1 right, +1 left
        self.sfx = 'r' if sg < 0 else 'l'
    def y(self, y):
        return -y if self.sg > 0 else y
    def v(self, v):
        return (v[0], -v[1], v[2]) if self.sg > 0 else tuple(v)
    def b(self, x0, x1, y0, y1, z0, z1):
        a, c = self.y(y0), self.y(y1)
        return (x0, x1, min(a, c), max(a, c), z0, z1)
    def servo(self, sid, name, S, a, t):
        return Servo(sid, name, (S[0], self.y(S[1]), S[2]), self.v(a), self.v(t))

def build_side(sd, ids):
    """returns parts (dict), servos (dict name->(Servo, carrier part)), joints (list)"""
    P, SV, J = {}, {}, []
    n = lambda s: f"{sd.sfx}_{s}"
    # ---------------- LEG ----------------
    Zar, Zap, Zk, Zh = 25.0, 78.0, 175.0, 275.0
    Yf = -55.0                          # foot / ankle-roll / hip-roll plane (right side)
    YA = Yf - 15.2                      # ankle pitch horn plane (roll sleeve lives inboard of it)
    YH = YA + 4.2                       # hip pitch horn plane (thigh plate coplanar with ankle bracket plate)
    plate_in = YH - GAP                 # inboard face of thigh plate
    plate_out = plate_in - PT
    YK = plate_in - 41.5                # knee horn plane (knee body outboard of thigh plate)
    shin_in = YK - GAP
    shin_out = shin_in - PT
    assert abs((YA - 41.5) - shin_out) < 1e-6, (YA - 41.5, shin_out)
    Yc = Yf
    XR_h = -28.2
    XAR = 22.0
    Zy = 317.0
    XY_ = -40.0
    s_ar = sd.servo(ids['ankle_roll'], n('ankle_roll'), (XAR, Yf, Zar), (1, 0, 0), (0, 0, 1))
    s_ap = sd.servo(ids['ankle_pitch'], n('ankle_pitch'), (0, YA, Zap), (0, 1, 0), (0, 0, 1))
    s_k = sd.servo(ids['knee'], n('knee'), (0, YK, Zk), (0, -1, 0), (0, 0, 1))
    s_hp = sd.servo(ids['hip_pitch'], n('hip_pitch'), (0, YH, Zh), (0, -1, 0), (0, 0, -1))
    s_hr = sd.servo(ids['hip_roll'], n('hip_roll'), (XR_h, Yf, Zh), (1, 0, 0), (0, 0, 1))
    s_hy = sd.servo(ids['hip_yaw'], n('hip_yaw'), (XY_, Yf, Zy), (0, 0, -1), (0, 1, 0))

    # foot (child of ankle roll)
    foot = Part(n('foot'))
    foot.plate(s_ar, -15, 15, -15, 15)
    foot.add(sd.b(XAR + GAP, XAR + GAP + PT, Yc - 15, Yc + 15, 0, 42))          # upright plate
    foot.add(sd.b(-50, 72, Yc - 28, Yc + 28, 0, 4))                              # sole
    for yy in (-14, 14):                                                         # gussets (3 mm) from sole to plate
        foot.add(sd.b(XAR + GAP, XAR + 28, Yc + yy - 1.5, Yc + yy + 1.5, 0, 4 + 0.0))
        foot.add(sd.b(XAR + GAP, XAR + GAP + PT + 22, Yc + yy - 1.5, Yc + yy + 1.5, 0, 4))
    foot.add(sd.b(XAR + GAP, XAR + GAP + PT + 20, Yc - 15, Yc - 12, 0, 30))
    foot.add(sd.b(XAR + GAP, XAR + GAP + PT + 20, Yc + 12, Yc + 15, 0, 30))
    # ankle link: ankle pitch hub + roll sleeve
    al = Part(n('ankle_link'))
    al.sleeve(s_ar)
    al.plate(s_ap, -15, 15, -15, 15)
    al.add(sd.b(-15, 15, YA + GAP, YA + GAP + PT, Zar + 37.0, Zap + 15))        # hub to sleeve top (overlap)
    # shin: knee hub + strip + ankle-pitch sleeve
    sh = Part(n('shin'))
    sh.plate(s_k, -15, 15, -15, 15)
    sh.sleeve(s_ap)
    sh.add(sd.b(-15, 15, shin_out, shin_in, Zap + 35, Zk + 15))
    # thigh: hip pitch hub + strip + knee sleeve
    th = Part(n('thigh'))
    th.plate(s_hp, -15, 15, -15, 15)
    th.sleeve(s_k)
    th.add(sd.b(-15, 15, plate_out, plate_in, Zk + 35, Zh + 15))
    # hip roll link: roll hub plate + bridge + pitch sleeve
    hr = Part(n('hip_roll_link'))
    hr.plate(s_hr, -15, 15, -9.5, 9.5)
    hr.sleeve(s_hp)
    xp0 = XR_h + GAP
    hr.add(sd.b(xp0, xp0 + PT, Yc - 9.5, Yc + 9.5, Zh - 15, Zh + 15))
    hr.add(sd.b(xp0, -12.7, YH + 3 - 0.0, YH + 3 + 10.0, Zh - 15, Zh + 13))     # bridge to pitch sleeve side wall
    # hip yaw link: yaw hub + roll sleeve
    hy = Part(n('hip_yaw_link'))
    hy.sleeve(s_hr)
    hy.plate(s_hy, -15, 15, -15, 15)
    hy.add(sd.b(XY_ - 15, XY_ + 15, Yc - 15, Yc + 15, Zy - PT - GAP - 0.5, Zy - GAP - 0.0))
    P.update({p.name: p for p in (foot, al, sh, th, hr, hy)})
    carrier = {s_ar: al, s_ap: sh, s_k: th, s_hp: hr, s_hr: hy}
    for s in (s_ar, s_ap, s_k, s_hp, s_hr, s_hy):
        SV[s.name] = s
    # joints: name, id, servo, parent link (carries body), child link (has plate), limits
    J += [
        dict(name=n('hip_yaw'), id=ids['hip_yaw'], servo=s_hy, parent='base', child=hy.name, lim=(-0.6, 0.6)),
        dict(name=n('hip_roll'), id=ids['hip_roll'], servo=s_hr, parent=hy.name, child=hr.name, lim=(-0.5, 0.5)),
        dict(name=n('hip_pitch'), id=ids['hip_pitch'], servo=s_hp, parent=hr.name, child=th.name, lim=(-1.0, 1.0)),
        dict(name=n('knee'), id=ids['knee'], servo=s_k, parent=th.name, child=sh.name, lim=(-0.1, 2.0)),
        dict(name=n('ankle_pitch'), id=ids['ankle_pitch'], servo=s_ap, parent=sh.name, child=al.name, lim=(-0.8, 0.8)),
        dict(name=n('ankle_roll'), id=ids['ankle_roll'], servo=s_ar, parent=al.name, child=foot.name, lim=(-0.35, 0.35)),
    ]
    # ---------------- ARM ----------------
    ys = -93.5
    Zs = 440.0
    s_sp = sd.servo(ids['sh_pitch'], n('shoulder_pitch'), (XT, ys, Zs), (0, -1, 0), (0, 0, 1))
    Xr = XT + 22.25
    Yr = ys - 15.2
    Zr = 412.0
    s_sr = sd.servo(ids['sh_roll'], n('shoulder_roll'), (Xr, Yr, Zr), (1, 0, 0), (0, 0, -1))
    Xe = Xr + 16.65
    Ye = -90.7
    Ze = 359.0
    s_el = sd.servo(ids['elbow'], n('elbow'), (Xe, Ye, Ze), (0, 1, 0), (0, 0, 1))
    Zg = 305.0
    Yg = (Ye + GAP + PT) - 41.5 - 0.0     # gripper rear wall inner end at forearm plate
    s_gr = sd.servo(ids['gripper'], n('gripper'), (Xe, Yg, Zg), (0, -1, 0), (0, 0, -1))
    sl = Part(n('shoulder_link'))
    sl.plate(s_sp, -15, 15, -15, 15)
    sl.sleeve(s_sr)
    sl.add(sd.b(Xr - 15, Xr - 3.6, ys - GAP - PT, ys - GAP, Zr + 12, Zs + 15))
    ua = Part(n('upper_arm'))
    ua.plate(s_sr, -11, 11, -15, 15)
    ua.sleeve(s_el)
    ua.add(sd.b(Xr + GAP, Xr + GAP + PT, Ye - 41.5, Ye - 3, Ze - 12.9, Zr + 11))
    fa = Part(n('forearm'))
    fa.plate(s_el, -12, 12, -12, 12)
    fa.sleeve(s_gr)
    fa.add(sd.b(Xe - 12, Xe + 12, Ye + GAP, Ye + GAP + PT, Zg - 37.9, Ze + 12))
    fa.add(sd.b(Xe + 14, Xe + 22, Yg + 3, Yg + 41.5 + 0.5, Zg - 75, Zg - 30))      # fixed jaw
    fi = Part(n('finger'))
    fi.plate(s_gr, -12, 12, -12, 12)
    fi.add(sd.b(Xe - 12, Xe + 4, Yg - GAP - PT, Yg - GAP, Zg - 75, Zg + 12))
    fi.add(sd.b(Xe - 6, Xe + 4, Yg - GAP - PT, Yg + 25, Zg - 75, Zg - 55))
    P.update({p.name: p for p in (sl, ua, fa, fi)})
    for s in (s_sp, s_sr, s_el, s_gr):
        SV[s.name] = s
    J += [
        dict(name=n('shoulder_pitch'), id=ids['sh_pitch'], servo=s_sp, parent='base', child=sl.name, lim=(-2.0, 2.0)),
        dict(name=n('shoulder_roll'), id=ids['sh_roll'], servo=s_sr, parent=sl.name, child=ua.name, lim=(-1.2, 1.2)),
        dict(name=n('elbow'), id=ids['elbow'], servo=s_el, parent=ua.name, child=fa.name, lim=(-1.5, 1.5)),
        dict(name=n('gripper'), id=ids['gripper'], servo=s_gr, parent=fa.name, child=fi.name, lim=(-0.5, 0.2)),
    ]
    return P, SV, J

def build_all():
    R_ids = dict(hip_yaw=1, hip_roll=2, hip_pitch=3, knee=4, ankle_pitch=5, ankle_roll=6, sh_pitch=13, sh_roll=14, elbow=15, gripper=16)
    L_ids = dict(hip_yaw=7, hip_roll=8, hip_pitch=9, knee=10, ankle_pitch=11, ankle_roll=12, sh_pitch=17, sh_roll=18, elbow=19, gripper=20)
    P, SV, J = {}, {}, []
    for sg, ids in ((-1, R_ids), (1, L_ids)):
        p, s, j = build_side(Side(sg), ids)
        P.update(p); SV.update(s); J += j
    # ---- pelvis: two yaw sleeves + crossbar ----
    Yc = 45.5
    pel = Part('pelvis')
    for sg in (-1, 1):
        sd = Side(sg)
        sy = [j['servo'] for j in J if j['name'] == f"{sd.sfx}_hip_yaw"][0]
        pel.sleeve(sy)
    pel.add((-52.5, -27.5, -18.0, 18.0, 320.0, Z_BASE))
    pel.cutbox((-49.9, -30.1, -14.7, 14.7, 323.0, 355.5))
    # ---- torso: tube + shoulder pitch sleeves + neck yaw sleeve ----
    tor = Part('torso')
    x0, x1 = TORSO_X; y0, y1 = TORSO_Y
    tor.add((x0, x1, y0, y1, Z_BASE, TORSO_TOP))
    tor.cutbox((x0 + WALL, x1 - WALL, y0 + WALL, y1 - WALL, Z_BASE + PT, TORSO_TOP - PT))
    # front/back windows
    for xs in ((x0 - 1, x0 + WALL + 1), (x1 - WALL - 1, x1 + 1)):
        tor.cutbox((xs[0], xs[1], -34, 34, Z_BASE + 22, TORSO_TOP - 22))
    # floor mounting holes to pelvis sleeve tops (M3 clearance) and cable passage
    tor.cutbox((-45, -25, -8, 8, Z_BASE - 1, Z_BASE + PT + 1))
    # e-stop hole in top plate
    tor.cut(cqcyl((-50, 30, TORSO_TOP - PT - 1), (0, 0, 1), 11.25, PT + 2))
    # Pi 5 standoffs on inside of back wall: board 85(z) x 56(y); holes 58x49 pattern, 3.5 from corner
    pi_y0, pi_z0 = -47.0, Z_BASE + 14
    for (hx, hy_) in ((3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)):
        cx, cz = pi_y0 + hy_, pi_z0 + hx
        tor.add((x0 + WALL - 0.5, x0 + WALL + 6.5, cx - 3.0, cx + 3.0, cz - 3.0, cz + 3.0))
    for (hx, hy_) in ((3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)):
        cx, cz = pi_y0 + hy_, pi_z0 + hx
        tor.cut(cqcyl((x0 + WALL - 1, cx, cz), (1, 0, 0), 1.2, 8.5))
    # battery strap slots in floor
    for yy in (22.0, 38.0):
        tor.cutbox((-40, -8, yy - 1.5, yy + 1.5, Z_BASE - 1, Z_BASE + PT + 1))
    # shoulder pitch sleeves and neck yaw sleeve
    for sg in (-1, 1):
        sd = Side(sg)
        sv = [j['servo'] for j in J if j['name'] == f"{sd.sfx}_shoulder_pitch"][0]
        tor.sleeve(sv)
    s_ny = Servo(21, 'neck_yaw', (XN, 0, TORSO_TOP + 41.5), (0, 0, 1), (1, 0, 0))
    tor.sleeve(s_ny)
    # ---- neck bracket (child of neck yaw, holds horn plate of neck pitch) ----
    ypn = -22.25
    Znp = 566.5
    s_np = Servo(22, 'neck_pitch', (XN, ypn, Znp), (0, -1, 0), (0, 0, 1))
    nb = Part('neck_bracket')
    nb.plate(s_ny, -15, 15, -15, 30)   # local y' = -world y for this servo frame -> world y in [-30, 15]
    nb.plate(s_np, -15, 15, -15, 15)
    nb.add((XN - 15, XN + 15, ypn - GAP - PT, ypn - GAP, s_ny.S[2] + GAP + PT - 0.5, Znp + 15))
    hd = Part('head')
    hd.sleeve(s_np)
    hd.add((XN + 12.7, XN + 36.0, -17.0, 17.0, Znp - 6, Znp + 37.9))
    hd.cutbox((XN + 18.0, XN + 33.6, -14.6, 14.6, Znp - 3.6, Znp + 40))        # camera pod cavity (open top)
    hd.cutbox((XN + 15.0, XN + 20.0, -4, 4, Znp + 10, Znp + 30))               # cable passage
    hd.cut(cqcyl((XN + 30, 0, Znp + 20), (1, 0, 0), 5.0, 10))                  # lens hole
    P.update({'pelvis': pel, 'torso': tor, 'neck_bracket': nb, 'head': hd})
    SV['neck_yaw'] = s_ny; SV['neck_pitch'] = s_np
    J += [dict(name='neck_yaw', id=21, servo=s_ny, parent='base', child='neck_bracket', lim=(-1.2, 1.2)),
          dict(name='neck_pitch', id=22, servo=s_np, parent='neck_bracket', child='head', lim=(-0.35, 0.35))]
    import json, os
    lf = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'limits.json')
    if os.path.exists(lf):
        lim = json.load(open(lf))
        for j in J:
            j['lim_nom'] = j['lim']
            if j['name'] in lim:
                j['lim'] = tuple(lim[j['name']])
    return P, SV, J

# which printed parts belong to the fixed base link
BASE_PARTS = ['pelvis', 'torso']
