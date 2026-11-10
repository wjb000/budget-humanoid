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
FW = 34.0                   # front opening half-width (y)
WZ0, WZ1 = Z_BASE + 6.0, TORSO_TOP - 6.0     # opening z range (364.5 .. 494)
BAT_Y0, BAT_Y1 = 8.5, 43.5  # battery bay (3S 2200 mAh assumed 105 x 35 x 25 mm), y range
BY0, BY1 = 7.8, 44.2        # battery door opening (y)
COVER_PTS_F = [(-40.0, Z_BASE + 10.0), (40.0, Z_BASE + 10.0), (-40.0, TORSO_TOP - 8.0), (40.0, TORSO_TOP - 8.0)]
COVER_PTS_B = [(4.0, Z_BASE + 10.0), (46.8, Z_BASE + 10.0), (4.0, TORSO_TOP - 8.0), (46.8, TORSO_TOP - 8.0)]
ESTOP_X, ESTOP_Y = -30.0, 36.0          # e-stop axis on the torso top plate
COVER_HOST = {'torso_front_cover': 'base', 'battery_door': 'base', 'head_lid': 'head'}

def rbox(b, r, top=None, vsel='|Z'):
    """rounded box: vertical corner radius r, optional top-edge fillet"""
    wp = cqbox(b).edges(vsel).fillet(r)
    if top:
        wp = wp.faces('>Z').edges().fillet(top)
    return wp

def ring_groove(b, r, depth=0.8, width=1.2):
    """thin shell-shaped cutter that scribes a panel line around a rounded box of outline b (z centre = mean of b z)"""
    x0, x1, y0, y1, z0, z1 = b
    zc = 0.5 * (z0 + z1)
    A = rbox((x0 - 1.0, x1 + 1.0, y0 - 1.0, y1 + 1.0, zc - width / 2, zc + width / 2), r + 1.0)
    B = rbox((x0 + depth, x1 - depth, y0 + depth, y1 - depth, zc - width, zc + width), max(r - depth, 0.5))
    return A.cut(B)

def loft_shell(origin, xdir, normal, secs, wall=2.4, crest_fillet=2.5):
    """Tapered shell grown from a base plane. secs = [(depth, cu, cv, W, H, r), ...] rounded-rectangle sections in the plane's (u, v) axes
    at increasing depth along `normal`. Returns (outer, inner): the inner cutter is the outer offset by `wall` (the base plane is closed by the part's own plate)."""
    pl = cq.Plane(origin=origin, xDir=xdir, normal=normal)
    def mk(ss, dw, d_end_trim):
        sk = []
        for (d, cu, cv, W, H, r) in ss:
            s_ = cq.Sketch().rect(max(W - 2 * dw, 1.0), max(H - 2 * dw, 1.0)).vertices().fillet(max(min(r - dw, (min(W, H) - 2 * dw) * 0.45), 0.4))
            sk.append(s_.moved(cq.Location(cq.Vector(cu, cv, d - (d_end_trim if d == ss[-1][0] else 0)))))
        return cq.Workplane(pl).placeSketch(*sk).loft(ruled=True)
    outer = mk(secs, 0.0, 0.0)
    try:
        outer = outer.faces('>Z' if False else cq.selectors.DirectionMinMaxSelector(cq.Vector(*normal), True)).edges().fillet(crest_fillet)
    except Exception:
        pass
    inner = mk(secs, wall, wall)
    return outer, inner

def mir(sd, wp):
    return wp.mirror('XZ') if sd.sg > 0 else wp

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
    foot.addshape(cqbox(sd.b(XAR + GAP, XAR + GAP + PT, Yc - 15, Yc + 15, 0, 40)).faces('>Z').edges('|X').fillet(14.5))   # upright plate, top is a half-round that blends into the hub disc
    foot.addshape(cqbox(sd.b(-50, 72, Yc - 28, Yc + 28, 0, 4)).edges('|Z').fillet(14.0).faces('>Z').edges().fillet(1.6))  # sole, softened top edge
    # v4 sculpted foot: two closed hollow shells on the sole replace the gussets and walls: a toe ramp from the upright plate (kept below the four horn screws so they stay reachable) to the toe tip and a heel wedge
    # (printed sole-down; roofs slope 20-25 deg from horizontal, so they print as shallow bridged skins; the shells are closed against the plate and the sole)
    XP = XAR + GAP + PT
    for (x0_, nx, xd, secs) in ((XP - 0.5, (1, 0, 0), (0, 1, 0), [(0, 0, 10.0, 34.0, 12.0, 5.0), (22.0, 0, 8.5, 40.0, 9.0, 4.5), (44.0, 0, 7.0, 44.0, 6.0, 2.9)]),
                                (-22.0, (-1, 0, 0), (0, -1, 0), [(0, 0, 14.0, 44.0, 20.0, 8.0), (24.0, 0, 7.5, 38.0, 7.0, 5.0)])):
        o_, i_ = loft_shell((x0_, Yc, 0), xd, nx, secs, crest_fillet=2.0)
        foot.addshape(mir(sd, o_)); foot.cut(mir(sd, i_))
    # ankle link: ankle pitch hub + roll sleeve
    al = Part(n('ankle_link'))
    al.sleeve(s_ar)
    al.plate(s_ap, -15, 15, -15, 15)
    al.add(sd.b(-15, 15, YA + GAP, YA + GAP + PT, Zar + 37.0, Zap), r=6.0, sel='|Y')   # hub to sleeve top (overlap), ends in the hub disc
    # shin: knee hub + strip + ankle-pitch sleeve
    sh = Part(n('shin'))
    sh.plate(s_k, -15, 15, -15, 15)
    sh.sleeve(s_ap)
    sh.add(sd.b(-15, 15, shin_out, shin_in, Zap + 35, Zk), r=6.0, sel='|Y')
    # v4 shin shell: a closed tapered bulge on the outboard face (covers the strip and the ankle pod end), crest down on the bed, no supports
    o_, i_ = loft_shell((0, shin_out, 0), (1, 0, 0), (0, -1, 0), [(0, 0, 112.0, 30.2, 86.0, 8.0), (13.0, 0, 108.0, 14.0, 62.0, 5.0)])
    sh.addshape(mir(sd, o_)); sh.cut(mir(sd, i_))
    # thigh: hip pitch hub + strip + knee sleeve
    th = Part(n('thigh'))
    th.plate(s_hp, -15, 15, -15, 15)
    th.sleeve(s_k)
    th.add(sd.b(-15, 15, plate_out, plate_in, Zk + 35, Zh), r=6.0, sel='|Y')
    # v4 thigh shell: closed tapered bulge on the outboard face above the knee pod (the hand hangs right beside it, so it is only 13 mm deep)
    o_, i_ = loft_shell((0, plate_out, 0), (1, 0, 0), (0, -1, 0), [(0, 0, 237.0, 30.2, 44.0, 8.0), (13.0, 0, 235.0, 14.0, 28.0, 5.0)])
    th.addshape(mir(sd, o_)); th.cut(mir(sd, i_))
    # hip roll link: roll hub plate + bridge + pitch sleeve
    hr = Part(n('hip_roll_link'))
    hr.plate(s_hr, -15, 15, -9.5, 9.5, cover=False)
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
        dict(name=n('knee'), id=ids['knee'], servo=s_k, parent=th.name, child=sh.name,
             lim=((-0.1, 2.0) if sd.sg > 0 else (-2.0, 0.1))),  # +flexion: right axis is authored -y, left is the y-flipped copy, so limits mirror
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
    Xr = XT + 22.25                     # (v1 shoulder-roll axis position, kept as a geometry reference for the rigid upper arm)
    Yr = ys - 15.2
    Zr = 412.0
    Xe = Xr + 16.65
    Ye = -90.7
    Ze = 359.0
    s_el = sd.servo(ids['elbow'], n('elbow'), (Xe, Ye, Ze), (0, 1, 0), (0, 0, 1))
    Zg = 305.0
    Yg = (Ye + GAP + PT) - 41.5 - 0.0     # gripper rear wall inner end at forearm plate
    s_gr = sd.servo(ids['gripper'], n('gripper'), (Xe, Yg, Zg), (0, -1, 0), (0, 0, -1))
    # v2: the shoulder link and upper arm are ONE printed part (no shoulder roll): hub disc on the shoulder-pitch horn,
    # a strip down the torso side, a bridge to the elbow plane, and the elbow sleeve
    ua = Part(n('upper_arm'))
    ua.plate(s_sp, -15, 15, -15, 15)
    ua.add(sd.b(Xr - 15, Xr - 3.6, ys - GAP - PT, ys - GAP, Zr + 12, Zs + 15), r=4.5, sel='|Y')
    ua.add(sd.b(Xr - 15, Xr + GAP + PT, ys - GAP - PT, ys - GAP, Zr - 12, Zr + 14))      # joins the shoulder strip to the elbow-plane plate
    ua.sleeve(s_el)
    ua.add(sd.b(Xr + GAP, Xr + GAP + PT, Ye - 41.5, Ye - 3, Ze - 12.9, Zr + 11))
    # v4 upper-arm shell: a closed tapered bulge on the back of the elbow plate (hides the plate edge chain from behind), printed plate-down, no supports
    o_, i_ = loft_shell((Xr + GAP, 0, 0), (0, -1, 0), (-1, 0, 0), [(0, -(Ye - 22.25), 384.5, 38.0, 77.0, 9.0), (14.0, -(Ye - 22.25), 384.5, 22.0, 52.0, 6.0)])
    ua.addshape(mir(sd, o_)); ua.cut(mir(sd, i_))
    fa = Part(n('forearm'))
    fa.plate(s_el, -12, 12, -12, 12)
    fa.sleeve(s_gr)
    fa.add(sd.b(Xe - 12, Xe + 12, Ye + GAP, Ye + GAP + PT, Zg - 37.9, Ze), r=5.0, sel='|Y')
    fa.add(sd.b(Xe + 14, Xe + 22, Yg + 3, Yg + 41.5 + 0.5, Zg - 75, Zg - 30))      # fixed jaw
    fi = Part(n('finger'))
    fi.plate(s_gr, -12, 12, -12, 12)
    fi.add(sd.b(Xe - 12, Xe + 4, Yg - GAP - PT, Yg - GAP, Zg - 75, Zg + 12), r=5.0, sel='|Y')
    fi.add(sd.b(Xe - 6, Xe + 4, Yg - GAP - PT, Yg + 25, Zg - 75, Zg - 55))
    P.update({p.name: p for p in (ua, fa, fi)})
    for s in (s_sp, s_el, s_gr):
        SV[s.name] = s
    J += [
        dict(name=n('shoulder_pitch'), id=ids['sh_pitch'], servo=s_sp, parent='base', child=ua.name, lim=(-2.0, 2.0)),
        dict(name=n('elbow'), id=ids['elbow'], servo=s_el, parent=ua.name, child=fa.name, lim=(-1.5, 1.5)),
        dict(name=n('gripper'), id=ids['gripper'], servo=s_gr, parent=fa.name, child=fi.name,
             lim=((-0.2, 0.5) if sd.sg > 0 else (-0.5, 0.2))),  # left joint axis is the y-flipped copy, so its limits mirror
    ]
    return P, SV, J

def build_all():
    R_ids = dict(hip_yaw=1, hip_roll=2, hip_pitch=3, knee=4, ankle_pitch=5, ankle_roll=6, sh_pitch=13, elbow=14, gripper=15)
    L_ids = dict(hip_yaw=7, hip_roll=8, hip_pitch=9, knee=10, ankle_pitch=11, ankle_roll=12, sh_pitch=16, elbow=17, gripper=18)
    P, SV, J = {}, {}, []
    for sg, ids in ((-1, R_ids), (1, L_ids)):
        p, s, j = build_side(Side(sg), ids)
        P.update(p); SV.update(s); J += j
    # ---- pelvis: two yaw sleeves + crossbar ----
    Yc = 45.5
    # ---- torso: tube + shoulder pitch sleeves + neck yaw sleeve ----
    tor = Part('torso')
    # v2: the pelvis (two hip-yaw sleeves + crossbar) is part of the torso print (one part, ~185 mm tall, fits a 220 mm bed)
    for sg in (-1, 1):
        sd = Side(sg)
        sy = [j['servo'] for j in J if j['name'] == f"{sd.sfx}_hip_yaw"][0]
        tor.sleeve(sy)
    tor.add((-52.5, -27.5, -18.0, 18.0, 320.0, Z_BASE))
    tor.cutbox((-49.9, -30.1, -14.7, 14.7, 323.0, 355.5))
    x0, x1 = TORSO_X; y0, y1 = TORSO_Y
    TR = 6.5                                                    # v3 sculpted torso: R6.5 vertical corners (R4.1 inside = uniform 2.4 wall), R5 shoulder-top edge
    tor.addshape(rbox((x0, x1, y0, y1, Z_BASE, TORSO_TOP), TR, top=5.0))
    tor.cutround((x0 + WALL, x1 - WALL, y0 + WALL, y1 - WALL, Z_BASE + PT, TORSO_TOP - PT), TR - WALL, '|Z')
    # v3 pelvis skirt: the torso walls continue down to the hip sleeves (open at the bed side, the floor above rests on the walls, no supports)
    sk = rbox((x0, x1, y0, y1, 320.0, Z_BASE + 1.0), TR).cut(cqbox((x0 + WALL, x1 - WALL, y0 + WALL, y1 - WALL, 319.0, Z_BASE + 0.01)).edges('|Z').fillet(TR - WALL))
    tor.addshape(sk)
    # panel lines (0.8 deep, 1.2 wide): skirt seam, belt line below the shoulder pods
    for zl in (Z_BASE, 421.0):
        tor.cut(ring_groove((x0, x1, y0, y1, zl - 0.6, zl + 0.6), TR))
    # front opening (= access to Pi, XIAO, bus adapters, and the battery path) closed by torso_front_cover
    tor.cutbox((x1 - WALL - 1, x1 + 1, -FW, FW, WZ0, WZ1))
    # back opening for the battery bay, closed by battery_door
    tor.cutbox((x0 - 1, x0 + WALL + 1, BY0, BY1, WZ0, WZ1))
    # floor mounting holes to pelvis sleeve tops (M3 clearance) and cable passage
    tor.cutbox((-45, -25, -8, 8, Z_BASE - 1, Z_BASE + PT + 1))
    # e-stop (16.2 mm panel hole + anti-rotation key) in top plate
    tor.cut(cqcyl((ESTOP_X, ESTOP_Y, TORSO_TOP - PT - 1), (0, 0, 1), 8.2, PT + 2))                       # IDEC X6/XA dia 16.2 hole
    tor.cutbox((ESTOP_X + 7.0, ESTOP_X + 8.95 + 0.2, ESTOP_Y - 1.5, ESTOP_Y + 1.5, TORSO_TOP - PT - 1, TORSO_TOP + 1))   # 1.7 mm anti-rotation key (17.9 across)
    # M2 heat-set insert bosses for the front cover (4) and battery door (4); holes drilled from outside
    for (yy, zz) in COVER_PTS_F:
        tor.add((x1 - WALL - 4.5, x1 - WALL + 0.5, yy - 3.2, yy + 3.2, zz - 3.2, zz + 3.2), r=0.8)
        tor.insert_hole((x1 + 0.01, yy, zz), (-1, 0, 0), 6.5)
    for (yy, zz) in COVER_PTS_B:
        tor.add((x0 + WALL - 0.5, x0 + WALL + 4.5, yy - 3.2, min(yy + 3.2, 49.0), zz - 3.2, zz + 3.2), r=0.8)
        tor.insert_hole((x0 - 0.01, yy, zz), (1, 0, 0), 6.5)
    # Pi 5 standoffs on inside of back wall: board 85(z) x 56(y); holes 58x49 pattern, 3.5 from corner
    pi_y0, pi_z0 = -47.0, Z_BASE + 14
    for (hx, hy_) in ((3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)):
        cx, cz = pi_y0 + hy_, pi_z0 + hx
        tor.add((x0 + WALL - 0.5, x0 + WALL + 6.5, cx - 3.0, cx + 3.0, cz - 3.0, cz + 3.0), r=0.8)
    for (hx, hy_) in ((3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)):
        cx, cz = pi_y0 + hy_, pi_z0 + hx
        tor.cut(cqcyl((x0 + WALL - 1, cx, cz), (1, 0, 0), 0.85, 8.5))
    # battery bay: two cradle ribs on the floor, strap slots, cable tie slots
    for (ya, yb) in ((BAT_Y0 - 1.7, BAT_Y0 - 0.2), (BAT_Y1 + 0.2, BAT_Y1 + 1.7)):
        tor.add((-33.0, -7.0, ya, yb, Z_BASE + PT - 0.5, Z_BASE + PT + 22), r=0.5)
    for yy in (BAT_Y0 + 6.0, BAT_Y1 - 6.0):
        tor.cutbox((-40, -8, yy - 1.5, yy + 1.5, Z_BASE - 1, Z_BASE + PT + 1))
    for zz in (385.0, 410.0):                                   # cable-tie slot pairs through the side walls (below the shoulder sleeves)
        for sg in (-1, 1):
            tor.cutbox((-20.0, -16.8, min(sg * 49.1, sg * 52.5), max(sg * 49.1, sg * 52.5), zz - 0.8, zz + 0.8))
    # shoulder pitch sleeves and neck yaw sleeve
    for sg in (-1, 1):
        sd = Side(sg)
        sv = [j['servo'] for j in J if j['name'] == f"{sd.sfx}_shoulder_pitch"][0]
        tor.sleeve(sv)
    s_ny = Servo(21, 'neck_yaw', (XN, 0, TORSO_TOP + 41.5), (0, 0, 1), (1, 0, 0))
    tor.sleeve(s_ny)
    # v4 neck collar: a rounded wall around the neck pod hides the pod and its cable notch; cables drop through a slot in the torso top plate
    outer_c = cq.Workplane('XY').add(cqbox((-56.3, 6.0, -18.5, 18.5, TORSO_TOP - 1.0, TORSO_TOP + 40.0)).edges('|Z').fillet(8.0).val())
    inner_c = cqbox((-53.9, 3.6, -16.1, 16.1, TORSO_TOP - 2.0, TORSO_TOP + 41.0)).edges('|Z').fillet(5.6)
    tor.addshape(outer_c.cut(inner_c))
    tor.cutbox((-1.0, 3.0, -6.0, 6.0, TORSO_TOP - PT - 1.0, TORSO_TOP))

    # ---- torso front cover (chest panel; also the access door to Pi / XIAO / bus adapters / battery path) ----
    fc = Part('torso_front_cover')
    fc.addshape(cqbox((x1, x1 + 2.4, -46.0, 46.0, Z_BASE + 4.0, TORSO_TOP - 5.0)).edges('|X').fillet(5.0).faces('>X').edges().chamfer(0.9))
    fc.add((x1 - WALL, x1, -FW + 0.4, FW - 0.4, WZ0 + 0.4, WZ1 - 0.4))
    for (yy, zz) in COVER_PTS_F:
        fc.screw_hole((x1 + 2.4 + 0.01, yy, zz), (-1, 0, 0), 2.5)
    for zc in (424.0, 466.0):                                                   # bus adapter A/B: 4 M2 self-tapping bosses each (37 x 28 hole pattern), board hangs off the cover
        for dz in (-18.5, 18.5):
            for dy in (-14.0, 14.0):
                fc.add((11.7, x1 - WALL + 0.1, -31.0 + dy - 2.6, -31.0 + dy + 2.6, zc + dz - 2.6, zc + dz + 2.6), r=0.8)
                fc.cut(cqcyl((11.6, -31.0 + dy, zc + dz), (1, 0, 0), 0.85, 7.0))
            fc.add((13.4, x1 - WALL + 0.1, -47.6, -33.0, zc + dz - 2.6, zc + dz + 2.6), r=0.8)     # v3: tie the outer boss row to the cover insert (they were floating islands in v2)
    for dy in (-8.0, -4.0, 0.0, 4.0, 8.0):                                      # speaker grille (speaker centre y=-13, z=388)
        fc.cutbox((x1 - WALL - 0.1, x1 + 2.5, -13.0 + dy - 1.1, -13.0 + dy + 1.1, 376.0, 400.0))
    for xz in (14.2, 34.2):                                                     # XIAO ESP32-S3 cradle (board has no mounting holes): two rails + two lips
        fc.add((8.4, x1 - WALL + 0.1, xz, xz + 1.6, 417.5, 444.5), r=0.4)
    fc.add((8.4, x1 - WALL + 0.1, 14.2, 35.8, 417.5, 419.5), r=0.4)
    fc.add((8.4, x1 - WALL + 0.1, 14.2, 35.8, 442.5, 444.5), r=0.4)
    for zc in (410.0, 452.0, 486.0):                                              # panel lines (0.6 mm deep, 1 mm wide)
        fc.cutbox((x1 + 2.4 - 0.6, x1 + 2.5, -43.0, 43.0, zc - 0.5, zc + 0.5))
    # ---- battery door (back, with XT60 charge-port cutout) ----
    bd = Part('battery_door')
    bd.addshape(cqbox((x0 - 2.4, x0, 0.5, 49.5, Z_BASE + 4.0, TORSO_TOP - 5.0)).edges('|X').fillet(4.5).faces('<X').edges().chamfer(0.9))
    bd.add((x0, x0 + WALL, BY0 + 0.4, BY1 - 0.4, WZ0 + 0.4, WZ1 - 0.4))
    for (yy, zz) in COVER_PTS_B:
        bd.screw_hole((x0 - 2.4 - 0.01, yy, zz), (1, 0, 0), 2.5)
    bd.cutbox((x0 - 2.5, x0 + WALL + 0.1, 26.0 - 8.1, 26.0 + 8.1, 482.0 - 4.1, 482.0 + 4.1))   # XT60 panel-mount charge/discharge port
    # ---- neck bracket (child of neck yaw, holds horn plate of neck pitch) ----
    # v2: one neck joint (yaw). The neck bracket and the head are ONE printed part: hub disc on the yaw horn, a riser, the camera head
    hd = Part('head')
    hd.plate(s_ny, -15, 15, -15, 15)
    Znp = 566.5
    HX0, HX1, HY = XN - 14.0, XN + 36.0, 19.0
    HZ0, HZ1 = s_ny.S[2] + GAP + PT - 0.5, Znp + 37.9
    ZEYE = Znp + 20.0
    # v3: one continuous rounded head box (R8 corners) instead of riser + pod; open at the top, closed by the head lid
    hd.addshape(rbox((HX0, HX1, -HY, HY, HZ0, HZ1), 8.0))
    hd.cutbox((XN - 9.0, XN + 33.6, -14.6, 14.6, Znp - 3.6, Znp + 40))         # one open chamber: camera at the front wall, cables behind it
    hd.cutbox((XN - 8.5, XN + 8.5, -8.5, 8.5, s_ny.S[2] + GAP + PT, Znp + 40))   # screw/cable shaft from the lid down to the neck horn screws
    # eye: raised bezel ring, lens bore, engraved ring, and two side "ear" discs
    hd.addshape(cqcyl((XN + 36.0 - 0.5, 0, ZEYE), (1, 0, 0), 10.0, 3.0))
    hd.cut(cqcyl((XN + 28.0, 0, ZEYE), (1, 0, 0), 5.0, 14.0))                  # lens hole
    hd.cut(cqcyl((XN + 38.5 - 0.8, 0, ZEYE), (1, 0, 0), 8.0, 1.0).cut(cqcyl((XN + 38.5 - 0.8, 0, ZEYE), (1, 0, 0), 7.0, 1.0)))
    for sy_ in (-1, 1):
        hd.addshape(cqcyl((XN + 14.0, sy_ * (HY - 0.5), ZEYE - 2.0), (0, sy_, 0), 7.5, 3.0))
    # v4 head character: sloped brow hood over the eye, vent slots under it, cheek ring grooves round the ear discs
    hd.addshape(cqbox((HX1 - 2.0, HX1 + 8.0, -14.5, 14.5, ZEYE + 11.0, ZEYE + 15.0)).edges('|Z').fillet(4.5))
    hd.addshape(cqbox((HX1 - 2.0, HX1 + 2.0, -14.5, 14.5, ZEYE + 14.0, ZEYE + 17.0)).edges('|Z').fillet(1.9))
    for yy in (-8.0, -4.0, 0.0, 4.0, 8.0):
        hd.cut(cqbox((HX1 - 1.0, HX1 + 1.0, yy - 0.8, yy + 0.8, Znp + 6.0, Znp + 13.0)))
    for sy_ in (-1, 1):
        hd.cut(cqcyl((XN + 14.0, sy_ * (HY + 0.1), ZEYE - 2.0), (0, sy_, 0), 9.4, 0.9).cut(cqcyl((XN + 14.0, sy_ * (HY + 0.1), ZEYE - 2.0), (0, sy_, 0), 8.4, 0.9)))
    for zl in (Znp + 3.0, Znp + 33.0):                                          # panel lines
        hd.cut(ring_groove((HX0, HX1, -HY, HY, zl - 0.6, zl + 0.6), 8.0, depth=0.7, width=1.0))
    # head lid (closes the chamber; four M2 self-tapping screws into the back and side walls)
    HSCREWS = [(XN - 11.5, -8.0), (XN - 11.5, 8.0), (XN + 12.0, -16.8), (XN + 12.0, 16.8)]
    hl = Part('head_lid')
    hl.addshape(rbox((HX0, HX1, -HY, HY, HZ1, HZ1 + 5.0), 8.0, top=2.4))
    hl.add((XN - 8.6, XN + 33.2, -14.2, 14.2, HZ1 - 3.0, HZ1))
    hl.addshape(cqcyl((XN + 22.0, 0, HZ1 + 4.6), (0, 0, 1), 8.5, 5.0).faces('>Z').edges().fillet(2.0))   # radar dome
    hl.addshape(cqcyl((XN - 4.0, 0, HZ1 + 4.6), (0, 0, 1), 1.5, 15.0))                                    # antenna stub
    for (xx, yy) in HSCREWS:
        hl.screw_hole((xx, yy, HZ1 + 5.0 + 0.01), (0, 0, -1), 6.0)
        hd.cut(cqcyl((xx, yy, HZ1 + 0.01), (0, 0, -1), 0.9, 9.0))
    P.update({'torso': tor, 'head': hd, 'torso_front_cover': fc, 'battery_door': bd, 'head_lid': hl})
    SV['neck_yaw'] = s_ny
    J += [dict(name='neck_yaw', id=19, servo=s_ny, parent='base', child='head', lim=(-1.2, 1.2))]
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
BASE_PARTS = ['torso', 'torso_front_cover', 'battery_door']
