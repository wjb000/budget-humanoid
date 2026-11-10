"""Helpers: STS3215 servo frame, sleeve/plate builders (CadQuery). Units: mm."""
import cadquery as cq
import numpy as np

# ---- STS3215 geometry measured from SO-ARM100 STS3215_03a.step (Apache-2.0) ----
# STEP frame: shaft axis = +z through (x=12.5,y=0). Output horn disc r=10 top at z=18.7,
# body main block x[-22.7,22.7] y[-12.4,12.4] z[-15.9,17.0], rear disc r=10 to z=-17.7, rear shaft end z=-19.4.
SHAFT_X = 12.5
Z_HORN_TOP = 18.7
SRV_X0, SRV_X1 = -22.7 - SHAFT_X, 22.7 - SHAFT_X     # -35.2 .. +10.2 (tail side is negative)
SRV_HY = 12.4
SRV_Z0, SRV_Z1 = -17.7 - Z_HORN_TOP, 17.0 - Z_HORN_TOP  # rear disc ... top bump (local z' = z - 18.7)
HORN_PATTERN = 4.9      # 9.8 mm square M3 pattern about shaft on output horn (measured)
CASE_HOLES = [(-20.3 - SHAFT_X, -10.2), (-20.3 - SHAFT_X, 10.2), (4.2 - SHAFT_X, -10.2), (4.2 - SHAFT_X, 10.2)]
WALL = 2.4
CLR = 0.3
PT = 4.0     # horn plate thickness
GAP = 0.1    # plate to horn gap

def unit(v):
    v = np.array(v, float)
    return v / np.linalg.norm(v)

class Servo:
    """Feetech STS3215 placed by shaft point S (center of output horn top face), output direction a,
    and tail direction t (direction from shaft toward the long end of the body)."""
    def __init__(self, sid, name, S, a, t):
        self.id, self.name = sid, name
        self.S = np.array(S, float)
        self.a = unit(a)
        self.t = unit(t)
        self.e1 = -self.t
        self.e3 = self.a
        self.e2 = np.cross(self.e3, self.e1)
        self.R = np.stack([self.e1, self.e2, self.e3], 1)
        assert abs(np.linalg.det(self.R) - 1) < 1e-9
    def w(self, xl, yl, zl):
        return self.S + xl * self.e1 + yl * self.e2 + zl * self.e3
    def lbox(self, x0, x1, y0, y1, z0, z1):
        pts = np.array([self.w(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)])
        return tuple(np.round(np.array([pts[:, 0].min(), pts[:, 0].max(), pts[:, 1].min(), pts[:, 1].max(), pts[:, 2].min(), pts[:, 2].max()]), 4))
    def step_to_world(self):
        """4x4 matrix mapping STEP coordinates -> world."""
        M = np.eye(4)
        M[:3, :3] = self.R
        M[:3, 3] = self.S - self.R @ np.array([SHAFT_X, 0, Z_HORN_TOP])
        return M

def cqbox(b):
    x0, x1, y0, y1, z0, z1 = b
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))

def cqcyl(p, d, r, h):
    """cylinder starting at point p, direction d, radius r, length h"""
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(r, h, cq.Vector(*p), cq.Vector(*d)))

class Part:
    def __init__(self, name, group="printed"):
        self.name, self.group = name, group
        self.adds, self.cuts, self.servos, self.windows = [], [], [], []
    def add(self, b, r=None, sel=None):
        """b = (x0,x1,y0,y1,z0,z1). r/sel: fillet radius and edge selector for this box (default: FILLET on all edges)"""
        self.adds.append((tuple(b), r, sel))
    def addshape(self, wp):
        """add a ready-made CadQuery Workplane (sculpted shell, fillets already applied)"""
        self.adds.append(wp)
    def cut(self, shape):
        self.cuts.append(shape)
    def cutbox(self, b):
        self.cuts.append(cqbox(b))
    def cutround(self, b, r, sel='|Z'):
        self.cuts.append(cqbox(b).edges(sel).fillet(r))
    def insert_hole(self, p, d, depth=6.5):
        """M2 heat-set insert pocket (3.2 mm dia) starting at point p going along unit vector d"""
        self.cut(cqcyl(p, d, 1.6, depth))
    def screw_hole(self, p, d, depth, csk=True):
        """M2 clearance hole (2.4) with countersink, through a cover plate"""
        self.cut(cqcyl(p, d, 1.2, depth))
        if csk:
            self.cut(cqcyl(p, d, 2.2, 1.2))
    def sleeve(self, sv):
        """Printed sleeve that carries servo sv (body inboard of its horn plane)."""
        self.servos.append(sv)
        lb = sv.lbox
        ax = int(np.argmax(np.abs(sv.a)))
        # v3: pod corners parallel to the servo axis are rounded (R 5.0 outside; the pocket corner keeps >= 1.1 mm wall)
        self.add(lb(SRV_X0 - CLR - WALL, SRV_X1 + CLR + WALL, -(SRV_HY + CLR + WALL), SRV_HY + CLR + WALL, -41.5, -3.0), r=self.POD_R, sel=('|X', '|Y', '|Z')[ax])
        # pocket (open to horn side)
        self.cutbox(lb(SRV_X0 - CLR, SRV_X1 + CLR, -(SRV_HY + CLR), SRV_HY + CLR, -34.9, 3.0))
        # rear disc / shaft end recess d21
        self.cut(cqcyl(sv.w(0, 0, -38.5), sv.a, 10.5, 3.75))
        # rear pad recess (measured: STEP rear pad x'[-29.9,-13.0] y+-9.2 z' to -38.1)
        self.cutbox(lb(-30.5, -12.5, -9.7, 9.7, -38.5, -34.8))
        # M2 case screws (4) with counterbore
        for (x, y) in CASE_HOLES:
            self.cut(cqcyl(sv.w(x, y, -42.0), sv.a, 1.2, 7.3))
            self.cut(cqcyl(sv.w(x, y, -42.0), sv.a, 2.2, 3.8))
        # side windows expose the servo body (saves filament, lets the servo read in the model); built last, see build()
        self.windows.append(sv)
        # cable notch in tail end wall
        self.cutbox(lb(SRV_X0 - CLR - WALL - 0.05, SRV_X0 - CLR + 0.01, -6, 6, -30, -2.0))
    def plate(self, sv, x0=-15, x1=15, y0=-15, y1=15, extra=None):
        """Horn plate on the output side of servo sv (this part is the moving child)."""
        lb = sv.lbox
        ax = int(np.argmax(np.abs(sv.a)))
        self.add(lb(x0, x1, y0, y1, GAP, GAP + PT), r=99.0, sel=('|X', '|Y', '|Z')[ax])     # round hub disc (pod cap)
        self.cut(cqcyl(sv.w(0, 0, -1), sv.a, 4.0, GAP + PT + 2))
        for sx in (-1, 1):
            for sy in (-1, 1):
                p = (sx * HORN_PATTERN, sy * HORN_PATTERN)
                self.cut(cqcyl(sv.w(p[0], p[1], -1), sv.a, 1.7, GAP + PT + 2))
                self.cut(cqcyl(sv.w(p[0], p[1], GAP + 2.0), sv.a, 2.9, 3.0))
    FILLET = 2.5
    POD_R = 5.0
    def _round(self, item):
        if not isinstance(item, tuple):
            return item
        b, r0, sel = item
        x0, x1, y0, y1, z0, z1 = b
        m = min(x1 - x0, y1 - y0, z1 - z0)
        if sel == '|Z': m = min(x1 - x0, y1 - y0)
        elif sel == '|X': m = min(y1 - y0, z1 - z0)
        elif sel == '|Y': m = min(x1 - x0, z1 - z0)
        wp = cqbox(b)
        r = min(r0 if r0 is not None else self.FILLET, 0.45 * m if r0 is not None else 0.3 * m)
        if r >= 0.6:
            try:
                wp = (wp.edges(sel) if sel else wp.edges()).fillet(r)
            except Exception:
                pass
        return wp
    def _window_boxes(self, sv, side):
        lb = sv.lbox
        y0 = SRV_HY + CLR + 0.0; y1 = SRV_HY + CLR + WALL + 0.2
        ys = (y0, y1) if side > 0 else (-y1, -y0)
        return cqbox(lb(-28.0, 3.5, ys[0], ys[1], -31.0, -6.0))
    def build(self, windows=False):
        s = self._round(self.adds[0])
        for b in self.adds[1:]:
            s = s.union(self._round(b))
        for c in self.cuts:
            s = s.cut(c)
        s = s.clean()
        self.window_report = []
        if windows:
            for sv in self.windows:
                for side in (1, -1):
                    t = s.cut(self._window_boxes(sv, side))
                    ok = len(t.val().Solids()) == len(s.val().Solids())
                    self.window_report.append((sv.name, side, ok))
                    if ok:
                        s = t
        return s.val()
