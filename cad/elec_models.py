"""Modelled (not vendor-STEP) electronics: built in CadQuery from published outline dimensions. Every function lists its sources
and which features are datasheet values and which are approximate. Local frames: documented per part, placed in electronics.py."""
import numpy as np, trimesh, cadquery as cq

def _mesh(wp, tol=0.05):
    from assembly import tess_to_mesh
    s = wp.val() if hasattr(wp, 'val') else wp
    return tess_to_mesh(s, tol, 0.3)

def box(x, y, z, sx, sy, sz, r=0.0):
    """box centred at (x,y,z)"""
    w = cq.Workplane('XY').box(sx, sy, sz).translate((x, y, z))
    if r > 0:
        try: w = w.edges().fillet(min(r, 0.45 * min(sx, sy, sz)))
        except Exception: pass
    return w

def cyl(x, y, z0, r, h, axis='z'):
    d = {'z': (0, 0, 1), 'x': (1, 0, 0), 'y': (0, 1, 0)}[axis]
    return cq.Workplane('XY').add(cq.Solid.makeCylinder(r, h, cq.Vector(x, y, z0), cq.Vector(*d)))

def pcb(L, W, T, holes=(), hole_d=2.5):
    """L x W x T board, centred in XY, bottom at z=0, mounting holes at (x,y) offsets from centre"""
    b = cq.Workplane('XY').box(L, W, T, centered=(True, True, False))
    b = b.edges('|Z').fillet(0.8)
    for (hx, hy) in holes:
        b = b.cut(cyl(hx, hy, -1, hole_d / 2, T + 2))
    return b

# ---- Pololu D24V50F5: 17.8 x 20.3 x 8.8 mm overall, two 0.086 in holes at opposite corners 0.53 in x 0.63 in apart
# Sources: pololu.com/product/2851 (size 0.7 x 0.8 x 0.35 in; mounting holes). Inductor/IC blocks approximate.
def pololu_d24v50f5():
    """local: board long side along Y (20.3), short along X (17.8), PCB bottom z=0, top of tallest part z=8.8"""
    hx, hy = 13.5 / 2, 16.0 / 2
    b = pcb(17.8, 20.3, 1.6, holes=[(-hx, -hy), (hx, hy)], hole_d=2.2)
    b = b.union(box(0.0, 2.0, 1.6 + 3.6, 10.0, 10.0, 7.2, 0.8))                                      # shielded inductor (approx)
    b = b.union(box(-5.0, -7.2, 1.6 + 0.9, 3.5, 4.0, 1.8))                                           # controller IC (approx)
    return b

# ---- Waveshare Bus Servo Adapter (A): 42 x 33 mm, 4 x 2.5 mm holes on 37 x 28 mm (waveshare.com/product/bus-servo-adapter-a.htm).
# DC jack / terminal blocks / headers: approximate generic parts (outline of the board and holes are the datasheet values)
def waveshare_adapter_a():
    """local: long side X (42), short Y (33), PCB bottom z=0"""
    b = pcb(42.0, 33.0, 1.6, holes=[(sx * 18.5, sy * 14.0) for sx in (-1, 1) for sy in (-1, 1)], hole_d=2.5)
    b = b.union(box(-16.5, 0.0, 1.6 + 5.5, 14.0, 9.0, 11.0, 0.5))                                   # 5.5 x 2.1 DC jack (approx body)
    b = b.union(box(-17.0, 11.0, 1.6 + 4.0, 10.0, 7.0, 8.0, 0.4))                                   # power screw terminal (approx)
    for k in range(2):
        b = b.union(box(12.0, -9.0 + k * 9.0, 1.6 + 4.0, 7.0, 7.0, 8.0, 0.3))                      # servo 3-pin connectors (approx)
    b = b.union(box(3.0, 0.0, 1.6 + 0.9, 9.0, 7.0, 1.8))                                            # driver IC (approx)
    return b

# ---- Seeed XIAO ESP32-S3: 21 x 17.8 mm board, USB-C overhang to 22.5 mm, 4.5 mm high incl. parts (Seeed wiki outline)
def xiao_esp32s3():
    """local: board long side along X (22.5 total incl. USB-C), width Y 17.8, PCB bottom z=0"""
    b = pcb(21.0, 17.8, 1.0)
    b = b.union(box(-7.5, 0.0, 1.0 + 1.6, 9.0, 8.94, 3.2, 0.6))                              # USB-C receptacle, overhangs the edge
    b = b.union(box(3.0, 0.0, 1.0 + 1.1, 12.0, 13.0, 2.2))                                         # RF shield can (approx)
    return b

# ---- Raspberry Pi Camera Module 3 (standard): 25 x 24 x 11.5 mm, 4 x dia 2.2 holes on 21 x 12.5 mm
# Sources: Raspberry Pi camera module 3 product brief + mechanical drawing RP-008153. Lens barrel dia and position approximate.
def picam3():
    """local: board 25 (X) x 24 (Y); PCB bottom z=0; lens axis +z, top at z=11.5. Ribbon exits -Y edge (not modelled beyond a stub)"""
    holes = [(sx * 10.5, sy * 6.25) for sx in (-1, 1) for sy in (-1, 1)]
    b = pcb(25.0, 24.0, 1.0, holes=holes, hole_d=2.2)
    b = b.union(box(0.0, 0.0, 1.0 + 2.0, 8.5, 8.5, 4.0 - 0.0))                                      # sensor + AF actuator base (approx 8.5 sq)
    b = b.union(cyl(0.0, 0.0, 1.0, 4.15, 11.5 - 1.0 - 0.0))                                        # lens barrel dia 8.3
    b = b.faces('>Z').workplane().circle(3.4).cutBlind(-1.2)                                          # lens glass recess (dia ~6.9)
    b = b.union(box(0.0, -12.0 + 1.5, 1.0 + 0.5, 10.0, 4.0, 1.0))                                   # FPC connector (approx)
    return b

# ---- XT60 panel connector (Amass XT60 family): housing face 16.0 x 8.1 mm, 20.5 mm long, contacts 7.2 mm apart, dia 3.5 mm bullets.
# Sources: Amass XT60 drawing (hobbydex.com/rc/specs/connector/xt60, electricsizes.com). Panel flange per XT60E: approximate.
def xt60_panel():
    """local: mating face at z=0 facing -z (towards the outside), body extends +z (into the robot), 16.0 (X) x 8.1 (Y), corner chamfer on one end = polarity"""
    L = 18.0   # inside depth used here (body 20.5 incl. the 2.5 mm in the panel cut-out zone); contacts at 7.2 pitch
    body = cq.Workplane('XY').box(16.0, 8.1, L, centered=(True, True, False))
    # polarity: chamfer the two corners at the -X end (XT60 housing has one bevelled end)
    body = body.edges('|Z and <X').chamfer(1.6)
    for sx in (-1, 1):
        body = body.cut(cyl(sx * 3.6, 0, -0.01, 1.9, 8.0))        # contact sockets, dia ~3.8 for 3.5 mm bullets
    return body

# ---- e-stop: IDEC/APEM dia 16 panel-hole unibody emergency stop (X6 / XA): 30 mm mushroom, 20.6 mm above the panel,
# 19.5 mm behind the panel (solder terminals), dia 16.2 +0.2 hole with 1.7 mm anti-rotation key. Sources: IDEC X6 / XA datasheets.
def estop16():
    """local: panel face at z=0, button up (+z): head z 0..20.6, body behind to z=-19.5. Internal profile approximate"""
    top = cyl(0, 0, 8.0, 15.0, 12.6)
    try: top = top.faces('>Z').edges().fillet(5.0)
    except Exception: pass
    neck = cyl(0, 0, 0.0, 10.0, 8.5)                                   # operator neck above the locking ring
    PT = 4.0                                                           # printed panel (torso top plate) thickness
    ring = cyl(0, 0, -PT - 3.1, 11.5, 3.1)                             # locking ring under the panel
    thread = cyl(0, 0, -PT - 19.5, 7.9, 19.5 + PT)                     # dia 15.8 body through the panel, 19.5 mm behind it
    term = box(0, 0, -PT - 17.0, 12.0, 14.0, 5.0)                      # contact block / #110 tab zone (approx)
    return top.union(neck).union(ring).union(thread).union(term)

# ---- LiPo 3S 2200 mAh (typical hobby pack, dims are a typical/assumed outline NOT from one product sheet)
def lipo3s():
    """local: long axis Z (105), thickness X (25), width Y (35); XT60 + balance lead stubs on +Z end"""
    b = box(0, 0, 0, 25.0, 35.0, 105.0, 3.0)
    b = b.union(box(-4.0, 6.0, 52.5 + 4.0, 3.0, 12.0, 8.0))            # balance lead stub (approx)
    b = b.union(box(3.0, -6.0, 52.5 + 6.0, 7.0, 16.0, 12.0, 0.5))      # XT60 plug on pack lead (approx, not to scale of wire)
    return b

def place(mesh, R=np.eye(3), t=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t
    m = mesh.copy(); m.apply_transform(M); return m
