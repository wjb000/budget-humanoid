"""Electronics placement inside torso/head (mm, world). 'real' = vendor CAD (MIT/Apache, decimated visual), 'envelope' = datasheet-size box."""
import numpy as np, trimesh
from robot import *

PI_Y0, PI_Z0, PI_XPLANE = -47.0, Z_BASE + 14, TORSO_X[0] + WALL + 6.5   # board corner (y,z), board rear plane x
ZNP = 566.5

def T(R=np.eye(3), t=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M

# proper rotation (x_s,y_s,z_s) -> (z_s, -y_s, x_s): board/axis along STEP z points to +x world
R_X = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]], float)

def box_env(name, dims, center, **kw):
    m = trimesh.creation.box(extents=dims); m.apply_translation(center)
    return dict(name=name, kind='envelope', mesh=m, **kw)

def real_env(name, V, F, M, **kw):
    m = trimesh.Trimesh(V, F, process=False); m.apply_transform(M)
    return dict(name=name, kind='real', mesh=m, **kw)

def electronics(dec):
    """dec: dict name -> (V,F) decimated real meshes"""
    E = []
    # Pi 5 (real, MIT): STEP board corner origin; z_s=0 board rear plane
    M = np.eye(4); M[:3, :3] = R_X; M[:3, 3] = (PI_XPLANE, PI_Y0 + 56.0, PI_Z0)
    if 'pi5' in dec:
        E.append(real_env('Raspberry Pi 5', *dec['pi5'], M, link='base', mass=46, cat='compute', price='$77.50',
                          spec='Raspberry Pi 5 (88.5 x 57.6 x 19 mm incl. connectors). Vendor STEP, MIT. Decimated visual mesh.',
                          src='Raspberry Pi Ltd product portal STEP (MIT)'))
    else:
        E.append(box_env('Raspberry Pi 5', (19, 57.6, 88.5), (PI_XPLANE + 8, PI_Y0 + 28, PI_Z0 + 42.5), link='base', mass=46, cat='compute', price='$77.50', spec='envelope', src=''))
    E.append(box_env('3S LiPo 2200 mAh (assumed size)', (25, 35, 105), (-20, 26, 415.5), link='base', mass=185, cat='power', price='$19.99',
                     spec='Envelope 105 x 35 x 25 mm: size NOT verified, check your pack', src='listing weight/size varies'))
    E.append(box_env('Pololu D24V50F5 buck', (8.8, 20.3, 17.8), (10.8, 35, 385), link='base', mass=7, cat='power', price='$32.95',
                     spec='Envelope 17.8 x 20.3 x 8.8 mm (from vendor STEP bbox; STEP license unknown, not redistributed)', src='Pololu'))
    E.append(box_env('Waveshare bus adapter A (legs)', (8, 33, 42), (11.2, -31, 424), link='base', mass=8, cat='compute', price='$4.99',
                     spec='Envelope 42 x 33 x 8 mm assumed', src='assumed'))
    E.append(box_env('Waveshare bus adapter B (arms/neck)', (8, 33, 42), (11.2, -31, 466), link='base', mass=8, cat='compute', price='$4.99',
                     spec='Envelope 42 x 33 x 8 mm assumed', src='assumed'))
    E.append(box_env('Seeed XIAO ESP32-S3', (4.5, 17.8, 22.5), (11.8, 25, 431), link='base', mass=3, cat='compute', price='$7.90',
                     spec='Envelope 22.5 x 17.8 x 4.5 mm (vendor STEP bbox; STEP license unknown, not redistributed)', src='Seeed'))
    E.append(box_env('Relay + fuse holder', (20, 20, 30), (-25, -32, 378), link='base', mass=25, cat='power', price='~$12 est.',
                     spec='Envelope assumed 30 x 20 x 20 mm', src='assumed'))
    # BNO085 (real, MIT) lying on torso floor: STEP x,y in plane, z up
    M = np.eye(4); M[:3, 3] = (-22.7, -16.43, Z_BASE + PT)
    if 'bno085' in dec:
        E.append(real_env('Adafruit BNO085 IMU', *dec['bno085'], M, link='base', mass=2, cat='sensor', price='$29.50',
                          spec='Adafruit 4754 STEMMA QT, 25.4 x 22.9 x 4.5 mm. Vendor STEP, MIT.', src='Adafruit_CAD_Parts (MIT)'))
    else:
        E.append(box_env('Adafruit BNO085 IMU', (25.4, 22.86, 4.53), (-10, 0, Z_BASE + PT + 2.3), link='base', mass=2, cat='sensor', price='$29.50', spec='envelope', src=''))
    # Speaker 40 mm (real, MIT) axis along x
    M = np.eye(4); M[:3, :3] = R_X; M[:3, 3] = (TORSO_X[1] - WALL - 25.0, -13.0, 388.0)
    if 'speaker40' in dec:
        E.append(real_env('40 mm speaker', *dec['speaker40'], M, link='base', mass=12, cat='sensor', price='$4.95',
                          spec='Adafruit 3968, 40 mm dia x 25 mm. Vendor STEP, MIT.', src='Adafruit_CAD_Parts (MIT)'))
    M = np.eye(4); M[:3, :3] = R_X; M[:3, 3] = (TORSO_X[1] - WALL - 2.6, -2.0 + 9.5, 420.0 - 8.9)
    if 'max98357' in dec:
        E.append(real_env('MAX98357A I2S amp', *dec['max98357'], M, link='base', mass=2, cat='sensor', price='$5.95',
                          spec='Adafruit 3006, 17.8 x 19.1 x 2.6 mm. Vendor STEP, MIT.', src='Adafruit_CAD_Parts (MIT)'))
    # e-stop envelope (cylinder along z) on top plate
    cyl = trimesh.creation.cylinder(radius=11.0, height=22.0); cyl.apply_translation((-50, 30, TORSO_TOP + 9))
    E.append(dict(name='E-stop (NC latching mushroom)', kind='envelope', mesh=cyl, link='base', mass=15, cat='power', price='$7.50',
                  spec='Envelope: 22.5 mm panel hole, ~22 mm tall body above plate', src='assumed', show=True))
    # XT60 panel-mount charge/discharge port in the battery door (adds ~$1.2 vs. a plain pigtail)
    E.append(box_env('XT60 panel connector (charge port)', (14.0, 15.8, 8.0), (TORSO_X[0] - 4.0, 26.0, 482.0), link='base', mass=6, cat='power', price='$1.20',
                     spec='Envelope; 15.8 x 8.2 mm cutout in battery_door. Cost-adding vs a loose pigtail; flagged in BOM.', src='assumed', show=True))
    # head: camera + mic
    E.append(box_env('Pi Camera Module 3', (10.25, 25.0, 23.86), (XN + 26.0, 0, ZNP + 20), link='head', mass=4, cat='sensor', price='$25',
                     spec='Envelope 23.9 x 25 x 10.3 mm from vendor STEP bbox (STEP license unclear, not redistributed)', src='Raspberry Pi'))
    E.append(box_env('I2S MEMS mic', (12.7, 1.6, 8.0), (XN + 25, 13.6, ZNP + 5), link='head', mass=1, cat='sensor', price='$6.95',
                     spec='Envelope assumed', src='assumed'))
    return E
