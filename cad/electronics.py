"""Electronics placement inside torso/head (mm, world). 'real' = vendor CAD (MIT/Apache, decimated visual), 'envelope' = datasheet-size box."""
import numpy as np, trimesh
from robot import *
import elec_models as EM

PI_Y0, PI_Z0, PI_XPLANE = -47.0, Z_BASE + 14, TORSO_X[0] + WALL + 6.5   # board corner (y,z), board rear plane x
ZNP = 566.5

def T(R=np.eye(3), t=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M

# proper rotation (x_s,y_s,z_s) -> (z_s, -y_s, x_s): board/axis along STEP z points to +x world
R_XF = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float)   # board normal -> -x (components face the robot's rear)
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
    def mod(name, m, R, t, **kw):
        return dict(name=name, kind='modeled', mesh=EM.place(m, R, t), **kw)
    E.append(mod('3S LiPo 2200 mAh (typical outline)', EM._mesh(EM.lipo3s()), np.eye(3), (-20, 26, 415.5), link='base', mass=185, cat='power', price='$19.99',
                 spec='Modelled typical 105 x 35 x 25 mm hobby pack with lead stubs. The outline is typical, NOT from one product sheet: check your pack.', src='typical hobby 3S 2200 mAh'))
    E.append(mod('Pololu D24V50F5 5 V buck', EM._mesh(EM.pololu_d24v50f5()), R_X, (6.4, 35, 385), link='base', mass=7, cat='power', price='$32.95',
                 spec='Modelled: board 17.8 x 20.3, 8.8 mm tall, two 2.2 mm holes 13.5 x 16.0 apart (Pololu product 2851). Inductor and IC blocks approximate.', src='pololu.com/product/2851'))
    E.append(mod('Waveshare bus adapter A (legs)', EM._mesh(EM.waveshare_adapter_a()), R_XF, (11.7, -31, 424), link='base', mass=8, cat='compute', price='$4.99',
                 spec='Modelled: 42 x 33 mm board, 4 x 2.5 mm holes on 37 x 28 mm (Waveshare). Connector bodies approximate.', src='waveshare.com/product/bus-servo-adapter-a.htm'))
    E.append(mod('Waveshare bus adapter B (arms/neck)', EM._mesh(EM.waveshare_adapter_a()), R_XF, (11.7, -31, 466), link='base', mass=8, cat='compute', price='$4.99',
                 spec='Modelled: 42 x 33 mm board, 4 x 2.5 mm holes on 37 x 28 mm (Waveshare). Connector bodies approximate.', src='waveshare.com/product/bus-servo-adapter-a.htm'))
    E.append(mod('Seeed XIAO ESP32-S3', EM._mesh(EM.xiao_esp32s3()), R_X, (9.55, 25, 431.75), link='base', mass=3, cat='compute', price='$7.90',
                 spec='Modelled: 21 x 17.8 mm board, USB-C overhang to 22.5 mm, 4.5 mm high (Seeed outline). Component blocks approximate.', src='wiki.seeedstudio.com/xiao_esp32s3_getting_started'))
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
    E.append(mod('E-stop (16 mm unibody, NC, latching)', EM._mesh(EM.estop16()), np.eye(3), (ESTOP_X, ESTOP_Y, TORSO_TOP), link='base', mass=23, cat='power', price='$7.50 est.',
                 spec='Modelled from IDEC X6/XA outline: dia 30 mushroom, 20.6 mm above the panel, 19.5 mm behind it, dia 16.2 hole with 1.7 mm anti-rotation key. Internal profile approximate. Price is an estimate for a generic unit, not the IDEC list price.', src='IDEC X6/XA unibody e-stop datasheets', show=True))
    # XT60 panel-mount charge/discharge port in the battery door (adds ~$1.2 vs. a plain pigtail)
    E.append(mod('XT60 panel connector (charge port)', EM._mesh(EM.xt60_panel()), np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float), (TORSO_X[0] - 2.4, 26.0, 482.0), link='base', mass=6, cat='power', price='$1.20',
                 spec='Modelled from Amass XT60 outline: 16.0 x 8.1 mm face, contacts 7.2 mm apart, dia 3.5 mm bullets; 15.8 x 8.2 mm cutout in battery_door. Cost-adding vs a loose pigtail; flagged in BOM.', src='Amass XT60 drawing (hobbydex.com/rc/specs/connector/xt60)', show=True))
    # head: camera + mic
    E.append(mod('Pi Camera Module 3', EM._mesh(EM.picam3()), np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float), (XN + 26.0 - 5.75, 0, 579.6), link='head', mass=4, cat='sensor', price='$25',
                 spec='Modelled from the Raspberry Pi mechanical drawing: 25 x 24 x 11.5 mm, 4 x dia 2.2 holes on 21 x 12.5 mm. Lens barrel diameter and position approximate; lens axis aligned to the head lens window.', src='Raspberry Pi camera module 3 product brief + drawing RP-008153'))
    E.append(box_env('I2S MEMS mic', (12.7, 1.6, 8.0), (XN + 25, 13.6, ZNP + 5), link='head', mass=1, cat='sensor', price='$6.95',
                     spec='Envelope assumed', src='assumed'))
    return E
