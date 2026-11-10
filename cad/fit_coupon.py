#!/usr/bin/env python3
"""Print-fit test coupons for the STS3215 mounting scheme (small, ~15 to 25 g total). PRINT THESE FIRST.
  fit_coupon_sleeve : servo sleeve (pocket, rear recess, 4 case-screw holes, cable notch) - test the servo drops in and the M2 screws line up
  fit_coupon_plate  : horn plate (4 x M3 horn pattern, counterbored, center relief) - test the plate sits on the horn
Writes cad/stl/fit_coupon_*.stl and cad/step/fit_coupon_*.step.   python cad/fit_coupon.py"""
import os, cadquery as cq
from lib import *
HERE = os.path.dirname(os.path.abspath(__file__))

def build():
    sv = Servo(0, 'coupon', (0, 0, 0), (0, 0, 1), (1, 0, 0))
    sl = Part('fit_coupon_sleeve'); sl.sleeve(sv)
    pl = Part('fit_coupon_plate'); pl.plate(sv, -15, 15, -15, 15)
    return {'fit_coupon_sleeve': sl.build(), 'fit_coupon_plate': pl.build()}

if __name__ == '__main__':
    for d in ('stl', 'step'):
        os.makedirs(os.path.join(HERE, d), exist_ok=True)
    for n, s in build().items():
        # sleeve prints with the horn-side opening up (+z), plate prints flat on its outer face (-z up for plate = horn side down)
        cq.exporters.export(cq.Workplane('XY').add(s), os.path.join(HERE, 'step', n + '.step'))
        import numpy as np, trimesh
        v, f = s.tessellate(0.05, 0.25)
        m = trimesh.Trimesh(np.array([[p.x, p.y, p.z] for p in v]), np.array(f), process=True); m.merge_vertices()
        if n == 'fit_coupon_sleeve':   # print opening up: shaft axis +z is already up
            m.apply_translation((0, 0, -m.bounds[0][2]))
        else:                          # plate prints flat on its servo-side face
            m.apply_translation((0, 0, -m.bounds[0][2]))
        m.export(os.path.join(HERE, 'stl', n + '.stl'), file_type='stl_ascii')
        bb = s.BoundingBox(); print(n, f'{bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, valid={s.isValid()}, volume {s.Volume()/1000:.1f} cm3')
