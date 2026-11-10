"""Fast viewer preview: build parts only (no STEP/STL/print report/electronics) and write site/assets/robot.gltf + robot_info.json"""
import json, os
from export import *
from sim_export import write_gltf
P, SV, J, solids, meshes = build_parts()
links = link_table(P, J)
R = dict(J=J, E=[], links=links, P=P, meshes=meshes, solids=solids, sm=servo_mesh())
info = write_gltf(R, os.path.join(ROOT, 'site', 'assets', 'robot.gltf'))
json.dump(info, open(os.path.join(ROOT, 'site', 'assets', 'robot_info.json'), 'w'), indent=0)
print('preview written')
