#!/usr/bin/env python3
"""Regenerates the per-part table in PRINT_GUIDE.md from print_report.json (run after export.py)."""
import json, os
HERE = os.path.dirname(os.path.abspath(__file__))
R = json.load(open(os.path.join(HERE, 'print_report.json')))
UPTXT = {'+z': 'as-is (STL is already in print orientation)'}
rows = ['| Part | Qty | Servos it carries | Print size (mm) | Overhang | Est. mass* | Supports |', '|---|---|---|---|---|---|---|']
for k, v in R.items():
    e = v['print_extents_mm']
    sup = 'none needed' if v['overhang_pct'] < 3 else ('tree/touching-bed supports, check preview' if v['overhang_pct'] < 5 else 'supports on cable/screw pockets (see preview)')
    rows.append(f"| `{k}` | 1 | {', '.join(v['servos']) or '-'} | {e[0]:.0f} x {e[1]:.0f} x {e[2]:.0f} | {v['overhang_pct']:.1f}% | {v['mass_g_est']:.0f} g | {sup} |")
tot = sum(v['mass_g_est'] for v in R.values())
tbl = '\n'.join(rows)
out = open(os.path.join(HERE, 'PRINT_GUIDE.template.md')).read().replace('{{TABLE}}', tbl).replace('{{TOTAL}}', f'{tot:.0f}').replace('{{N}}', str(len(R)))
open(os.path.join(HERE, 'PRINT_GUIDE.md'), 'w').write(out)
print('wrote PRINT_GUIDE.md', len(R), 'parts', f'{tot:.0f} g')
