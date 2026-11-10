# Budget-22: dirt-cheap 22-DOF humanoid

Open design study for a ~0.5 m, ~2.2 kg, 22-DOF biped on Feetech ST3215 servos with a Raspberry Pi 5 + XIAO ESP32-S3.

**Live site:** https://wjb000.github.io/budget-humanoid/ (needs Pages source set to *GitHub Actions*, see below)

Site contains: component map, interactive 3D viewer, DOF/servo-ID table, electrical schematic (Mermaid), full BOM (~$791 recommended tier, price basis labelled per line), reference designs and CAD status.

## Status
- Phase 1 (this commit): site, BOM, schematic, envelope 3D viewer.
- Phase 2 (next): real vendor CAD (licenses credited), parametric CadQuery frame with STEP + STL per part, FDM print guide, URDF + MuJoCo MJCF with mass/inertia and STS3215 actuator parameters, Isaac Lab notes.

The 3D viewer is **not** vendor CAD: it uses datasheet-size envelope boxes (ST3215 45.2 x 24.7 x 35 mm) at approximate positions.

## Enabling GitHub Pages
Repo **Settings > Pages > Build and deployment > Source: GitHub Actions**, then re-run the "Deploy site to GitHub Pages" workflow (Actions tab) or push any commit.

## Layout
- `site/` static site (no build step)
- `tools_gen_data.py` regenerates `site/assets/data.js` (BOM, component pins, DOF table)
- `.github/workflows/pages.yml` deploys `site/`

Design study, no warranty. Verify fit, torque and power before building.
