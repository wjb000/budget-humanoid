# DFM: from FDM prototype to injection molding

Status: **design notes, not a mold design.** Nothing here was run through mold-flow or quoted by a toolmaker. The current CAD is made for FDM printing: vertical walls have **no draft**, bosses are solid, and the servo sleeves are blocky. This file says what to change for each part when volume justifies tooling. Cost figures are rough rules of thumb, so get a real quote.

## General rules applied to every part

| Topic | FDM now | Injection molding change |
|---|---|---|
| Wall | 2.4 mm shell (`WALL`), 4 mm plates | 2.0 to 2.5 mm nominal, ribs at 50 to 60 % of the wall to avoid sink. Plates at 4 mm become cored or ribbed. |
| Draft | 0 degrees | 1 to 2 degrees on every wall in the pull direction (3 degrees on textured faces). |
| Fillets | 0.3 x thickness on outside edges | Keep. Add inside radius of at least 0.5 x wall at every inside corner. |
| Fasteners | M2 heat-set inserts pressed in after printing, M2 self-tapping screws into plastic, M3 horn screws | Insert-mold or ultrasonically press the M2 inserts. Self-tapping bosses get the screw maker's boss spec (OD about 2.2 x screw, ribbed). |
| Covers | M2 screws into inserts | Replace 2 of 4 screws with molded snap hooks (0.8 to 1.0 mm engagement) once the cover fit is proven. Hooks need a side core or a lifter. |
| Material | PLA or PETG | PC/ABS for shells and covers (impact, paintable), PA6-GF30 for loaded limb parts (thigh, shin, ankle, foot). Check creep at the servo screw bosses. |
| Parting line | n/a | Along the widest silhouette of each part, so the cover/torso parting line follows the front/back split already used in the CAD. |
| Left and right | Two STLs per limb part | Mirror-image parts need two cavities or two molds. One family mold with two inserts is the usual saving. |

## Per part

| Part | Pull direction | Undercuts to resolve | Notes for molding |
|---|---|---|---|
| `torso` | Along x (front and back are open), core from each side | Side windows (servo access), battery rib slots, strap slots, e-stop key notch: all need side actions or through-core slides. The four insert bosses on the front and the four on the back are in the pull direction (no slide). | Biggest part (about 137 g printed). Split into left and right halves for a lower press tonnage if needed. Floor and top plate are 4 mm, so core them. Rounded vertical corners (R5 outside, R2.6 inside) already keep a uniform 2.4 mm wall. |
| `torso_front_cover` | Along x | Speaker slots (through, no undercut). XIAO cradle rails and lips are on the core side and face along the pull. | Solid 2.4 mm plug inside a 2.4 mm plate, so the plug would sink. Turn it into a 1.2 mm skin with a rib grid. Add a molded gasket lip if dust matters. |
| `battery_door` | Along x | XT60 cutout is through. | Add a molded finger notch and one snap hook on the lower edge. |
| `head` | Along z (open top) | Camera window (through the front wall), mic pocket. | Two M2 pilot holes for the lid go into a thick front wall, which is a sink risk. Core them out from behind. |
| `head_lid` | Along z | None. | 1 to 2.4 mm. Can become a clip-on lid and drop both screws. |
| `neck_bracket` | Along the servo axis | Sleeve pocket is open on the horn side, good. | Small, a good first part to mold. |
| `pelvis` | Along z | Two servo sleeves with rear recess. | Left and right halves are mirror parts, so molding one half twice is cheaper. |
| `*_hip_yaw_link`, `*_hip_roll_link` | Along the servo axis | Pocket open on the horn side. Cable notch on the side is a small slide, or move it to the parting plane. | Good candidates for PA6-GF. Bosses at the horn screws need ribs. |
| `*_thigh`, `*_shin` | Along the servo axis | Cable-tie slots (3.2 x 1.6 mm) are through-holes in the pull direction. Windows are exposed servo faces. | The long web needs ribbing. Draft matters on the 120 mm length. |
| `*_ankle_link`, `*_foot` | Along z for the foot | Foot sole pocket, ankle sleeve. | Foot gets a TPU or overmolded rubber sole. Molding the sole in two shots adds cost, so keep the glued TPU pad in the first volume run. |
| `*_shoulder_link`, `*_upper_arm`, `*_forearm`, `*_finger` | Along the servo axis (finger: along its pivot axis) | Servo sleeves and cable notches as above. | Forearm is the tallest part (141 mm), so check ejection. Finger can be glass-filled nylon. |

## Not covered yet

* **Limb shells.** Only the torso, head, and the three covers are enclosed today. Arms and legs still show the servos through windows. A production design would add two clamshell shells per limb segment (about 12 more parts), and each shell is molded in PC/ABS. This is the phase 3 item.
* **Bearings on the side opposite the horn.** The STS3215 rear stub is about 2 mm tall with a Ø2.4 hole. A real bearing support needs a fork bracket and a check on a physical servo. See README known gaps.
* **Tolerances.** FDM clearance (`CLR`) is tuned for printing. Molded parts hold tighter tolerances, so the sleeve pockets can be tightened after measuring the first molded sample.
* **Tooling cost.** As a rule of thumb, prototype aluminium tools are in the low thousands of dollars per cavity, and production steel tools are several times that. At low volume (under about 500 robots) printed parts are cheaper. Do not mold before the fit coupons have been tested on a real servo.
