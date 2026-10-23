# Open-Source, Free and Low-Cost CAM with 3/4/5-Axis Milling (as of Oct 2026)

Method note: GitHub metadata (license, last push, stars) was pulled live via `gh api` on 2026-10-23. Web searches and fetches were limited, so some vendor prices and features could not be checked this session. Those are marked **[unverified]** and listed under Gaps. URLs marked "(not fetched)" are well-known project pages that I did not re-open this session.

---

## Q1. Catalog: license, price, platform, strategies, rotary/5-axis, sim, posts, activity, integration

### Takeaway
The open-source tools are strong at 2.5D/3-axis (FreeCAD CAM, Kiri:Moto, Fabex, OpenCAMLib). Their multi-axis support is limited to 4th-axis indexing, lathe-style rotary or wrapping, plus experimental scripts. No mature open-source project offers usable simultaneous 5-axis. In the low-cost commercial tier, rotary support is usually "wrapped" (Vectric) or "indexed" (DeskProto, MeshCAM, Kiri:Moto). Of the products found, only SprutCAM X and similar mid-price CAM (~$250/mo or more) offer real simultaneous 5-axis.

### Cited Findings

**Open source: engines and full applications**

- **OpenCAMLib (OCL)**: C++ library with Python/JS bindings for drop-cutter, push-cutter and waterline 3-axis surface toolpaths. License LGPL-2.1. Repo is active: last push 2026-10-17, ~546 stars — [GitHub aewallin/opencamlib](https://github.com/aewallin/opencamlib) (metadata via gh api). It is 3-axis only, with no multi-axis tool-orientation algorithms **[unverified from docs this session; based on the library's known drop-cutter/waterline scope]**.
- **FreeCAD CAM workbench (formerly Path)**: FreeCAD is LGPL-2.1, under very active development (last push 2026-10-23, ~34.7k stars) — [GitHub FreeCAD/FreeCAD](https://github.com/FreeCAD/FreeCAD). FreeCAD 1.1 was released 24 March 2026 — [OSArch](https://osarch.org/2026/03/25/freecad-1-1-released/). The 1.1 CAM changes centre on a new tool library/toolbit editor and selector, with no rotary features mentioned — [Medium summary](https://medium.com/@Re-News/freecad-1-1-arrives-with-major-ux-core-and-engineering-workflow-enhancements-bd5f3bfec0e1); [Fabbaloo](https://www.fabbaloo.com/?p=236104).
  - The 4th-axis support is "experimental" and "early development". It consists of 4th-axis faces/pockets via scripts (PathProfileFaces.py, PathAreaOp.py, PathPocketShape.py) that users copy in by hand. It "do[es] not handle complex/compound rotations", and there is "no GUI integration of 4th-axis rotational settings in the release branch". The page dates from the 0.19 era — [FreeCAD wiki: CAM fourth axis](https://wiki.freecad.org/CAM_fourth_axis); [dev forum: Pocket Shape 4th-axis integration](https://devtalk.freecad.org/t/path-pocket-shape-4th-axis-integration/31154).
  - The 3D Surface and Waterline ops use OpenCAMLib. Adaptive clearing uses libarea/"actp"-derived code. Posts are Python (LinuxCNC, GRBL, Mach3, Fanuc and others). The simulator is basic **[unverified this session; based on known FreeCAD CAM docs: https://wiki.freecad.org/CAM_Workbench (not fetched)]**.
  - Integration: standalone, inside FreeCAD's own CAD. No Fusion integration.
- **Fabex (formerly BlenderCAM)**: GPL-3.0 Blender extension for Windows, macOS and Linux. It was renamed after the Blender Foundation asked external projects to stop using the "Blender" name. Strategies include Profile, Pocket, Drill, Parallel, Cross, Block, Spiral and Medial Axis. Cutters include ballnose, bullnose, V-carve and user-defined. It can work from images or 3D meshes — [README, pppalain/blendercam](https://github.com/pppalain/blendercam/blob/master/README.md).
  - Multi-axis: the README states "4 and 5 axis milling are only manual" — [README](https://github.com/pppalain/blendercam/blob/master/README.md). NLnet describes "3+ axis" machines — [NLnet FabCNC](https://nlnet.nl/project/FabCNC/). This contradicts the README only mildly.
  - Activity: pppalain/blendercam and SpectralVectors/blendercam both pushed 2026-10-16. The original vilemduha/blendercam (~585 stars) last pushed 2026-01-01. All are GPL-3.0 (gh api). Docs: [spectralvectors.github.io/blendercam](https://spectralvectors.github.io/blendercam/index.html).
  - Integration: Blender add-on. Mesh-centric, so no B-rep.
- **Kiri:Moto (Grid.Space)**: browser-based slicer and CAM, part of the grid-apps repo. License MIT, very active (last push 2026-10-22, ~924 stars) — [GitHub GridSpace/grid-apps](https://github.com/GridSpace/grid-apps).
  - 4th axis: an "indexed" stock option enables index and lathe operations. Developers said lathe mode is "meant for 4th axis on a traditional mill". Users report roughing at 0°/180° and then finishing with the lathe op — [Grid.Space forum: CNC lathe settings](https://forum.grid.space/t/cnc-lathe-settings/1370); [Can Moto 4th axis mill](https://forum.grid.space/t/can-moto-4th-axis-mill/1127); [Lathe mode real lathes?](https://forum.grid.space/t/lathe-mode-support-real-lathes/934).
  - Spiral or continuous rotary cutting was earlier reported as unsupported — [forum](https://forum.grid.space/t/can-kirimoto-do-4th-axis-cutting-of-spirals/350).
  - Makera lists Kiri:Moto as a free option with "3 and 4 axis CAM operations include: roughing, pocketing, contouring, tracing" — [Makera software page](https://www.makera.com/pages/software).
  - Integration: standalone web app, with an Onshape app integration **[unverified this session]**.
- **PyCAM**: Python 3-axis CAM. Effectively dormant: SebKuzminsky/pycam last push 2024-01-30, ~403 stars. GitHub reports the license as NOASSERTION; it is historically GPL-3 **[license unverified]** — [GitHub SebKuzminsky/pycam](https://github.com/SebKuzminsky/pycam).
- **CAMotics**: open-source 3-axis G-code simulator and "TPL" toolpath language by Cauldron Development (Buildbotics). Last push 2026-03-23, ~761 stars. GitHub reports the license as NOASSERTION; it is historically GPL-2+ **[unverified]** — [GitHub CauldronDevelopmentLLC/CAMotics](https://github.com/CauldronDevelopmentLLC/CAMotics); [camotics.org](https://camotics.org) (not fetched). It is a simulator, not a full CAM, and has no 5-axis simulation **[unverified]**.
- **jscut**: browser 2D CAM for SVG files (pocket, outline, V-carve). GPL-3.0, dormant (last push 2023-07-26) — [GitHub tbfleming/jscut](https://github.com/tbfleming/jscut).
- **HeeksCNC / libarea**: legacy C++ CAD/CAM and pocketing library, dormant (heekscnc last push 2021-10-14; libarea last push 2020-01-31) — [GitHub Heeks/heekscnc](https://github.com/Heeks/heekscnc), [Heeks/libarea](https://github.com/Heeks/libarea).

**Open source: experimental 5-axis projects (existence verified, maturity low)**

- **OpenMill (charlatanshost/OpenMill)**: Rust "open-source 5-axis CAM". Its README claims 13 strategies: 3+2 indexed, 4+1 indexed, simultaneous 5-axis roughing, surface-normal 5-axis (parallel/scallop/spiral/rotary), swarf, geodesic parallel, 5-axis pencil, 5-axis drilling, plus holder collision checks and LinuxCNC/GRBL posts. Only ~9 stars; last push 2026-07-25; GitHub reports the license as NOASSERTION. The README contains a "(Placeholder for actual screenshot)" line, which suggests an early or possibly AI-generated project — [GitHub charlatanshost/OpenMill](https://github.com/charlatanshost/OpenMill).
- **collet (ericgonzadev/collet)**: "Browser-based CAM: 2.5D to simultaneous 5-axis toolpaths, stock simulation and G-code". 0 stars, pushed 2026-10-14, no license detected — [GitHub](https://github.com/ericgonzadev/collet).
- **FiveAxisSimCore (li-yunwen)**: MIT, a 5-axis CAM *simulation* engine (voxel/SDF). 5 stars, pushed 2026-03-22 — [GitHub](https://github.com/li-yunwen/FiveAxisSimCore).
- **Mas5ACAM**: MIT, "5 Axis CAM System", 0 stars, 2026-08-30 — [GitHub mallemann/Mas5ACAM](https://github.com/mallemann/Mas5ACAM).
- **LuaCAM**: "C library with Lua bindings for generating multi-axis toolpaths", 3 stars, 2024-09 — [GitHub LibreEngineer/LuaCAM](https://github.com/LibreEngineer/LuaCAM).
- **Rab Gordon's CNC Toolkit**: "3, 4 & 5 axis CAM" archive copy, 2017 — [GitHub 3DLIRIOUS/cnctoolkit](https://github.com/3DLIRIOUS/cnctoolkit).
- **ROS-Industrial Noether**: confirmed to exist. It is a surface-raster toolpath planner for robots (painting, sanding), not CNC G-code. License Apache-2.0, last push 2026-01-08, ~180 stars — [GitHub ros-industrial/noether](https://github.com/ros-industrial/noether). Its tool-orientation-on-mesh code could be relevant and the license is permissive.
- **Open5x (Freddie Hong)**: confirmed to exist, but it is **5-axis 3D printing** (conformal slicing on a modified Prusa), not milling. MIT, ~1,198 stars, last push 2024-04-02 — [GitHub FreddieHong19/Open5x](https://github.com/FreddieHong19/Open5x). A derivative is MM-Open5x (MIT, 2026-09) — [GitHub](https://github.com/BigKoala33/MM-Open5x).
- **"OpenMill" name collisions**: PlutoMills/OpenMill is a planetary ball mill and marbocub/Openmill-3D is a DIY CNC machine. Neither is CAM (gh search).

**Commercial or vendor: free and low cost**

- **Onshape CAM Studio** (native cloud CAM by PTC): 2.5–3-axis, with adaptive included in all versions. It is included with Onshape Professional, Enterprise, Education Enterprise and Government, and is *not* available on Free, Standard or Student plans. It has full machine-kinematics simulation on cloud compute, fixture collision objects, and customizable tool libraries, machines and posts. **"CAM Studio Advanced" (4-axis, 3+2, 5-axis) is still "Coming Soon" with a waitlist** — [Onshape CAM Studio page](https://www.onshape.com/en/features/cam-studio).
  - At launch it was available "for no additional cost" to Professional and Enterprise subscribers, 2.5/3-axis only — [All3DP](https://all3dp.com/4/onshapes-new-cloud-native-cam-is-cooler-than-you-may-think/?critical=1).
  - Users complain there is no tier between Free and paid with CAM. A forum post puts Professional at about $2,500/yr (user-reported) — [Onshape forum](https://forum.onshape.com/discussion/26582/improvements-to-onshape-january-31st-2025/p2); [LinuxCNC forum](https://www.forum.linuxcnc.org/post-processors/52199-onshape-cam-post-processor).
  - The official page does not mention ModuleWorks. Separately, Onshape has long hosted third-party CAM apps in its App Store (e.g., HSMWorks-like partners, Kiri:Moto, CAM Works/ModuleWorks-based partners) **[unverified this session]**.
- **Autodesk Fusion (personal use)**: since Oct 2020 the personal license has removed rapid moves (G0 output as G1 at feed), 3+2 tool orientation, multi-axis milling, probing and automatic tool changes — [Onefinity forum repost](https://forum.onefinitycnc.com/t/major-changes-to-fusion-360-for-hobby-use/874); [Langmuir forum](https://forum.langmuirsystems.com/t/october-1-changes-to-fusion-360-and-its-impact-on-firecontrol/10557/1); [Autodesk FAQ (updated Feb 2025)](https://autodesk.com/support/technical/article/Fusion-360-Free-License-Changes).
  - Fusion's post library is free, open, editable JavaScript, and needs a machine configuration (kinematics) to output rotary angles instead of tool vectors — [Autodesk post article](https://autodesk.com/support/technical/article/How-to-make-a-3-axis-post-processor-from-the-HSM-post-processor-library-support-4-or-5-axis-motion); [Autodesk forum](https://forums.autodesk.com/t5/fusion-manufacture/questions-on-5-axial-processing-in-fusion360/m-p/8046806).
- **SprutCAM X**: full simultaneous and indexed 5-axis, turning and mill-turn. The official shop shows simultaneous 5-axis milling "from 250 USD/month" — [SprutCAM shop](https://shop.sprutcam.com/main).
  - Aggregators list EUR 1,000 and a free EDU version — [FitGap](https://us.fitgap.com/products/sprutcam); [TrustRadius](https://tr-web.dev.trustradius.com/compare-products/sprutcamx).
  - Tormach OEM bundles are locked to Tormach posts — [Tormach](https://tormach.com/accessories/software.html).
  - I could not find a current "SprutCAM X Hobby" SKU or price **[gap]**.
- **DeskProto** (Delft Spline Systems): the Multi-Axis edition costs €995 commercial and €248 for the hobby/school license. Its 5-axis is *indexed* only ("model can be machined from almost all sides"), plus 4th-axis rotary. Windows only. Free and Entry editions exist **[unverified]** — [DeskProto Multi-Axis page](https://www.deskproto.com/products/multiax-ed.php); [StepCraft reseller (US$299 hobby, not available)](https://shop.stepcraft.us/shop/32142-deskproto-multi-axis-hobby-license-non-commercial-use-only-132); [SoftwareFinder](https://softwarefinder.com/manufacturing-software/deskproto).
- **MeshCAM** (now owned by Carbide 3D **[ownership unverified this session]**): mesh-based 3-axis CAM with two-sided A/B flips. It can make 4th-axis programs "only for index milling not continuous" — [Carbide 3D community](https://community.carbide3d.com/t/gcode-sender-for-rotary-4th-axis/34346/2).
- **Carbide Create / Carbide Create Pro**: 2.5D, with simple 3D in Pro. **No rotary/4th-axis support**. Carbide Motion is unlikely to drive a 4th axis, and staff said Shapeoko 5 Pro lacks the electronics for it — [Carbide 3D community](https://community.carbide3d.com/t/can-a-4-axis-unit-be-used-with-carbide-create-pro/100349); [4th axis for Shapeoko 5 Pro](https://community.carbide3d.com/t/4th-axis-for-shapeoko-5-pro/68902).
- **Vectric Aspire 12** (and VCarve Pro): Aspire is ~$1,995 perpetual — [OpenBuilds store](https://us.openbuilds.com/aspire); [Onefinity store](https://www.onefinitycnc.com/product-page/vectric-aspire-perpetual-life-time-purchase).
  - Its rotary support is *wrapped*: Y is wrapped onto A. Users report no native 3+1 indexing and work around it with multiple rotated setups — [Avid CNC forum](https://forum.avidcnc.com/t/using-4th-axis-as-indexer/1585); [Snapmaker forum](https://forum.snapmaker.com/t/is-the-rotary-module-worth-the-price/20598).
  - Windows only; plugin-free standalone. VCarve Pro rotary capability is **[unverified]**.
- **Makera CAM**: free for Carvera and Carvera Air owners, public beta on Windows and macOS. It handles 2D, 3D, 4th-axis "rotary relief" strategies and laser — [Makera CAM](https://global.makera.com/pages/makera-cam); [Makera software](https://www.makera.com/pages/software).
  - The 4th-axis module is sold separately. Makera's own page says "For Carvera Air only", which conflicts with a retailer listing — [Makera 4th axis](https://www.makera.com/products/4th-axis); [Core Electronics](https://core-electronics.com.au/makera-carvera-4th-axis-module-expansion.html).
- **Estlcam**: inexpensive Windows CAM and controller. Users report "Estlcam doesn't do 4 axis" (pre-v12); the workaround is unwrapping to treat Y as rotary — [V1E forum](https://forum.v1e.com/t/4th-axis/38834). A historical v11 price was ~$40–60 (user report) — [OpenBuilds](https://builds.openbuilds.com/goto/post?id=64841). Status of 4th axis in v12 **[gap]**.
- **Not verified this session** (training knowledge only; flagged for the writer):
  - **CamBam**: Windows, ~$150, 2.5D plus basic 3D waterline/scanline, Python plugins, 4th-axis via plugins/unwrap.
  - **SheetCAM**: 2D only, plasma/router.
  - **BobCAD-CAM**: commercial, multi-thousand-dollar, with 4/5-axis add-ons.
  - **MillMage** (LightBurn Software): 2D/2.5D router CAM plus sender, entered beta in 2024.
  - **Easel** (Inventables): browser 2D/2.5D, free plus Pro subscription, no rotary.

**5-axis hobby machines and controllers**

- **Pentamachine (Pocket NC successor)**: no source was found this session for its recommended CAM workflow. From training knowledge **[unverified]**, Penta recommends Autodesk Fusion with a Penta-supplied post and machine config, and the machine runs a LinuxCNC-based controller ("Kinetic Control").
- **LinuxCNC**: provides 5-axis kinematics modules such as xyzac-trt-kins and xyzbc-trt-kins, plus generic switchkins with TCP **[unverified this session; docs at https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html (not fetched)]**.
- A LinuxCNC forum moderator (2024) said they were "pretty sure there is no free 5 axis software" — [LinuxCNC forum: Free CAM software 5 axes](https://forum.linuxcnc.org/38-general-linuxcnc-questions/52452-free-cam-software-5-axes).

### Inferences
- The open-source multi-axis gap is real and confirmed by practitioners: the LinuxCNC forum view above, Fabex's "manual only" note, and FreeCAD's experimental scripts. New 2026 GitHub projects (OpenMill, collet, Mas5ACAM) show interest but are not proven.
- Low-cost commercial tools cluster at "wrapped rotary" (Vectric, Makera relief) or "indexed" (DeskProto, MeshCAM, Kiri:Moto). Simultaneous 4- or 5-axis jumps to SprutCAM-class pricing (~$3k/yr) or Fusion paid plus the Machining Extension.
- Onshape is the most direct threat. CAM Studio already has kinematic machine simulation and has announced 3+2, 4- and 5-axis in its "Advanced" tier, but that tier is not yet shipped as of Oct 2026.

### Gaps
- Exact current prices for SprutCAM X Hobby, Estlcam 12, CamBam, MeshCAM, BobCAD, MillMage and Easel Pro were not verified this session.
- Whether CAM Studio Advanced will be ModuleWorks-based, and what it will cost, is not stated on Onshape's page.
- Pentamachine's official CAM guidance page could not be fetched (404 at the guessed URL).
- No working check was done on OpenMill or collet beyond their READMEs.

---

## Q2. Which free or open tools have genuine simultaneous 5-axis, and which only 3+2 or 4th-axis wrap?

### Takeaway
No established free or open-source CAM ships production simultaneous 5-axis milling. Fabex (manual only), FreeCAD (experimental 4th axis), Kiri:Moto (indexed plus lathe) and DeskProto (indexed 5-axis) stop short of it. Only unproven 2026 hobby repos claim it.

### Cited Findings
- Fabex: "4 and 5 axis milling are only manual" — [README](https://github.com/pppalain/blendercam/blob/master/README.md).
- FreeCAD CAM: experimental 4th axis with no compound rotations and no GUI in the release branch — [wiki](https://wiki.freecad.org/CAM_fourth_axis).
- Kiri:Moto: indexed stock plus lathe op; spiral or continuous rotary was reported unsupported — [forum](https://forum.grid.space/t/cnc-lathe-settings/1370); [forum](https://forum.grid.space/t/can-kirimoto-do-4th-axis-cutting-of-spirals/350).
- DeskProto Multi-Axis: "Five-axis machining (indexed)" — [DeskProto](https://www.deskproto.com/products/multiax-ed.php).
- MeshCAM: index-only 4th axis — [Carbide 3D forum](https://community.carbide3d.com/t/gcode-sender-for-rotary-4th-axis/34346/2).
- Vectric Aspire: wrapped rotary — [Avid forum](https://forum.avidcnc.com/t/using-4th-axis-as-indexer/1585).
- Makera CAM: 4th-axis "rotary relief" — [Makera](https://global.makera.com/pages/makera-cam).
- OpenMill README *claims* simultaneous 5-axis roughing, surface-normal and swarf, but the project is immature — [GitHub](https://github.com/charlatanshost/OpenMill). collet makes a similar claim — [GitHub](https://github.com/ericgonzadev/collet).
- Simultaneous 5-axis in paid tools: SprutCAM X (from $250/mo) — [shop](https://shop.sprutcam.com/main). Onshape CAM Studio Advanced is announced but "Coming Soon" — [Onshape](https://www.onshape.com/en/features/cam-studio).

### Inferences
- A Fusion add-in offering affordable 4-axis (indexed plus continuous wrap plus true 4-axis) and 3+2 would beat every free option. Simultaneous 5-axis would be essentially unique below ~$3k/yr. One caveat: Fusion's own paid Machining Extension sits in this space.

### Gaps
- The quality and accuracy of the OpenMill and collet 5-axis claims are untested.

---

## Q3. Code licenses and reuse in a closed or open Fusion add-in

### Takeaway
Permissive and reusable in a closed add-in: Kiri:Moto/grid-apps (MIT), Noether (Apache-2.0), FiveAxisSimCore (MIT), Mas5ACAM (MIT). Reusable with conditions: OpenCAMLib and FreeCAD are LGPL-2.1, so they are usable from closed code if dynamically linked and replaceable. Copyleft that forces an open-source add-in: Fabex (GPL-3), jscut (GPL-3), PyCAM and CAMotics (GPL, per repo history). Unclear and not reusable until clarified: OpenMill and collet.

### Cited Findings
| Project | License (GitHub SPDX) | Last push | Source |
|---|---|---|---|
| OpenCAMLib | LGPL-2.1 | 2026-10-17 | [repo](https://github.com/aewallin/opencamlib) |
| FreeCAD (CAM wb) | LGPL-2.1 | 2026-10-23 | [repo](https://github.com/FreeCAD/FreeCAD) |
| Kiri:Moto (grid-apps) | MIT | 2026-10-22 | [repo](https://github.com/GridSpace/grid-apps) |
| Fabex/BlenderCAM | GPL-3.0 | 2026-10-16 | [repo](https://github.com/pppalain/blendercam) |
| jscut | GPL-3.0 | 2023-07-26 | [repo](https://github.com/tbfleming/jscut) |
| PyCAM | NOASSERTION (historically GPL-3) | 2024-01-30 | [repo](https://github.com/SebKuzminsky/pycam) |
| CAMotics | NOASSERTION (historically GPL-2+) | 2026-03-23 | [repo](https://github.com/CauldronDevelopmentLLC/CAMotics) |
| Noether | Apache-2.0 | 2026-01-08 | [repo](https://github.com/ros-industrial/noether) |
| Open5x (printing) | MIT | 2024-04-02 | [repo](https://github.com/FreddieHong19/Open5x) |
| OpenMill | NOASSERTION | 2026-07-25 | [repo](https://github.com/charlatanshost/OpenMill) |
| FiveAxisSimCore | MIT | 2026-03-22 | [repo](https://github.com/li-yunwen/FiveAxisSimCore) |
| collet | none detected | 2026-10-14 | [repo](https://github.com/ericgonzadev/collet) |
| HeeksCNC | NOASSERTION | 2021-10-14 | [repo](https://github.com/Heeks/heekscnc) |

- Fusion posts are open, editable JavaScript, and Fusion's post/kinematics machine configuration can be leveraged rather than reimplemented — [Autodesk](https://autodesk.com/support/technical/article/How-to-make-a-3-axis-post-processor-from-the-HSM-post-processor-library-support-4-or-5-axis-motion).

### Inferences
- The most useful permissively licensed CAM code is Kiri:Moto's JS CAM (MIT). Fusion add-ins can host HTML/JS palettes and Python, so JS is attractive. OpenCAMLib (LGPL) has Python bindings and suits Fusion's Python API if shipped as a separate dynamic module.
- FreeCAD CAM's Python ops are LGPL, but they depend heavily on FreeCAD/OCC objects. They are better mined for algorithms than for copied code.
- GPL code (Fabex, PyCAM, CAMotics) can only be reused in a GPL add-in. Their *algorithms* can be studied but not copied into closed code.

### Gaps
- License text for PyCAM, CAMotics and OpenMill was not opened this session (GitHub says NOASSERTION). Check COPYING files before any reuse.

---

## Q4. Fusion integration, and community pain points for prosumer multi-axis

### Takeaway
None of the open-source or low-cost CAMs found here integrates with Fusion. They are standalone, Blender-, FreeCAD- or browser-based, or Onshape-native. The pain points are:
- Fusion personal use blocks multi-axis and 3+2 output and slows rapids.
- Onshape CAM requires a ~$2.5k/yr plan and has no multi-axis yet.
- Hobby rotary users (Shapeoko, Carvera, Avid, Snapmaker) are stuck with wrapped or indexed-only tools.
- There is a widely stated belief that no free 5-axis CAM exists.

### Cited Findings
- Fusion personal: no multi-axis, no 3+2, no rapids — [Onefinity forum](https://forum.onefinitycnc.com/t/major-changes-to-fusion-360-for-hobby-use/874); [Langmuir forum](https://forum.langmuirsystems.com/t/october-1-changes-to-fusion-360-and-its-impact-on-firecontrol/10557/1).
- Onshape: no tier between Free and the first paid tier that includes CAM; a user notes a Carvera Air ($2,100) costs less than a year of CAM Studio access — [Onshape forum](https://forum.onshape.com/discussion/26582/improvements-to-onshape-january-31st-2025/p2).
- Shapeoko users: no 4th-axis path in Carbide Create or Motion; Shapeoko 5 Pro lacks electronics — [Carbide 3D forum](https://community.carbide3d.com/t/4th-axis-for-shapeoko-5-pro/68902); [Shapeoko 5 thread](https://community.carbide3d.com/t/4-th-axe-on-shapeoko-5/75300).
- Avid and Vectric users want 3+1 indexing that Aspire lacks — [Avid forum](https://forum.avidcnc.com/t/using-4th-axis-as-indexer/1585).
- Kiri:Moto users ask for spiral or continuous rotary — [Grid.Space forum](https://forum.grid.space/t/can-kirimoto-do-4th-axis-cutting-of-spirals/350).
- "No free 5 axis software" — [LinuxCNC forum](https://forum.linuxcnc.org/38-general-linuxcnc-questions/52452-free-cam-software-5-axes); see also [LinuxCNC 4-axis software thread](https://forum.linuxcnc.org/38-general-linuxcnc-questions/36472-4-axis-softwar).
- Getting rotary angles (rather than vectors) from Fusion requires a correct machine configuration/kinematics in the post. This is a common stumbling block — [Autodesk forum](https://forums.autodesk.com/t5/fusion-manufacture/questions-on-5-axial-processing-in-fusion360/m-p/8046806); [Autodesk article](https://autodesk.com/support/technical/article/How-to-make-a-3-axis-post-processor-from-the-HSM-post-processor-library-support-4-or-5-axis-motion).
- A Fusion default post could not handle simultaneous 5-axis on a DMU50 until it was modified — [UiTM thesis](https://ir.uitm.edu.my/id/eprint/37755).

### Inferences
- A Fusion add-in has an open lane. Its target buyers are paid-Fusion users who don't want the Machining Extension cost, plus prosumers with rotary or 5-axis desktop machines (Carvera 4th axis, Pentamachine, Onefinity/Avid rotaries). Its value would be easy GRBL/LinuxCNC rotary and 3+2 posts, continuous 4-axis (beyond wrap), and kinematic simulation.
- Whether an add-in can bypass Fusion personal-use limits is a policy question, not just a technical one. If the add-in generates its own G-code it may sidestep the limits, but Autodesk's terms need review.

### Gaps
- No direct Reddit (r/hobbycnc, r/CNC) or Pentamachine forum sentiment was collected this session.
- Whether Fusion add-in APIs allow third-party toolpath generation and posting inside personal-use licenses is unverified.
- Fusion Machining Extension pricing was not collected (out of scope / covered elsewhere).
