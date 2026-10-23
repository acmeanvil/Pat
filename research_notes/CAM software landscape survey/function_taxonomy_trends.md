# Functional Taxonomy of 3/4/5-Axis Milling CAM + Market Trends (as of Oct 2026)

> Scope: vendor-neutral map of what a complete multi-axis milling CAM must do, plus 2023–2026 trends and gaps, for scoping a new Autodesk Fusion CAM plugin.
> Method note: ~20 searches/fetches. Two Autodesk blog pages returned HTTP 403 on fetch, so Autodesk details come from search snippets. Much of the source material is vendor marketing; performance numbers are vendor claims unless stated otherwise. Where a taxonomy item is standard CAM practice but I did not find a source in this session, it is listed under **Inferences** and marked "(domain knowledge, uncited)" so the writer can tell which claims are sourced.

---

## 1. Workflow functions (setup → post → shop floor)

### Takeaway
A complete CAM has a fixed pipeline that does not depend on the vendor: **setup/WCS → stock → fixtures → tools/holders → feeds/speeds → (feature recognition/templates) → toolpath ops → linking → simulation/verification → post → setup sheet/DNC → probing**. For a Fusion plugin, Fusion already provides most of this pipeline (setups, WCS, stock, tool library, simulation, the JS post engine, setup sheets). A plugin should plug into that pipeline and focus on toolpath generation.

### Cited Findings
- Fusion WCS and setup must match machine kinematics. For example, Pentamachine (Pocket NC V2) tutorials require Y pointing up from the B table and G54 via "WCS offset 1". The V2 workflow drops workholding from the CAM model and locates stock by on-machine touch-off — [Pentamachine WCS tutorial](https://pentamachine.atlassian.net/wiki/spaces/PNFUR/pages/1732739077/V2+Series+Work+Coordinate+System+Tutorial+Kinetic+Control); [V2 First Part Tutorial](https://pentamachine.atlassian.net/wiki/spaces/PNFUR/pages/1727037445)
- Posting goes through Fusion's post dialog. Machine OEMs ship posts for several CAMs (Pocket NC's post list is at pocketnc.com → Help → Post Processor) — [V2-50 Quick Start Guide PDF](https://pentamachine.atlassian.net/wiki/download/attachments/610598969/V2-50+Quick+Start+Guide.pdf?version=5)
- Post quality is a recurring failure point. A generic post that "almost fits" the controller forces hand-editing of G-code, causes formatting problems and leaves out canned cycles. A badly configured 5-axis post can introduce axis-limit, retract-position and collision-check errors — [MecSoft blog](https://mecsoft.com/blog/why-many-shops-regret-their-cam-software-purchase); [industrialmonitordirect hyperMILL article](https://industrialmonitordirect.com/de/blogs/knowledgebase/5-axis-programming-time-estimation-hypermill-best-practices) (low-authority source)
- The recommended 5-axis verification chain is: collision detection and machine simulation in CAM before posting, then verifying the G-code in the controller simulator — [industrialmonitordirect](https://industrialmonitordirect.com/de/blogs/knowledgebase/5-axis-programming-time-estimation-hypermill-best-practices)
- Simulation performance is now a selling point. In a vendor test, Mastercam 2026.R2's GPU-accelerated simulation ran a ~90-minute CPU simulation in about 22 minutes. The release also added support for the Blum laser tool setter (tool measurement/probing data handoff) — [Mastercam 2026.R2 press release](https://www.bolsamania.com/nota-de-prensa/mercados/mastercam-2026r2-delivers-gpu-accelerated-simulation-and-ai-powered-assistance--21635723.html); [Mastercam 2026 R2 page](https://www.mastercam.com/solutions/products/mastercam-2026-r2/)
- Fusion for Manufacturing already ships automated whole-part strategies, hole recognition, deburring automation and simultaneous 4/5-axis — [MMS: Autodesk Fusion for Manufacturing](https://www.mmsonline.com/products/autodesk-fusion-for-manufacturing-connects-cnc-design-and-machining-workflows)
- Fusion's Deburr operation includes automatic edge detection (external sharp edges with a minimum edge-angle filter) and supports ball, lollipop, flat, chamfer, tapered and spot-drill tools in 3-, 4- or 5-axis — [Autodesk Deburr blog (Apr 2025)](https://www.autodesk.com/products/fusion-360/blog/deburr-autodesk-fusion-manufacturing/)

### Inferences
**Feature checklist, Level 1: workflow functions** (definitions are domain knowledge and uncited, except where linked above)
1. **Setup & WCS**: part/stock/fixture model, work offsets (G54–G59.x), multiple setups, tool-orientation frames for 3+2 (G68.2 / CYCLE800 / PLANE SPATIAL), datum transfer between setups.
2. **Stock definition**: box/cylinder/offset/solid/mesh stock, and stock carried forward from earlier operations as in-process stock (IPW).
3. **Fixtures/workholding**: fixture solids that take part in collision checking. Includes vise/dovetail/zero-point libraries, which matter on small 5-axis machines.
4. **Tool & holder libraries**: full cutter geometry, including barrel/lens/oval/taper, lollipop, chamfer and taper ball. Also holder and extension geometry for gouge and collision checks, gauge length, and cloud-shared libraries.
5. **Feeds & speeds**: per-tool/material presets, chip-thinning, engagement-based feed control (feed optimization), and links to tooling vendors' data.
6. **Templates & automation**: operation templates, scripted/API generation, rule-based "knowledge" machining.
7. **Feature recognition (FBM)**: holes, pockets, bosses, chamfers, edges for deburr, and accessibility/orientation analysis for 3+2.
8. **Toolpath operations**: the strategy families in sections 2–4 below.
9. **Linking/leads/transitions**: lead-in/out (arc, ramp, helix), retract policies (clearance plane, sphere/cylinder for 5-axis, "minimum retract"), stay-down links, safe tool-axis reorientation between passes, and ordering/sequence optimization.
10. **Simulation & verification**: material removal, gouge/excess check against the design model, holder and fixture collision, full machine-kinematic simulation, G-code-driven simulation (as opposed to CL-data simulation), and stock comparison.
11. **Post-processing**: kinematic solve (IK), RTCP/TCPC vs. non-RTCP output, rotary limits and wind-up/unwind, controller dialects. Fusion uses JavaScript posts.
12. **Documentation**: setup sheets, tool lists, cycle-time estimates.
13. **Data handoff/DNC**: send to controller or network, MTConnect/OPC UA feedback, tool-presetter export.
14. **Probing**: on-machine part setup (WCS probing), in-process inspection, and tool setting.

**Plugin implication:** Fusion's API exposes setups, tools, operations and posts. A plugin that writes geometry back as Fusion operations, or that emits CL data the Fusion post engine can consume, gets simulation, posting and setup sheets "for free". This is an inference; the exact current API limits for custom toolpath generation in Fusion were not verified in this session.

### Gaps
- I did not verify the current Fusion API's support for injecting third-party toolpath geometry. Other researchers or the API docs should confirm this. It is critical for plugin architecture.
- I found no neutral source that enumerates the full workflow; the list above is synthesized.

---

## 2. 3-axis strategy families

### Takeaway
The 3-axis toolset has matured and become a commodity, with ModuleWorks as the common kernel behind many products. Differentiation now comes from **roughing physics/automation** (engagement-controlled, thin-wall-aware, auto feeds) and finishing quality. Fusion already covers nearly every 3-axis family, so a new plugin gains little by re-implementing them.

### Cited Findings
- Fusion "Steep and Shallow" is a 4-in-1 strategy that gives access to parallel, scallop and contour. It has separate stepover, stepdown, overlap, smoothing and wall-clearance controls, and it works for 3-axis and multi-axis with automatic collision avoidance — [Autodesk Fusion blog search snippet](https://www.autodesk.com/products/fusion-360/blog/fusion-360-machining-extension-beyond-5-axis-machining/)
- In May 2022, Fusion added multi-axis options to parallel, scallop, pencil, spiral, radial, morphed spiral and project. These options supported only ball and lollipop tools, and lead/lean was not included at that time — [Autodesk forum](https://forums.autodesk.com/t5/fusion-manufacture-forum/open-looking-for-feedback-on-the-multi-axis-options-which-have/td-p/11189111)
- ModuleWorks 2026.08 added: VoluMill inside SmartRoughing with automatic optimization of depth, width, speed and feed; thin-wall-aware Adaptive Roughing; consistent zigzag in multi-axis area roughing; SuperFinish on NURBS (combining mesh robustness with NURBS accuracy); a "Physics Report" for SmartRoughing; and FeedControl support for tapered and convex-tip end mills — [ModuleWorks 2026.08 highlights](https://www.moduleworks.com/software-components/whats-new/software-release-2026-08/); [ModuleWorks 2026.08 announcement](https://www.moduleworks.com/moduleworks-2026-08-software-release-now-available/)
- ModuleWorks 2026.04 added chip-load-aware VoluMill roughing, adaptive stock detection, and zigzag on thin walls with one-way elsewhere (vendor claim: 10–30% cycle-time reduction on thin-wall parts) — [ModuleWorks 2026.04](https://www.moduleworks.com/software-components/whats-new/software-release-2026-04/)
- Mastercam 2026.R2 added "optimized level-based ordering" and smarter clearance-plane calculation — [Mastercam 2026.R2 release](https://www.newequipment.com/product-directory/software-computers/software/cad-cam/product/55360437/cnc-software-llc-ai-enhanced-cam-software-speeds-simulation-and-programming)

### Inferences
**Feature checklist, Level 2a: 3-axis strategies** (definitions are domain knowledge and uncited; "known for" examples are general industry reputation)
- **Roughing**
  - *Adaptive/trochoidal/HSM roughing*: constant tool engagement and full-flute-depth cuts. Known for: Fusion Adaptive, Mastercam Dynamic, VoluMill (ModuleWorks/Siemens), hyperMILL, PowerMill Vortex, Esprit ProfitMilling, SolidCAM iMachining.
  - *Rest roughing*: uses in-process stock or a previous tool's remaining material.
  - *Plunge roughing*: for long-reach or deep cavities.
  - *Pocket/offset/raster clearing*: the older roughing methods.
- **Semi-finish/finish**
  - *Z-level/waterline/contour*: for steep walls.
  - *Parallel/raster*: for shallow areas.
  - *Scallop/constant-stepover (3D offset)*: constant cusp across the surface.
  - *Pencil/corner*: follows the internal-corner bitangency.
  - *Rest finishing*: corner re-machining against a reference tool.
  - *Spiral/radial*: for round features.
  - *Flowline/morph/morphed spiral*: follows surface UV or blends between two curves.
  - *Steep-and-shallow hybrid*: combines Z-level and scallop/parallel by slope.
  - *Project/curve projection*: engraving.
  - *Blend/morph between curves.*
  - *Flat-area finishing*: horizontal faces.
- **Quality and kinematics features**: arc fitting, point reduction and smoothing, corner rounding, and climb/conventional control. All of these matter for 5-axis output too.
- **Plugin opportunity**: the gap in 3-axis is automation and physics-aware feed optimization, not new strategy families.

### Gaps
- I found no independent benchmark comparing the quality of 3-axis strategies across vendors.

---

## 3. 4-axis strategies

### Takeaway
4-axis has five sub-families: **indexed (3+1)**, **rotary wrap**, **rotary roughing/pocket**, **rotary finishing/contour**, and **4-axis simultaneous** (including 4-axis swarf). Fusion's rotary pocket and contour come from ModuleWorks via the extension. Vendors were still improving 4-axis swarf in 2026.

### Cited Findings
- Fusion's Rotary Pocket, Rotary Contour and Deburr arrived as Extension Preview features through the ModuleWorks partnership (Dec 2022 post) — [Autodesk: what strategies are coming next](https://www.autodesk.com/products/fusion-360/blog/fusion-360-machining-extension-future-moduleworks-machining-strategies/)
- Fusion Deburr can run 4-axis for rotary-style parts that need an A or B axis — [Autodesk deburring blog](https://www.autodesk.com/products/fusion-360/blog/deburring-autodesk-fusion-for-manufacturing/)
- Mastercam 2026.R2 lists "improved 4-axis swarf milling" — [Mastercam 2026 R2](https://www.mastercam.com/solutions/products/mastercam-2026-r2/)

### Inferences
**Feature checklist, Level 2b: 4-axis** (domain knowledge, uncited)
- *Indexed 4-axis (3+1)*: rotate to an angle, then run 3-axis ops. Needs per-angle WCS/tool-plane handling and safe index moves.
- *Rotary wrap*: maps 2D geometry onto a cylinder (engraving, cams, wrapped slots). Uses cylindrical mapping (unwrap, then compute, then rewrap).
- *Rotary roughing*: radial slicing or "Z-level around axis" stock removal.
- *Rotary finishing*: helical/spiral around the axis, or axial along the axis. Used for shafts, propellers and helical features.
- *4-axis simultaneous contour/swarf*: tool axis constrained to the plane perpendicular to the rotary axis.
- *Machine requirements*: rotary axis on X or Y (A vs B), rotary radius/feed (inverse-time G93 vs degrees-per-minute), unwinding, and tailstock collision.
- The prosumer 4th-axis market (rotary add-ons on hobby/benchtop mills) is likely a meaningful underserved slice for wrap and indexed workflows. This is an inference only.

### Gaps
- I found no market data on how many Fusion users run 4th axes. The Fusion forums would be the place to gauge pain around wrap/rotary.

---

## 4. 5-axis strategies, tool-axis control, collision avoidance

### Takeaway
5-axis splits into: **3+2 positional** (most common, now with automatic orientation); **auto-convert 3→5 with tilt**; **swarf**; **multi-axis contour/flowline/morph**; **multi-axis roughing**; **curve/trim**; **application packs** (port, impeller/blisk, blade, tube); **deburr/chamfer**; and **barrel-cutter finishing**. All of these depend on a shared core of **tool-axis control + collision avoidance + smoothing + singularity and limit handling**. Automating orientation choice (auto 3+2, auto tilt) is the active development frontier in 2025–26.

### Cited Findings
- **3+2 automation**: CloudNC CAM Assist added AI-generated strategies for 3+2 in March 2024. CloudNC estimates it then covered about two-thirds of the CNC machining market — [Engineering.com](https://www.engineering.com/?p=100189); [CTE](https://ctemag.com/?p=101213)
- **3+2 automation**: ModuleWorks 2026.08 "Feature-Based Machining Direction Optimization in Auto 3+2 Roughing" picks tool orientations automatically from part geometry — [ModuleWorks 2026.08](https://www.moduleworks.com/software-components/whats-new/software-release-2026-08/)
- **Automatic tilting**: Mastercam 2026.R2 added automatic tilting for bullnose and end mills. One source says ball nose instead, so the tool types are inconsistent across sources — [Mastercam 2026.R2 press](https://www.bolsamania.com/nota-de-prensa/mercados/mastercam-2026r2-delivers-gpu-accelerated-simulation-and-ai-powered-assistance--21635723.html)
- **Fusion 5-axis scope (July 2025)**: positional and simultaneous 5-axis, multi-axis contour, swarf, multi-axis flow, steep and shallow, and collision avoidance. Autodesk recommends Morph/Flow/Blend for individual high-finish features and Swarf for straight-walled pockets — [Understanding 5-axis in Fusion](https://www.autodesk.com/products/fusion-360/blog/understanding-5-axis-machining-autodesk-fusion/); [Fusion strategy selection](https://autodesk.com/products/fusion-360/blog/5-axis-machining-fusion-360-part-2-selecting-strategy)
- **Fusion via ModuleWorks**: simultaneous 5-axis roughing for floor-less pocket areas (with Pocket or Adaptive styles) and simultaneous 5-axis pocket finishing with barrel-cutter support — [Autodesk Dec 2022 post](https://www.autodesk.com/products/fusion-360/blog/fusion-360-machining-extension-future-moduleworks-machining-strategies/)
- **Tool-axis control vocabulary (Fusion)**: From/To Point and From/To Curve. Lead tilts the tool along the direction of travel and lean tilts it sideways; negative lead means lag — [Fusion help: Steep & Shallow Tool Axis Mode](https://help.autodesk.com/cloudhelp/ENU/Fusion-CAM/files/MFG-REF-3D-STEEP-SHALLOW-TOOL-AXIS-MODE.htm)
- **Tool-axis control (vendor-neutral)**: with a point or line target, the tip follows the path while the top of the tool aims at that target. Lag is tilt away from the direction of travel — [Hurco blog](https://blog.hurco.com/blog/bid/242006/5-axis-programming-understanding-tool-axis-and-collision-controls). Other modes include normal-to-surface and tilt relative to cutting direction with side tilt — [BobCAD](https://bobcad.com/?p=51607). Mastercam uses axis limits to keep tilt within part and machine limits — [Mastercam Multiaxis (CTE)](https://ctemag.com/?p=109914)
- **Barrel/conical-barrel tools**: OPEN MIND (hyperMILL) developed the conical barrel cutter and supports it with "5-axis tangent machining" in hyperMILL MAXX Machining. Cited barrel radii run about 250–1,500 mm with 6–8 mm stepovers. An aerospace case reports a 12× larger step-down than ball-nose finishing and about 5 hours saved per part (vendor claims) — [OPEN MIND conical barrel cutter](https://openmind-tech.com/en/cam/hypermill-maxx-machining/finishing/conical-barrel-cutter); [mfgnewsweb](https://www.mfgnewsweb.com/archives/4/47738/Software-oct16/ad_info.aspx); [electronicspecifier](https://aerospacedefence.electronicspecifier.com/design-3/reduced-cycle-time-in-aerospace-manufacturing)
- **Academic background**: tool-orientation planning is typically done with configuration-space (C-space) search (Jun, Cha & Lee 2003; Morishige 1997). Global collision avoidance can use physically based models (Lacharnay et al., CAD 2015). For ball-end tools, orientation and CC point can be planned separately; for flat-end tools they are coupled. Makhanov (2010) surveyed five-axis geometric patterns (IJAMT 47:1167–1208) — [Springer IJAMT](https://link.springer.com/article/10.1007/s00170-023-12031-7); [Springer s00170-022-09445-0](https://link.springer.com/article/10.1007/s00170-022-09445-0); [jzus survey](https://www.springerprofessional.de/article/10.1631/jzus.A1200160); [McMaster thesis](https://macsphere.mcmaster.ca/bitstream/11375/7591/1/fulltext.pdf); [rotation-minimizing frame orientation](https://journal.hep.com.cn/aim/EN/10.1007/s40436-019-00271-4)

### Inferences
**Feature checklist, Level 2c: 5-axis** (definitions are domain knowledge and uncited, except where cited above)
- **A. Positional**
  - 3+2 indexed with automatic orientation search (accessibility cones).
  - Multi-orientation roughing (auto 3+2).
  - Tilted-plane output (G68.2/CYCLE800) vs. fully solved rotary angles.
- **B. Auto-convert 3→5**: take a 3-axis pattern (parallel/scallop/pencil/etc.) and add tilt so a shorter tool can be used. Modes are fixed lead/lean, tilt away from point/curve, or automatic tilt on collision ("tilt only when needed").
- **C. Swarf**: side-milling of ruled walls. Needs multi-pass, floor offset, and handling of non-ruled surfaces ("swarf with deviation"). Known for: hyperMILL, NX, Mastercam, PowerMill, Fusion.
- **D. Multi-axis contour/flowline/morph/blend/parallel-to-curve**: finishing driven by surface UV or curves.
- **E. Multi-axis roughing**: 5-axis-sim pocket roughing for floor-less pockets, and rest roughing with tilt.
- **F. Curve-based/trimming**: 5-axis trim of thermoformed/composite parts with the tool normal to the surface along an edge curve.
- **G. Application packs**: port machining (inlet/exhaust ports, reached from both ends with collision-free tilt); impeller/blisk (hub, blade, splitter, fillet; Mastercam Blade Expert, hyperMILL, NX Turbomachinery, PowerMill Blade); turbine blade (spiral around airfoil); tube/manifold.
- **H. Deburr/chamfer**: edge detection plus automatic tilt.
- **I. Barrel/oval/lens/taper-barrel tools**: tangent-plane finishing and specialized gouge checks. Known for: hyperMILL MAXX, NX, PowerMill, Mastercam, Fusion (in some multi-axis ops).
- **J. Geodesic**: constant-distance offsets along the surface. Used in some vendors' 5-axis finishing; I found no source in this session.
- **K. Tool-axis control modes**: fixed; normal to surface; lead/lag + tilt/lean; to/from point; to/from line or curve; through/along chain or curve; interpolated between user-defined vectors; constant angle to axis; limited cone; auto-collision-tilt.
- **L. Collision avoidance hierarchy**: shank, holder, extension, fixture, machine head/table. Responses include tilt away, retract, trim the path, or lengthen the tool. A "minimum tool stickout" calculation is part of this.
- **M. Motion quality**: tool-axis smoothing (avoid sudden rotary flips); singularity handling near the pole (for example, B=0 on head/table; small-angle avoidance, preferred solution); rotary limits and wind-up; inverse-time feed; output of linearization tolerance and points per degree.
- **Highest-value gap for a plugin**: automatic tool-axis selection with guaranteed collision freedom against holder, fixture and machine, plus smoothing. This is the hard algorithmic core, the subject of current academic work, and the place where users struggle most (see section 7).

### Gaps
- I did not confirm whether Fusion currently supports conical-barrel tools across all multi-axis operations, or only in select ModuleWorks-based ops.
- I found no 2025–2026 independent review that ranks vendors on 5-axis collision avoidance.
- I did not research geodesic machining or tube/manifold sources in this session.

---

## 5. Machine kinematics and simulation

### Takeaway
Multi-axis CAM must model the three kinematic classes (**table-table (trunnion), head-table, head-head**) and support **RTCP/TCPC**-style output (controller does compensation) as well as non-RTCP output (CAM solves pivot offsets). Full machine-kinematic simulation is table stakes for 5-axis credibility. Small machines such as the Pocket NC are trunnion-style table-table machines whose WCS has to match the B-table orientation.

### Cited Findings
- On Pocket NC V2, the Y axis moves the table up and down, and the setup has to reflect this or the output code fails. WCS orientation is therefore a kinematic issue, not just a cosmetic one — [Pentamachine WCS tutorial](https://pentamachine.atlassian.net/wiki/spaces/PNFUR/pages/1732739077/V2+Series+Work+Coordinate+System+Tutorial+Kinetic+Control)
- Pocket NC tutorials were rewritten for the Kinetic Control interface (released Nov 1, 2021), and post-specific content was moved out of the tutorial in March 2022. This suggests the post and controller pairing changes over time — [Pocket NC V2 Demo Part (Kinetic Control)](https://pentamachine.atlassian.net/wiki/spaces/PNFUR/pages/1732673537/Pocket+NC+V2+Demo+Part-Spiral+Kinetic+Control)
- Mastercam 2026.R2 can build machine groups from a verbal description through Copilot — [Mastercam 2026.R2 press](https://www.bolsamania.com/nota-de-prensa/mercados/mastercam-2026r2-delivers-gpu-accelerated-simulation-and-ai-powered-assistance--21635723.html)

### Inferences
**Feature checklist, Level 3: machine** (domain knowledge, uncited)
- *Kinematic model*: axis chain, pivot points, limits, home, and rotary direction conventions. Should import from vendor machine files (Fusion .mch/machine library).
- *Table-table*: most common on small/prosumer machines (Pocket NC, Haas UMC, DMG MORI DMU eVo-type trunnions). Part setup offset relative to the rotary center matters.
- *Head-table*: B head + C table, common on larger parts.
- *Head-head*: gantry/large aerospace and mold machines.
- *RTCP/TCPC/TRAORI/M128*: controller-side tool-center-point compensation. Needed for portable programs; without it, CAM must output machine-specific coordinates.
- *Machine simulation*: driven by CL vs. by G-code, with near-miss tolerance and axis-limit violation reporting.
- *Solver issues*: choosing between two rotary solutions, handling rotary-limit overtravel (retract-rotate-reengage), and singularity.
- For Fusion, machine simulation already exists in the base product (per Autodesk marketing). A plugin should reuse Fusion's machine definition rather than introduce its own. This is an inference; it needs API verification.

### Gaps
- I did not fetch Fusion machine-simulation documentation (403). I did not confirm which Fusion tiers include full machine simulation in 2026.

---

## 6. Market trends 2023–2026

### Takeaway
The CAM market is about **$3.1B (2025) and growing 8–10%/yr**. The dominant 2024–26 trend is **AI-assisted programming**: CloudNC CAM Assist; Toolpath (now partnered with and invested in by Autodesk); Mastercam Copilot; Autodesk's announced AI CAM at AU 2026. The other trends are **automation of 3+2 orientation**, **GPU simulation**, **physics/engagement-based roughing**, **barrel tools**, and the early emergence of **robot-machining CAM** as a product category.

### Cited Findings
- **Market size**: CIMdata estimates CAM software and services grew 9.5% in 2025, from $2.9B to $3.1B in end-user payments, and forecasts +8.3% to $3.4B in 2026. More than two-thirds of payments go directly to suppliers and somewhat less than one-third to resellers (article dated June 19, 2026) — [DE247: CIMdata 2026 CAM report](https://www.digitalengineering247.com/article/cimdata-releases-2026-global-cam-market-analysis-report)
- **Market size, prior year**: CAM grew 6.2% in 2024, from $2.7B to $2.9B — [CIMdata](https://www.cimdata.com/en/news/item/30210-cimdata-publishes-global-cam-market-analysis-report)
- **CIMdata on demand**: CAM purchases are tied to machine tool sales. The press summary does not discuss AI or cloud as trends — [DE247](https://www.digitalengineering247.com/article/cimdata-releases-2026-global-cam-market-analysis-report)
- **CloudNC CAM Assist**:
  - March 2024: added AI strategies for 3+2 and is available on the Fusion App Store — [Engineering.com](https://www.engineering.com/?p=100189)
  - September 2025: CAM Assist 2.0 lets users review and edit each strategy operation before committing. CloudNC says it has more than 1,000 shops as users and supports Fusion, Mastercam and Siemens NX (vendor claims) — [Design World](https://www.designworldonline.com/cloudnc-cam-assist-software-creates-toolpaths-with-ai/); [PES Media](https://pesmedia.com/ai-generated-32-machining-strategies)
- **Toolpath (toolpath.com)**:
  - Offers AI quoting, DFM and CAM automation, and generates toolpaths inside Fusion. Its simulation is optimized for 3-axis — [toolpath.com](https://toolpath.com/); [Engineering.com](https://www.engineering.com/?p=140047)
  - $10M seed in September 2024 led by Leaders Fund. A later round was led by Kennametal with ModuleWorks participating, and Autodesk made a strategic investment (Aug 2025) — [Develop3D](https://develop3d.com/manufacture/toolpath-sees-further-investment-in-its-ai-cam-tools/); [DE247](https://www.digitalengineering247.com/article/autodesk-invests-in-toolpath-fueling-ai-innovation-in-manufacturing/news)
- **Autodesk AU 2026 (September 2026)**: announced "AI-generated CAM programming" coming to Fusion, which generates toolpaths and learns from user adjustments. Also announced a Toolpath partnership for AI quoting and machining in Fusion. No availability dates were given — [Autodesk blog](https://www.autodesk.com/products/fusion-360/blog/ai-powered-workflows-coming-to-autodesk-fusion/); [The Manufacturer](https://www.themanufacturer.com/articles/autodesk-expands-fusion-capabilities-leveraging-ai-to-unlock-automation-and-creativity-across-design-and-make/)
- **Earlier Autodesk AI positioning in Fusion manufacturing** (partners including CloudNC) — [Autodesk blog: driving manufacturing forward with AI](https://www.autodesk.com/products/fusion-360/blog/driving-manufacturing-forward-with-ai/)
- **Mastercam 2026.R2 (February 9, 2026)**: Copilot takes voice or text commands, edits feeds and speeds across operations, knows about 200 toolpath types, and searches training videos. GPU simulation, automatic tilting, and better 4-axis swarf were also added. Mastercam has moved to semi-annual releases — [Mastercam 2026 R2](https://www.mastercam.com/solutions/products/mastercam-2026-r2/); [New Equipment Digest](https://www.newequipment.com/product-directory/software-computers/software/cad-cam/product/55360437/cnc-software-llc-ai-enhanced-cam-software-speeds-simulation-and-programming)
- **Kernel trend (ModuleWorks 2026)**: automatic 3+2 orientation, physics reports, automatic feed optimization, thin-wall awareness — [ModuleWorks 2026.08](https://www.moduleworks.com/software-components/whats-new/software-release-2026-08/). A 2025.12 release was themed around "automation, safety and quality" — [DE247](https://www.digitalengineering247.com/article/moduleworks-2025.12-increases-automation-safety-and-quality/moduleworks)
- **Robot machining**: RoboDK launched RoboDK CAM in February 2026, covering milling, drilling, deburring, cutting and additive, with 3→5-axis conversion and collision checks. It was demoed at Automate 2026 on a Mecademic robot. The vendor claims up to 40% less deployment time — [Business Wire](https://www.businesswire.com/news/home/20260206791669/en/RoboDK-Launches-CAM-Solution-Designed-to-Slash-Robotic-Machining-Deployment-Times); [Robotics 24/7](https://www.robotics247.com/article/automate-2026-robodk-demonstrates-new-cam-software-that-saves-robotic-machining-deployment-time)
- **Barrel tools**: conical-barrel finishing has moved from a hyperMILL innovation into wider support, including barrel support in Fusion multi-axis pocket finishing via ModuleWorks — [OPEN MIND](https://openmind-tech.com/en/cam/hypermill-maxx-machining/finishing/conical-barrel-cutter); [Autodesk](https://www.autodesk.com/products/fusion-360/blog/fusion-360-machining-extension-future-moduleworks-machining-strategies/)
- **Additive/hybrid**: the Fusion Manufacturing Extension bundles metal additive and sheet-metal nesting alongside 4/5-axis — [Autodesk blog](https://www.autodesk.com/products/fusion-360/blog/fusion-360-machining-extension-beyond-5-axis-machining/)

### Inferences
- AI CAM moved from "generate a whole program" (2023–24) to "generate, then let the human review and edit, and learn from the edits" (CAM Assist 2.0, Autodesk AU26). **Human-in-the-loop editability is now expected.**
- **Competitive risk**: Autodesk is building native AI CAM and has both partnered with and invested in Toolpath. CloudNC already sells into Fusion. A plugin that competes on "AI auto-programming for 3-axis/3+2 in Fusion" would face Autodesk itself plus two funded incumbents. Simultaneous 5-axis, which the existing AI tools barely address (Toolpath is 3-axis-optimized; CAM Assist is 3-axis + 3+2), looks like the less crowded lane.
- Cloud CAM: Fusion itself is the cloud-CAM incumbent. I found no 2025–26 source that shows other vendors shifting to cloud-native CAM.

### Gaps
- I did not find independent (non-vendor) measurements of AI CAM productivity.
- I found no CIMdata vendor rankings in free sources.
- I did not research Siemens NX or Hexagon AI features (other researchers are cataloging products).
- I did not research hybrid additive/subtractive CAM trends beyond the Fusion extension bundling.

---

## 7. User pain points and underserved segments

### Takeaway
The pain points that recur most are: **posts**, meaning non-fitting generic posts that force G-code hand edits; **collision setup and verification burden**; **programming time and learning curve** for complex 5-axis work (anecdotally tens of hours per part); and **cost**. Prosumer 5-axis machines (Pocket NC/Pentamachine at about $4.9k–$9.7k) mostly run Fusion. For that segment, the CAM cost of high-end 5-axis packages (hyperMILL/NX/PowerMill-class) is far above the machine cost, so Fusion plus its extension is the realistic ceiling. This is where a focused plugin could fit.

### Cited Findings
- **Generic posts**: a near-fit post forces G-code hand edits on every program, and machines with non-standard attachments make it worse — [MecSoft blog](https://mecsoft.com/blog/why-many-shops-regret-their-cam-software-purchase)
- **Programming effort**:
  - The variables that drive 5-axis programming hours are tool count, feature access and tolerance/finish. One example cites about 80 programming hours for a fully ported 5-axis part. The same source claims a 3–5× spread across shops but gives no source (low reliability) — [industrialmonitordirect](https://industrialmonitordirect.com/de/blogs/knowledgebase/5-axis-programming-time-estimation-hypermill-best-practices)
  - CloudNC frames the value as strategies in seconds or minutes versus hours or days manually, and about 300 programming/estimating hours saved per shop per year (vendor claim) — [Design World](https://www.designworldonline.com/cloudnc-cam-assist-software-creates-toolpaths-with-ai/)
- **Prosumer 5-axis pricing**: Pocket NC V2-8Lite was $4,900 (Oct 2024), the lowest-priced 5-axis entry from Pentamachine. Retailer listings show the V2-10 at about $6,500 and the V2-50 at about $9,700 — [All3DP](https://all3dp.com/4/penta-machine-company-announces-pocket-nc-v2-8lite-desktop-cnc-machine/); [RobotShop V2-10](https://robotshop.com/products/pocket-nc-pocket-nc-v2-10); [RobotShop V2-50](https://www.robotshop.com/products/pocket-nc-pocket-nc-v2-50)
- **Prosumer workflow**: Pentamachine's tutorials are built around Fusion, with workholding left out of the CAM model and setup done by touch-off — [Pentamachine First Part Tutorial](https://pentamachine.atlassian.net/wiki/spaces/PNFUR/pages/1727037445)
- **Fusion users want multi-axis on more operations**: Autodesk solicited forum feedback on multi-axis options added to 3D ops (ball/lollipop only, no lead/lean at the time) — [Autodesk forum](https://forums.autodesk.com/t5/fusion-manufacture-forum/open-looking-for-feedback-on-the-multi-axis-options-which-have/td-p/11189111)

### Inferences
**Underserved segments and plugin opportunities** (inference)
1. **Prosumer/desktop 5-axis** (Pocket NC, trunnion add-ons, DIY 5-axis such as LinuxCNC builds): needs accurate small-machine kinematics, easy fixture models, simple sim-verified 5-axis finishing, and posts for LinuxCNC-family controllers.
2. **Small job shops with Haas UMC-class trunnions**: mostly 3+2 with some simultaneous work. They need automatic orientation, automatic tilt for short tools, and holder- and fixture-aware collision avoidance without per-op tuning.
3. **Specialty application packs missing from Fusion**: port, impeller/blisk, blade, tube. In the high-end packages these are paid add-ons. Whether Fusion has equivalents was not verified this session.
4. **"Explainable" collision avoidance**: show why a tilt was chosen, and give a reachability map before programming. No source; this is an inferred pain point.
5. **Post/kinematic validation tools for Fusion**: a large pain point. Note that the Fusion post is JavaScript, so a plugin could add post-verification or G-code simulation checks.
- Fusion is used by Pentamachine as the reference CAM, and AI vendors sell into Fusion first. Both signals suggest Fusion's App Store is a viable distribution channel for multi-axis add-ons.

### Gaps
- I found no published survey of the multi-axis CAM learning curve, and no quantitative data on pain points. Evidence here is anecdotal or vendor-sourced.
- I found no current (2025–26) Pentamachine pricing directly from the vendor.
- I did not quantify the prosumer 5-axis segment (units sold, number of users).
- I did not check Fusion Manufacturing Extension pricing in this session (another researcher may cover it).
