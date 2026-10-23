# CAM Infrastructure Layer: Toolpath Kernels, NC Verification/Machine Simulation, Post-Processors (as of Oct 2026)

Scope note: about 20 search/fetch calls. Vendor pages and press releases are the main sources, so customer and partner claims are mostly vendor-reported. Licensing terms for the commercial SDKs (ModuleWorks, MachineWorks) are **not public**, and everything below about pricing is uncertain.

## Q1. Toolpath kernels / SDKs: what can be licensed vs built?

### Takeaway
ModuleWorks is the main commercial toolpath and simulation component supplier. Autodesk Fusion, Mastercam, Siemens, PTC, Hexagon, SolidCAM, CAMWorks, BobCAD, TopSolid and about 200 others are listed as partners. Autodesk and PTC also hold minority equity stakes, which matters for a Fusion plugin developer, because the host vendor is itself a ModuleWorks partner and investor. MachineWorks is the main competitor on the **simulation/verification-kernel** side (it does not supply toolpath strategies). Its logo list also overlaps heavily: Fusion 360, HSMWorks, FeatureCAM, Hexagon/Vero brands, NCSIMUL, ICAM and IMS. On the open-source side there are mostly 3-axis building blocks: OpenCAMLib (LGPL), libarea/libactp (Heeks), FreeCAD's Adaptive and OCCT (LGPL 2.1 + exception). Nothing open-source covers production-grade 5-axis.

### Cited Findings
**ModuleWorks (Aachen, DE)**
- Current product menu:
  - Toolpath: 5-Axis, 4-Axis and 3-Axis Machining, Area Roughing, Automatic 3+2-Axis Roughing, Adaptive Roughing, SmartRoughing, Wall/Floor/Rest Finishing, SuperFinish, Turning, High Dynamic Turning, FeedControl, plus VoluMill/VoluTurn (Celeritive Technologies products).
  - Simulation: GPU Simulation, Simulation SDK, Cutting/CNC/Machine Simulation, Smart Measure, Visual Twin.
  - On-machine: Collision Avoidance System, Toolpath on CNC, Simulation on CNC, Shopfloor Programming.
  - Utilities: MDES–Equipment, Mesh Toolkit, **Post Processor Framework**, Stock Manager.

  Release cadence: 2025.12, 2026.04 and 2026.08 release pages are listed. — [ModuleWorks homepage](https://www.moduleworks.com/)
- Claims 200+ partner companies worldwide. — [ModuleWorks homepage](https://www.moduleworks.com/)
- The CAD/CAM partners page shows logos for: Autodesk Fusion, Autodesk, alphacam, BobCAD, CAMWorks, CAXA, Cimatron, edgecam, Exapt, GibbsCAM, Hexagon, InventorCAM, Mastercam, MecSoft, NCG CAM, PTC, Siemens, SmartCAM, SolidCAM, Spatial, Surfcam, TopSolid, Vero Software, Visi, and others.
  - CNC/OEM partners: Fanuc, Siemens, Beckhoff, Okuma, Hurco, Mitsubishi, Bosch Rexroth, Tornos, CHIRON, DN Solutions, and others.
  - The page does **not** say which partners license which components. — [ModuleWorks Partners](https://www.moduleworks.com/partners/)
- No public price list or startup program was found. The only hint is a customer testimonial from Global Dental Science: "They are very flexible in their pricing model which helped us a lot in the start-up phase." — [ModuleWorks Partners](https://www.moduleworks.com/partners/)
- A third-party directory says ModuleWorks publishes no pricing and sells to developers/OEMs. A company profile mentions "subscription-based services and one-time licensing fees". Both are secondhand and **uncertain**. — [fitgap](https://us.fitgap.com/products/moduleworks); [dealroom](https://app.dealroom.co/companies/moduleworks_gmbh)
- Strategic Partner Program (2023): key licensing partners took minority equity stakes. DMG MORI led, and **Autodesk and PTC** participated, with no change of control. Kennametal joined later. — [Digital Engineering 24/7](https://www.digitalengineering247.com/article/moduleworks-kicks-off-strategic-partner-program/cam)
- Rollout of ModuleWorks GPU simulation was reported as scheduled for upcoming Cimatron and Mastercam releases. ModuleWorks also extended its partnership with MAL Inc. (physics-based virtual machining/optimization) and DN Solutions (machine-tool software). — [Digital Engineering 24/7 company page](https://www.digitalengineering247.com/company/moduleworks); [DN Solutions article](https://www.digitalengineering247.com/article/moduleworks-partners-with-dn-solutions/moduleworks)
- "CAMbridge" was not found on the current ModuleWorks homepage menu, and no pricing turned up for it. See Gaps. — [ModuleWorks homepage](https://www.moduleworks.com/)

**MachineWorks (Sheffield, UK): simulation/verification SDK, not toolpath strategies**
- Technology areas:
  - Solid Modeling, Swept Volumes, Pixel-Perfect Graphics, Machine Simulation, Cutting Analysis, Feature Recognition, Additive Simulation, Cloud Simulation.
  - Tools: "Machine Builder" (builds the CNC kinematics tree) and "Machine Simulator" (ready-made UI).
  - Sister product: Polygonica (mesh toolkit).

  — [MachineWorks customers](https://www.machineworks.com/customers)
- Customer logos include:
  - CAD/CAM: Autodesk Fusion 360, CATIA, Hexagon, Open Mind, SolidCAM, Cimatron, GibbsCAM, TopSolid, CAMWorks, ZWSoft, EdgeCAM, VISI, WorkNC, SurfCAM, NCSimul, AlphaCAM, PEPS, Machining Strategist, PartMaker, FeatureCAM, HSMWorks, MecSoft.
  - Post/verify: ICAM, IMS.
  - Controls/OEMs: Heidenhain, DMG MORI, Okuma, Makino, Nakamura-Tome, Starrag, FIDIA (Vi-Mill anti-crash), GOODWAY (G-Linc 350 controls).

  — [MachineWorks customers](https://www.machineworks.com/customers)
- Claims ">60% of CAM developers" have integrated MachineWorks since 1994. This is vendor-reported and unverified. — [MachineWorks](https://www.machineworks.com/)
- Vero Software extended a 17-year relationship across PEPS, Alphacam, Edgecam and Surfcam (2016). MecSoft has been a customer since 2003 and licensed cloud simulation for VisualCAMc for Onshape. — [MachineWorks news (Vero)](https://www.machineworks.com/news/machineworks-used-within-vero-software-portfolio/); [MachineWorks news (VisualCAMc)](https://www.machineworks.com/news/visualcamc-for-onshape-to-use-cloud-based-machineworks-software/)
- No licensing terms are published. — [MachineWorks customers](https://www.machineworks.com/customers)

**Open source**
- **OpenCAMLib** (Anders Wallin) is C++ with Python, Node.js and browser bindings, and is on PyPI as `opencamlib`.
  - License: LGPL since August 2018 (the gitee mirror shows LGPL-2.1).
  - Algorithms: drop-cutter (drop the tool at fixed XY until contact), push-cutter (used to build waterline at constant Z).
  - Cutters: cylindrical, ball, bull/toroidal, cone, composite.

  — [GitHub aewallin/opencamlib](https://github.com/aewallin/opencamlib); [docs](https://opencamlib.readthedocs.io/); [PyPI](https://pypi.org/project/opencamlib/2022.12.17)
- **FreeSteel / libactp**: FreeSteel was written by Julian Todd and Martin Dunschen under the GPL. libactp is a C-library implementation of its adaptive clearing algorithm, hosted at github.com/Heeks/libactp. Todd later worked at Autodesk; Fusion's "Adaptive Clearing" lineage traces back to his HSMWorks/Freesteel work, which is background knowledge and not verified here. — [openSUSE libactp](https://build.opensuse.org/package/show/Education/libactp); [LinuxCNC forum](https://forum.linuxcnc.org/38-general-linuxcnc-questions/37092-adaptive-clearing-hsm)
- **libarea** (Dan Heeks) computes profiles and pockets with Python modules. It is used by HeeksCNC and the FreeCAD Path (now "CAM") workbench. — [FreeBSD libarea pkg-descr](https://cgit-dev.freebsd.org/ports/plain/devel/libarea/pkg-descr)
- **FreeCAD CAM Adaptive** is a separate implementation that originated in kreso-t/FreeCAD_Mod_Adaptive_Path. The wiki warns it "might contain a few bugs". — [FreeCAD wiki CAM Adaptive](https://wiki.freecad.org/CAM_Adaptive/it)
- **Open CASCADE Technology (OCCT)**: LGPL 2.1 with an additional exception. Open Cascade sells support, maintenance, consulting and commercial "DevTools" components on top. — [occt3d.com](https://occt3d.com/open-cascade-technology/index.html)

### Inferences
- Build vs buy for a Fusion plugin:
  - **Inside Fusion**, the host already does geometry (ShapeManager/ASM kernel), stock simulation, and post-processing via .cps. A plugin probably needs its own toolpath math (or a ModuleWorks license) plus a way to get toolpaths back into Fusion's operation/post pipeline. That depends on the Fusion CAM API, which is covered by another researcher.
  - **ModuleWorks is the only off-the-shelf 5-axis strategy library.** Since Autodesk is a partner/investor, ModuleWorks may treat a Fusion plugin as a conflict or require Autodesk coordination. This is speculative; ask ModuleWorks directly.
  - **Open-source building blocks** (OCL, libarea, Clipper, which is widely used for 2D offsets) cover 2.5D/3-axis. True 5-axis swarf, multi-axis finishing, tilting and collision avoidance would have to be built in-house or licensed.
- **Geometry kernels not verified this session** (background knowledge, needs confirmation): Parasolid (Siemens), ACIS/3D InterOp (Spatial, Dassault) and CGAL (GPL/commercial dual-license from GeometryFactory) are all commercially licensable. ModuleWorks also lists Spatial as a partner. — [ModuleWorks Partners](https://www.moduleworks.com/partners/)
- **Who uses which (inference from the logo lists):** many CAM vendors use **both** — ModuleWorks for multi-axis toolpaths and MachineWorks for verification/sim. Examples are Autodesk, Hexagon/Vero brands, SolidCAM, Cimatron, GibbsCAM, CAMWorks, TopSolid and MecSoft. The logos do not confirm current active licenses.

### Gaps
- No ModuleWorks or MachineWorks pricing, royalty structure or minimum fees. No confirmed startup program, beyond the "flexible pricing" testimonial.
- Status of "CAMbridge" not confirmed: no current product page found.
- Parasolid, ACIS, CGAL and Spatial licensing were not researched in this session.
- Maintenance status of libactp is unknown.

## Q2. NC verification and machine simulation products

### Takeaway
The independent G-code-driven verification market is led by CGTech **VERICUT**, currently v9.6 (2025) with AI assistant features and **Force** physics-based feed optimization, and by Hexagon **NCSIMUL** (from Spring Technologies, acquired 2018; a May 2026 release added GPU "selective simulation"). **CAMplete TruePath is Autodesk-owned, not Hexagon.** It came with Delcam, and Autodesk sells it as "Fusion with CAMplete". Post-centric simulators are ICAM Virtual Machine, IMSverify and Roboris Eureka (licensed into SolidCAM).

### Cited Findings
- **VERICUT 9.6**:
  - Announced June 30, 2025; the Japanese notice gives Oct 7, 2025.
  - Adds Vericut Assistant (AI in-app) and Vericut Intelligence (web knowledge hub).
  - Expands CAD/CAM interfaces, including NX auto-loading of stock, fixtures and designs.
  - Supports Siemens Run MyVirtual Machine and Fanuc Robot TP.

  — [automation.com](https://www.automation.com/en-us/products/july-2025/cgtech-announces-vericut-9-6); [vericut.com news](https://vericut.com/company/news/vericut-version-9.6-ai-meets-precision); [CGTech Japan via ipros](https://pr.mono.ipros.com/en/vericut/news/detail/192836/)
- **VERICUT Force** is a physics-based NC program optimization module for milling and turning:
  - Real-time force/chip-load charts linked to NC lines.
  - v9.1 added "learn mode" (learns while simulating, then auto-optimizes).
  - v9.3 highlighted feedrate and tool-deflection optimization.

  — [CGTech VERICUT 9.1](https://cgtech.com/vericut-9-1); [engineering.com](https://www.engineering.com/?p=13361)
- A VERICUT–Autodesk Fusion interface was not confirmed by any search result. See Gaps.
- **NCSIMUL (Hexagon)**:
  - Hexagon acquired SPRING Technologies (France, ~100 staff) in June 2018 into its Manufacturing Intelligence division. — [MMS Online](https://www.mmsonline.com/news/hexagon-ab-acquires-spring-technologies); [machinery.co.uk](https://www.machinery.co.uk/content/news/hexagon-ab-buys-ncsimul-author-spring-technologies-as-part-of-its-autonomous-connected-ecosystem-strategy)
  - May 19, 2026 release adds "Selective Simulation" and GPU-accelerated Rest Stock Previews (intermediate stocks during NC decoding), while keeping full simulation for sign-off. It is positioned as G-code verification, CNC simulation and optimization in one digital twin, for any CAM or CNC. — [Hexagon press release 2026](https://hexagon.com/company/newsroom/press-releases/2026/hexagon-helps-manufacturers-verify-and-optimise-nc-programs-faster-with-ncsimul); [Hexagon Nexus NCSIMUL](https://nexus.hexagon.com/home/product/ncsimul/)
  - NCSIMUL itself appears among MachineWorks customer logos. — [MachineWorks customers](https://www.machineworks.com/customers)
- **CAMplete (Autodesk)**:
  - TruePath targets multi-axis mills; TurnMill targets Nakamura-Tome machines.
  - Uses machine-builder-supplied models; checks collisions and overtravel.
  - Posts are developed with the machine builders and are editable.

  Sold as "Fusion with CAMplete". Autodesk acquired it via Delcam and still sells it as a package for CAM systems that compete with Fusion. — [Autodesk CAMplete features](https://www.autodesk.com/products/camplete/features); [Digital Engineering 24/7](https://www.digitalengineering247.com/article/celebrating-the-union-of-cad-and-cam/integrated-engineering)
- **ICAM Virtual Machine** pairs with CAM-POST and simulates how the machine reacts to the **post-processed G-code**, rather than the APT/CL toolpath. — [ICAM](https://fr.icam.com/?p=18478)
  - ICAM CAM Integration V27 supports **Fusion 360 2021–2025** and Mastercam 2019–2025. Fusion outputs APT-CL through a special post; ICAM GENER then post-processes it, and an Extractor pulls tools, holders, parts and fixtures for Virtual Machine. — [ICAM help V27](https://help.icam.com/270/english/integ/); [ICAM Fusion 360 kit](https://help.icam.com/270/english/cam/kit/fusion360/readme)
- **Roboris Eureka**:
  - EurekaGCode: G-code simulation with a 3D machine model; emulates Fanuc, Siemens, Heidenhain, Haas and others.
  - EurekaRobot: APT/ISO to robot cells.
  - SolidCAM licensed the Eureka engine (2020) as its "SolidCAM G-Code Simulator", including multi-channel mill-turn and Swiss.

  — [Metalworking World](https://www.metalworkingworldmagazine.com/roboris-eureka-virtual-machining/); [DE247 SolidCAM–Roboris](https://www.digitalengineering247.com/article/solidcam-and-roboris-join-to-integrate-g-code-simulation-with-solidcam/cam)
- **IMSverify** verifies post-processed code and can run the IMSpost processor inside the simulation. IMS uses MachineWorks for material removal, part comparison and collision checking. — [DE247 IMSverify](https://www.digitalengineering247.com/article/imsverify-2008-updated-for-solids-based-technology); [MachineWorks customers](https://www.machineworks.com/customers)
- **Open/free**: CAMotics (Cauldron Development) is GPLv2+, simulates **3-axis only**, and exports the cut workpiece as STL. — [CAMotics GitHub mirror](https://github.com/KimK/CAMotics); [LinuxLinks](https://www.linuxlinks.com/camotics-3-axis-cnc-gcode-simulator/)

### Inferences
- A new Fusion 5-axis plugin can lean on Fusion's built-in stock simulation and machine simulation for CL-level checks. For G-code-level sign-off, customers will expect VERICUT, NCSIMUL or CAMplete. CAMplete is Autodesk's own and is the most natural "Fusion-ecosystem" verification partner.
- Licensable sim kernels for embedding in a plugin are MachineWorks and ModuleWorks Simulation SDK. Open source offers nothing 5-axis-capable comparable to them; CAMotics is 3-axis.

### Gaps
- Not confirmed: a VERICUT–Fusion interface or a NCSIMUL–Fusion interface. Both very likely accept Fusion output via posts, but no source was found.
- NCViewer (a browser G-code plotter) and other free 5-axis viewers were not researched. NCViewer is believed to be a 3-axis line plotter, which is unverified.
- No pricing found for any verification product.

## Q3. Post-processor ecosystem and typical 5-axis post issues

### Takeaway
Fusion posts are JavaScript (.cps) files run by Autodesk's post engine. The API includes machine-configuration objects (`MachineConfiguration`, `getABCByPreference`, `optimizeMachineAngles2`) and supports inverse-time vs DPM output. Third-party post ecosystems (ICAM CAM-POST, IMSpost) consume CL/APT data, and ICAM ships an official Fusion kit. Recurring 5-axis post issues are TCP/RTCP mode selection, 3+2 vs simultaneous handling, and choosing the angle solution or preferences.

### Cited Findings
- Autodesk's post documentation has sections for Configuration, Keywords, Entry Functions, Cycles and Translation. It warns that all NC must be verified before use. — [Autodesk CAM Post Processor reference](https://cam.autodesk.com/posts/reference/)
- `PostProcessor.optimizeMachineAngles2(optimizeType)` exists. Forum sources **conflict** on the meaning of mode 0 versus 1: one calls 0 "maintain tool tip position (TCPM)", another calls 0 "map tip" and says to use 1 if the machine lacks TCP. Check the current reference. — [PostProcessor class ref](https://cam.autodesk.com/posts/reference/classPostProcessor.html); [Autodesk forum](https://forums.autodesk.com/t5/hsm-post-processor-ideas/idb-p/hsm-post-processor-ideas-en/label-name/autodesk%20proposed); [Autodesk forum Fanuc 5x](https://forums.autodesk.com/t5/hsm-post-processor-forum/fanuc-5x-no-a-c-indexing/m-p/7354454)
- `MachineConfiguration.getABCByPreference` returns the preferred ABC angles for an orientation, keeping forward (and optionally right) direction, governed by controllingAxis, control and options. — [MachineConfiguration ref](https://cam.autodesk.com/posts/reference/classMachineConfiguration.html)
- Autodesk's Bob Schultz says `optimizeMachineAngles2` only adjusts coordinates for multi-axis operations. For 3+2, set the `tcp` flag in `getWorkPlaneMachineABC` to false. — [Autodesk forum Fanuc 5x](https://forums.autodesk.com/t5/hsm-post-processor-forum/fanuc-5x-no-a-c-indexing/m-p/7354454)
- Feed modes:
  - Inverse-time feed = 1/move-time, and is recommended over degrees-per-minute (DPM) when the control supports both.
  - Machines with TCP generally accept direct feed-per-minute during multi-axis contouring.
  - Posts expose a `useInverseTimeOutput` property.

  — [Autodesk forum Multi-Axis Feedrates](https://forums.autodesk.com/t5/hsm-post-processor-forum/multi-axis-feedrates/m-p/7346609)
- LinuxCNC users report the stock Fusion post works directly if the machine runs TCP kinematics, and that jogging under TCP kinematics is awkward. — [LinuxCNC forum](https://forum.linuxcnc.org/42-deutsch/34328-fusion360-postprozessor-fuer-linuxcnc-5-achsen)
- RTCP on/off is a recurring forum topic (e.g., "How to off RTCP"), as are CMS/OSAI TCP posts. — [Autodesk forum RTCP](https://forums.autodesk.com/t5/hsm-post-processor-forum/how-to-off-rtcp/td-p/12874305); [CMS OSAI TCP](https://forums.autodesk.com/t5/hsm-post-processor-forum/5-axis-cms-osai-tcp/td-p/10192333)
- **ICAM CAM-POST** integrates with Fusion via APT-CL output, GENER and an Extractor, and also integrates with Mastercam and NX. — [ICAM Fusion kit](https://help.icam.com/270/english/cam/kit/fusion360/readme)
- **IMSpost** turns CL files into G/M codes for "all major CAD/CAM systems". v8.2 added transition management and robot controllers (Fanuc, KUKA, ABB). No Fusion-specific integration was found. — [IMS products](https://www.ims-software.com/products-services/); [DE247 IMSpost 8.2](https://www.digitalengineering247.com/article/ims-software-releases-imspost-8-2-cnc-postprocessing-software/products)
- ModuleWorks also sells a "Post Processor Framework" component. — [ModuleWorks homepage](https://www.moduleworks.com/)

### Inferences
- Controller-specific 5-axis concepts a post or plugin must handle are listed below. This is background knowledge; a source was not fetched in this session.
  - Fanuc: G43.4/G43.5 (TCP), G68.2/G53.1 (tilted work plane), G93 (inverse time).
  - Siemens 840D: TRAORI (5-axis transform), CYCLE800 (swivel plane).
  - Heidenhain: M128/FUNCTION TCPM, PLANE SPATIAL/PLANE VECTOR.
  - Haas: G234 (TCPC), G254 (DWO).
  - Post-level problems: rotary unwinding and wrap at ±360°, choosing between two IK solutions, retract-and-reconfigure on limit, and singularity flips near tool-axis ∥ rotary axis.
- For a **Fusion plugin**, the lowest-friction path is to emit toolpaths that Fusion's own post engine can consume, so existing .cps posts work. A custom post stack (ICAM, IMS or in-house) would only make sense for a standalone product.
- Mastercam's approach (MP/MP.NET posts, plus a separate machine definition and control definition) was not researched here.

### Gaps
- No official Autodesk document on singularity handling in posts was found.
- Mastercam post architecture was not verified.

## Q4. Core 5-axis toolpath algorithms a new implementation must solve

### Takeaway
The core problem set is documented in academic surveys, especially Lasemi, Xue & Gu, "Recent development in CNC machining of freeform surfaces: a state-of-the-art review" (Computer-Aided Design, 2010; citation confirmed only via a secondary reference list). It covers tool path topology, tool orientation, gouge/collision avoidance, and feedrate. Open implementations of full 5-axis are essentially absent; OpenCAMLib covers 3-axis drop/push-cutter primitives.

### Cited Findings
- The Lasemi/Xue/Gu review is cited in a 2018 Manufacturing Review article. — [Manufacturing Review 2018 refs](https://mfr.edp-open.org/articles/mfreview/ref/2018/01/mfreview170023/mfreview170023.html)
- OpenCAMLib provides drop-cutter and push-cutter (waterline) for cylindrical, ball, bull, cone and composite cutters, which are 3-axis primitives. — [OpenCAMLib docs](https://opencamlib.readthedocs.io/)
- Commercial physics-based feed optimization exists as a separate product category (VERICUT Force, MAL Inc. partnering with ModuleWorks). — [CGTech](https://cgtech.com/vericut-9-1); [DE247 ModuleWorks](https://www.digitalengineering247.com/company/moduleworks)

### Inferences
These are background domain knowledge and should be verified against the Lasemi review and the literature before citing.
- **Tool orientation and axis smoothing**: lead/tilt, interpolating tool-axis vectors along and across passes, and limiting angular acceleration in machine-axis space (not just workpiece space).
- **Gouge detection**: local (curvature) gouging and rear gouging; global collision of shank, holder and spindle against the part and fixtures. Methods include distance fields, BVH/OBB trees and GPU depth buffers.
- **Collision avoidance by tilting**: automatic tilt away from the contact normal, or toward a point or line, with "tilt only when needed" strategies.
- **Singularity avoidance**: near the rotary-axis pole (e.g., B≈0 on table-table AC), small vector changes cause large C swings. Remedies are pole-avoidance tilting, choosing the IK branch, and linearizing.
- **Linearization**: inserting points so the interpolated tool tip stays within chordal tolerance when rotary axes move. This is non-linear in the Cartesian path without TCP.
- **Feed rate optimization**: chip-thinning and engagement-based adaptive feed, and kinematic feed limits per axis.
- **Barrel and conical-barrel cutters**: tangent-plane/large-radius "barrel" finishing needs exact contact-point math for the circle-segment profile and orientation constraints.
- **Swarf (flank) milling**: ruled-surface approximation and minimizing deviation from non-developable surfaces.
- **Machine kinematics**: forward/inverse kinematics for head-head, head-table and table-table configurations; axis limits; rotary wrap.

### Gaps
- The Lasemi 2010 abstract/full text was not retrieved; the citation is confirmed only via a secondary reference list.
- No open-source 5-axis toolpath library of production quality was identified in this session.
