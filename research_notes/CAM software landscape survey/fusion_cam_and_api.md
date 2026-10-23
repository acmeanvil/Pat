# Autodesk Fusion CAM, Manufacturing Extension, and API/Add-in Ecosystem (as of Oct 2026)

Research note: autodesk.com product/blog pages returned HTTP 403 to direct fetches, so some Autodesk blog content below is taken from search-result excerpts rather than full-page reads. The capability matrix was read in full from the official Autodesk comparison PDF, which is mirrored by reseller CAD Studio. It is undated, but it lists capabilities consistent with 2024–2026. help.autodesk.com API pages were fetched directly.

## 1. Milling strategies by tier (Personal / Commercial Fusion / + Manufacturing Extension)

### Takeaway
Commercial Fusion covers 2D/2.5D milling, 3-axis milling, 3+1/3+2 positional milling (including 4th-axis wrapping of 2D toolpaths), machine simulation, and posts. Everything that is *simultaneous* 4/5-axis (Rotary, Swarf, Multi-Axis Contour, Multi-Axis Flow, 5-axis Steep & Shallow, Multi-Axis Clearing), multi-axis collision avoidance, the "automated" 3-axis finishing set (Steep & Shallow, Deburr, Geodesic, Corner), automatic hole recognition, toolpath editing, and advanced probing need the Manufacturing Extension. In 2026 Autodesk has been adding multi-axis roughing (Multi-Axis Clearing in January, 3+2 Clearing in July) and linking controls.

### Cited Findings
**Official capability matrix (Fusion vs Fusion + Manufacturing Extension)** — [Autodesk comparison chart PDF (CAD Studio mirror)](https://cadstudio.cz/dl/fusion-manufacturing-extension-comparison-matrix-en.pdf)
- Hole drilling (3-axis), with template-driven drilling, thread milling, tapping and boring: **in core Fusion**.
- Automatic hole recognition and drilling (3-axis and multi-axis), with feature recognition, machining templates and multi-axis drilling: **Extension only**.
- 2D and 2.5-axis milling (2D adaptive clearing, pocket, face, slot, contour): **core**.
- 3-axis milling ("3D adaptive clearing, flat, parallel, scallop, contour, flow machining, and more"): **core**. Note that the matrix lists 3-axis *Flow* as core.
- 3+1 and 3+2 positional milling ("4-axis wrapping of 2D toolpaths, align tool axis to view/surface normal, interactive tool axis tilt and turn"): **core**.
- Machine simulation ("digital twin", free library of 3D machine models, machine/toolpath animation): **core**.
- Automated entire-part machining ("3-axis steep and shallow machining, deburr, geodesic, corner finishing"): **Extension only**.
- 4- and 5-axis simultaneous milling ("4-axis rotary, 5-axis steep & shallow, swarf, multi-axis contour, multi-axis flow, and more"): **Extension only**.
- Multi-axis collision avoidance ("4- and 5-axis collision avoidance, advanced tool axis smoothing, tool axis tilting limits"), which avoids collisions by tilting the rotary axes: **Extension only**.
- Toolpath modifications (limit, split or delete sections by sketched polygon, editable modifications) and Replace Tool without recalculating: **Extension only**.
- 2-axis turning and turn-mill / mill-turn / live tooling: **core**.
- WCS setup probing and manual inspection: **core**. Geometry probing (tool-wear updates), surface inspection, and part alignment (3/4/5-axis best-fit from inspection): **Extension only**.
- Post-processing with a library of free, editable posts: **core**.
- Non-milling items that are Extension only: advanced and multi-sheet nesting, remnant cutting, DXF layer export, Metal Powder Bed Fusion, DED (3- and 5-axis additive toolpaths with simulation and collision checking), and MPBF process simulation.

**Other Autodesk statements on tiers and strategies**
- An Autodesk 2026 blog's comparison table lists these as Extension-only, not core: automatic hole recognition and drilling, automated part machining, 4- and 5-axis simultaneous milling, multi-axis collision avoidance, and toolpath modification — [Autodesk Fusion blog, "Your Machines Are Capable of More…" (search excerpt; page 403 on fetch)](https://www.autodesk.com/products/fusion-360/blog/advanced-manufacturing-autodesk-fusion-manufacturing-extension/)
- The Extension adds dedicated toolpaths "such as Swarf, multi-axis contour, multi-axis flow, and 4-axis Rotary", plus multi-axis options in Steep & Shallow. Tool-axis options include lead/lean and axis limits that keep NC within machine travel. Automatic collision avoidance tilts the holder away from the part — [Autodesk blog "All About the Fusion Manufacturing Extension" (2024; search excerpt)](https://www.autodesk.com/products/fusion-360/blog/autodesk-fusion-manufacturing-extension/)
- Steep & Shallow combines parallel, scallop and contour, using threshold angles and overlap. Geodesic produces passes held to a user scallop height. Swarf is flank milling with the full flute length. Corner finds material left by a larger tool — [Autodesk blog, "Unlocking the Full Potential of 3-Axis Machining with Fusion Manufacturing Extension"](https://www.autodesk.com/products/fusion-360/blog/unlocking-the-full-potential-of-3-axis-machining-with-fusion-manufacturing-extension/); [Autodesk blog, "So Much More Than 5-Axis Machining"](https://www.autodesk.com/products/fusion-360/blog/fusion-360-machining-extension-beyond-5-axis-machining/)
- Autodesk published a blog post titled "Fusion 360 Machining Extension: What Machining Strategies Are Coming Next?", which concerns ModuleWorks strategies. I could not read its content because the page returned 403 — [Autodesk blog](https://www.autodesk.com/products/fusion-360/blog/fusion-360-machining-extension-future-moduleworks-machining-strategies/)

**2026 additions (release notes and blog, via search excerpts)**
- **January 2026: Multi-Axis Clearing** (Extension). It roughs areas such as pockets with curved floors by offsetting from a floor surface, a ceiling surface, or a blend of the two. It offers Pocket or Adaptive styles, supports 3-, 4- and 5-axis work, and can machine undercuts — [January 2026 Product Update](https://www.autodesk.com/products/fusion-360/blog/january-2026-product-update-whats-new/); [Autodesk blog "5 New Fusion Features…"](https://www.autodesk.com/products/fusion-360/blog/autodesk-new-fusion-features/)
- **April 2026: box clearance geometry** for linking motion in multi-axis 3D toolpaths (Extension) — [April 2026 Product Update](https://www.autodesk.com/products/fusion-360/blog/april-2026-product-update-whats-new/)
- **July 2026: 3+2 Clearing** strategy. A single operation can use multiple fixed tool orientations, with Pocket, Adaptive and Parallel clearing and a "virtual fixture" to avoid workholding. The tier was not stated in the excerpt — [July 2026 Major Product Update](https://www.autodesk.com/products/fusion-360/blog/july-2026-major-product-update-whats-new/); [linecad summary](https://linecad.com/fusion-july-2026-update/)
- **September 2026 build (v.2705.1.11):** Polar Machining options in the Toolpath Operations dialog with optimized polar links, a 2-Axis method for Part Alignment, and "Straighten Cuts at Boundaries" for Geodesic (Extension) — [Fusion release notes (help.autodesk.com)](https://help.autodesk.com/view/fusion360/ENU/?guid=FIC-REL-NOTES-INC); [September 2026 top features blog](https://www.autodesk.com/products/fusion-360/blog/fusion-september-2026-update-top-features/)
- **October 2026 API:** a "MachiningPlan" (Preview) recognizes machinable features (hole groups, pockets) and produces planned toolpaths and process steps. This points to an automated-programming feature in the product — [Fusion API What's New](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/WhatsNew.htm)

### Inferences
- The core/Extension line falls between *positional* multi-axis (core) and *simultaneous* multi-axis plus automatic collision avoidance (Extension). A plugin offering simultaneous 4/5-axis would compete directly with a roughly $1,465/yr upsell. A plugin offering only 3+2 automation would compete with core features and the new 3+2 Clearing.
- Many strategy names from the question (Adaptive, Pocket, Contour, Parallel, Scallop, Pencil, Radial, Spiral, Morphed Spiral, Project, Ramp, Horizontal, Blend, Rest via stock/rest-material options) are well-known core 3-axis Fusion strategies. However, the matrix above is the only official tier source I read in full, and it says "and more". Treat per-strategy tiering for Blend, Morphed Spiral and Pencil as unverified here.

### Gaps
- I could not fetch the full Autodesk help strategy list (help.autodesk.com renders client-side, and blog pages returned 403). I did not confirm Blend, Morphed Spiral, Rest-machining specifics, or Deburr / Rotary parameter sets from primary pages.
- I did not confirm the license tier for 3+2 Clearing (July 2026).
- Whether the multi-axis strategies are ModuleWorks-based is suggested by Autodesk's blog title but not confirmed from page content.

## 2. Machine simulation, kinematics, machine library, verification, posts, tool library

### Takeaway
Core Fusion includes machine simulation with a free library of machine models and an editable post library. Kinematic machine definitions are also scriptable through the API (since April 2023). Tool and stock-material libraries have API access, and October 2026 added a PowerMill tool-library import.

### Cited Findings
- Machine simulation ("digital twin"), a free library of 3D machine models, and machine/toolpath animation are in core Fusion — [comparison PDF](https://cadstudio.cz/dl/fusion-manufacturing-extension-comparison-matrix-en.pdf)
- Post-processing uses a "library of free, editable post-processors" in core Fusion — [comparison PDF](https://cadstudio.cz/dl/fusion-manufacturing-extension-comparison-matrix-en.pdf)
- API machine model: `Machine.create(MachineInput)`, with `MachineFromFileInput` and `MachineFromLibraryInput`. Parts have unique ids and are built with `MachinePartInput`. `createAxisInput` produces a `LinearMachineAxisInput` (a unit direction vector in machine coordinates) or a `RotaryMachineAxisInput` (a rotation axis). `MachineAxis` has derived classes `LinearMachineAxis` and `RotaryMachineAxis`, with home values in cm and radians. All of these were introduced in April 2023 — [Machine.create](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/cam_Machine_create.htm); [MachinePartInput](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/cam_MachinePartInput.htm); [MachineAxis](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/cam_MachineAxis.htm); [RotaryMachineAxisInput](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/RotaryMachineAxisInput.htm)
- Tool libraries are accessed through the library manager's `ToolLibraries` object, which covers the Fusion library, vendor libraries, personal libraries and the document library. `DocumentToolLibrary` has `createEmpty` and `createFromJson` — [CAM Libraries guide](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/CAMLibraries_UM.htm)
- October 2026: `PowerMillImportedToolLibrary` (`importFromPowerMillProject`, `convertToToolLibrary`, diagnostics JSON and report). Stock material library support left preview, and `StockMaterial.hardnessUnit` (HRC/HRA/HB) was added — [API What's New](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/WhatsNew.htm)
- Vendor tool-library add-ins exist, such as the Sandvik Coromant CoroPlus Tool Library add-in, which recommends cutting data by machine and material — [Autodesk blog](https://www.autodesk.com/products/fusion-360/blog/sandvik-coromant-tool-library-add-in-fusion-360/)

### Inferences
- Because machine kinematics can be created and read through the API, a plugin could reuse Fusion's machine definition for its own inverse kinematics and collision checks instead of defining machines separately. The depth of read access (limits, offsets, holder/head geometry) still needs verifying against the reference.

### Gaps
- Fusion posts are JavaScript `.cps` files, with an online post library and the "Autodesk Post Processor" engine. This is well established, but I did not fetch a primary page in this session.
- I found no source that documents API access to stock simulation or verification results, such as gouge checks or the stock mesh.

## 3. Personal-use license restrictions on CAM

### Takeaway
The Personal (free, non-commercial) tier supports 2/2.5D and 3-axis milling, adaptive clearing and turning. It **excludes 3+2, 4/5-axis, mill-turn, probing, automatic tool change and rapid feed**. In practice, posted code converts rapids to feed moves and posts cannot include tool changes. The Extension is not available for Personal use.

### Cited Findings
- Autodesk's personal-use page lists personal CAM as "2 & 3-axis milling, adaptive clearing, turning, FFF additive, water jet/laser/plasma". The paid tier adds "3+2 milling, 4 & 5-axis milling, mill-turning, set-up probing, automatic tool change, and rapid feed" — [Autodesk Fusion for Personal Use page (search excerpt; regional page)](https://www-pt.autodesk.com/in/products/fusion-360/personal)
- Posted G-code headers on Personal state that rapid feed is reduced to the cutting feed, and that unrestricted rapids need a subscription. Tool changes are not supported in one post, so users split programs per tool. The open-source Batch Post add-in can optionally restore G0 rapids (described by its author as experimental) — [TimPaterson/Fusion360-Batch-Post (GitHub)](https://github.com/TimPaterson/Fusion360-Batch-Post); [V1 Engineering forum](https://forum.v1engineering.com/t/problem-with-fusion-and-z-feedrate/21487)
- A user describes the "only a single tool path at a time" limitation as the worst. This is an anecdote — [Sienci forum](https://forum.sienci.com/t/any-3d-carving-limitations-with-personal-use/26544)
- In an Autodesk community thread, a user summarized the free tier as "2.5-axis milling only". This conflicts with the official "2 & 3-axis" wording and is treated as unreliable — [Autodesk forum](https://forums.autodesk.com/t5/fusion-design-validate-document/specific-limitations-of-fusion-free-version/m-p/13194021)

### Inferences
- Hobbyists with 4th axes (a large maker segment) have no official 4-axis path on Personal. A plugin that generates and posts rotary toolpaths outside Fusion's posting pipeline would sidestep this. However, Autodesk's Personal-use terms (non-commercial use, and any terms on add-ins circumventing limits) need legal review.

### Gaps
- I did not find an official Autodesk knowledge-base article that enumerates every Personal CAM limit, such as the exact rules for per-operation versus per-setup posting or whether the API's `postProcess` is restricted on Personal.

## 4. Fusion API for CAM: capabilities, limits, workarounds

### Takeaway
The `adsk.cam` API can automate the **existing** Fusion CAM: create setups and operations from built-in strategy strings or templates, set parameters and geometry, assign tools, generate toolpaths asynchronously, post process, write setup sheets, and manage tool, stock and machine libraries. There is **no supported way to register a new toolpath strategy or inject externally computed CL data as a native operation**. Autodesk API experts have repeatedly confirmed that custom toolpaths are "not currently possible", and I found no 2025–2026 change to that. Third parties work around it with their own generation and their own G-code output, with Manual NC, or by driving built-in strategies (as Toolpath.com does).

### Cited Findings
- Object model: CAM → Setups → Operations / Folders / Patterns. Setups and operations use `createInput(...)` then `add(...)`. Operations take a **strategy string**, and the available strings come from `Operations.compatibleStrategies`, which "depend on license and extensions". An operation input can set a tool from the document tool library, select geometry, and set parameters. Most settings are CAM parameters (`parameters.itemByName(...).value/.expression`) — [CAM API Introduction](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/CAMIntroduction_UM.htm)
- `generateAllToolpaths(skipValid)` and `generateToolpath(op | ops | setup | setups)` are asynchronous and return a `GenerateToolpathFuture` — [CAM API Introduction](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/CAMIntroduction_UM.htm); [CAM.generateToolpath](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/cam_CAM_generateToolpath.htm)
- Posting: `postProcessAll(postInput)` and `postProcess(op/setup/collection, postInput)`. `PostProcessInput.create(programName, postConfig, outputFolder, units)` passes post properties by name. Setup sheets are generated with `generateAllSetupSheets` / `generateSetupSheet` (HTML; Excel is not supported on Mac) — [CAM API Introduction](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/CAMIntroduction_UM.htm)
- Sample "New Operation" script — [Fusion API sample](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/New_Operation_Sample_Sample.htm)
- September 2026: `CAMTemplate.isPrioritySet` / `priority`. October 2026: `MachiningPlan*` feature-recognition and planning API (Preview), plus the PowerMill tool import and stock materials covered above. The October 2026 release also fixed add-ins being unable to add marking-menu entries under CAM dropdowns such as New Operation — [API What's New (Oct 2026)](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/WhatsNew.htm)
- Brian Ekins (Autodesk API lead) confirmed that adding custom toolpaths is not possible. A September 2020 update added creating operations from templates — [Autodesk forum "Ability to create custom toolpaths"](https://forums.autodesk.com/t5/fusion-api-and-scripts-forum/ability-to-create-custom-toolpaths/td-p/10306786); [Autodesk forum "generating toolpath from DXF layer"](https://forums.autodesk.com/t5/fusion-api-and-scripts-forum/idea-generating-toolpath-from-dxf-layer-depending-on-their-name/m-p/9761502) (older, around 2020–2021)
- Ekins also described the Manufacture API as "very limited" at the time and said a full STL-to-G-code scripted flow was not possible (older) — [Autodesk forum](https://forums.autodesk.com/t5/fusion-api-and-scripts-forum/how-can-i-script-the-process-of-generating-g-code-toolpath-in/m-p/11484415/highlight/true)
- A community thread, "Manufacturing (CAM) API Feedback", collects user requests. Its content could not be fetched (403) — [Autodesk forum](https://forums.autodesk.com/t5/fusion-api-and-scripts-forum/manufacturing-cam-api-feedback/td-p/11869814)
- Cloud **Fusion Automation API (Design Automation for Fusion)**: open beta in February 2025 and generally available in May 2025. It runs Fusion API scripts in the cloud (TypeScript), exposes "7,000+" endpoints including subtractive CAM, and costs 3.0 Flex tokens per processing hour — [APS blog, Feb 2025](https://aps.autodesk.com/blog/design-automation-fusion-open-beta); [APS blog, May 2025 GA](https://aps.autodesk.com/blog/design-automation-api-fusion-now-generally-available)
- Workaround, Manual NC pass-through: Fusion "Manual NC" operations can carry labels and comments that downstream systems (for example CAMplete TruePath) map to custom G-code inserts — [Autodesk CAMplete TruePath help](https://help.autodesk.com/cloudhelp/2023/ENU/CAMTP-Fusion360/files/TPFusionManualNCtopicid.html)
- Workaround, external planning with native operations: Toolpath.com's add-in imports setups and operations that Toolpath generated into Fusion, where "they can be edited like any other Fusion program". In other words, it drives built-in strategies rather than injecting raw paths — [Toolpath docs](https://docs.toolpath.com/cam-automation/how-to-export-machining-strategies-to-fusion)
- An academic template-driven add-in (Iowa State) automates setups and operations from templates. It does not choose orientation or fixturing automatically — [Iowa State IMSE](https://imse.iastate.edu/jdham/?p=377)

### Inferences
- Options for a new 3/4/5-axis plugin:
  1. **Orchestrator**: drive native strategies (including Extension ones, when the user is licensed) with smarter automation, such as orientation selection, templates and MachiningPlan.
  2. **Parallel engine**: read BRep or mesh geometry and the machine definition through the API, compute toolpaths externally, show them as custom graphics or sketches, and post with its own post engine (or reuse `.cps` logic outside Fusion). Paths would not appear in Fusion's native simulation or NC Program unless pushed through workarounds such as Manual NC, or sketch geometry driving a native Trace/2D Contour/Project.
  3. **Geometry-driven**: generate drive curves or surfaces and feed them to native Swarf, Flow or Project strategies.
- Because `compatibleStrategies` is gated by license, an orchestrator plugin's value depends on the user's tier. A parallel-engine plugin would work on core Fusion, and possibly Personal, but gives up integration with native simulation.

### Gaps
- I found no documentation of API read access to computed toolpath moves (CL points or tool vectors) for an operation. If it exists, it would enable external verification or a custom post. Verify against the full `adsk.cam` reference.
- I found no confirmation of whether the API exposes stock simulation, collision-check results or machine simulation playback.
- I did not confirm whether Manual NC can carry bulk custom motion (for example, a large block of G-code passed through to the post).

## 5. Third-party Fusion CAM add-ins and apps

### Takeaway
The ecosystem is mostly peripheral: feeds and speeds, tool libraries, machine and verification connectors, batch posting, and CAM automation. I found no third-party add-in that adds new simultaneous multi-axis toolpath strategies inside Fusion, which is consistent with the API limits above.

### Cited Findings
- **Cam Optimization** (Automated Viking LLC) calculates speed, feed, stepdown and stepover per operation. It is v1.9.6 and uses a custom installer — [Autodesk App Store listing](https://marketplace.autodesk.com/apps/744bf773-4aae-4f4f-b189-7ab6318cbb42)
- **Sandvik Coromant CoroPlus Tool Library** add-in provides tools plus recommended cutting data — [Autodesk blog](https://www.autodesk.com/products/fusion-360/blog/sandvik-coromant-tool-library-add-in-fusion-360/)
- **Feeds and Speeds – MachiningCloud** is listed on the Autodesk App Store and draws on the MachiningCloud database — [Autodesk App Store search](https://apps.autodesk.com/FUSION/pw/List/Search?page=11)
- **Mazak Add-in** exports NC, part, fixture and stock geometry and tool/holder geometry to Mazak Smooth Project Manager for verification — [Autodesk blog](https://www.autodesk.com/products/fusion-360/blog/mazak-add-in-fusion-360/)
- **CAMplete TruePath** (Autodesk) is a Fusion interface for post, verification and simulation, with Manual NC labels — [Autodesk help](https://help.autodesk.com/cloudhelp/2023/ENU/CAMTP-Fusion360/files/TPFusionManualNCtopicid.html)
- **Toolpath** (toolpath.com) is a cloud CAM-automation tool whose add-in imports generated setups and operations — [Toolpath docs](https://docs.toolpath.com/cam-automation/how-to-install-the-toolpath-add-in-for-fusion)
- **Fusion360 Batch Post** (open source) posts all operations and optionally restores rapids on Personal — [GitHub](https://github.com/TimPaterson/Fusion360-Batch-Post)
- **Fusion + PowerMill** is sold by Autodesk partners for heavy 5-axis surfacing. This is vendor marketing — [Autodesk partner success story](https://www.autodesk.com/support/partners/success-stories/optimization-of-multiaxis-production/7605)

### Gaps
- I did not confirm HSMAdvisor or FSWizard integrations, Vericut-Fusion interface specifics, or any ModuleWorks-based third-party add-in from sources in this session.

## 6. Common user complaints and gaps in Fusion multi-axis CAM

### Takeaway
Autodesk itself describes Fusion as strong at 5-axis positional work and swarf trimming, with limits in simultaneous 5-axis surfacing. Recurring user complaints cover rotary-axis winding and unwinding (retract and rewind behavior), the learning curve for tool-axis control, and the steep Personal-tier limits. Most forum evidence found here is from older threads (2017–2019), so it may predate recent improvements.

### Cited Findings
- An Autodesk product manager said Fusion is "very good at 5 Axis positional and live 5-Axis for trimming (Swarf toolpath)", but that users doing 5-axis surfacing "may see the limits of Fusion 360" (older thread, around 2017–2018) — [Autodesk forum "5 Axis Simultaneous"](https://forums.autodesk.com/t5/fusion-manufacture-forum/5-axis-simultaneous/m-p/7219558)
- Users on trunnion machines (Haas VF6 with trunnion) report the B axis winding, then unwinding on later passes so that the tool buries into the part. No resolution was shown (older) — [Autodesk forum "5-axis gurus toolpath example request"](https://forums.autodesk.com/t5/fusion-manufacture-forum/re-5-axis-gurus-toolpath-example-request/m-p/8802224)
- A user said they were "having quite a hard time wrapping my head around 5 axis in fusion" — [Autodesk forum](https://forums.autodesk.com/t5/fusion-manufacture/5-axis-simultaneous/m-p/7918201)
- An Autodesk partner case study cites long calculation times and difficult 5-axis programming in a prior CAM system, and promotes Fusion + PowerMill. This is marketing — [Autodesk partner story](https://www.autodesk.com/support/partners/success-stories/optimization-of-multiaxis-production/7605)
- Personal-tier complaints: rapids slowed to feed, and no tool changes or single-tool posting — [Batch Post README](https://github.com/TimPaterson/Fusion360-Batch-Post); [Sienci forum](https://forum.sienci.com/t/any-3d-carving-limitations-with-personal-use/26544)

### Inferences
- The 2026 releases (Multi-Axis Clearing, 3+2 Clearing, box-clearance linking, polar machining links) target exactly the historical gaps in multi-axis roughing and linking. A plugin should not assume those gaps remain open.
- Likely remaining gaps that a plugin could address: high-end simultaneous 5-axis surface finishing comparable to hyperMILL or PowerMill (Autodesk partly answers this by pairing Fusion with PowerMill), impeller and blade or port modules, and full G-code-level machine simulation. These are suggested by the sources but not verified in 2026 user threads.

### Gaps
- No Reddit r/Fusion360, Practical Machinist or CNCzone threads from 2025–2026 surfaced in searches. Current (2026) user sentiment on multi-axis quality is unverified.

## 7. Pricing (2026)

### Takeaway
Core Fusion is about **$680/yr** (US). The Manufacturing Extension is about **$1,465/yr** as an add-on, reduced from $1,600 in 2024. The "Fusion for Manufacturing" bundle (core plus Extension) lists at **$2,040/yr** and is often promoted at about $1,367. Personal use is free and non-commercial. The Automation API is billed in Flex tokens.

### Cited Findings
- Autodesk's multi-seat offer page lists Fusion at $680 per subscription per year. Fusion for Manufacturing lists at $2,040, discounted to $1,367 per year, and "Includes all Fusion features, plus advanced manufacturing capabilities" — [Autodesk multi-seat offer](https://www.autodesk.com/campaigns/fusion-360/multi-seat-offer)
- A 4-for-3 offer runs from 2/1/2026 to 1/31/2027 and gives 25% off in blocks of four commercial seats — [Autodesk 4-for-3 offer](https://www.autodesk.com/campaigns/fusion-360/4-for-3-offer)
- In 2024, extensions priced at $1,600 SRP were cut to $1,465 SRP — [Autodesk blog "How much will Fusion 360 cost"](https://www.autodesk.com/products/fusion-360/blog/how-much-will-fusion-360-cost/). A 2026 secondary guide also cites $1,465 per year — [codegenes guide](https://www.codegenes.net/blog/autodesk-fusion-download/)
- A Fusion subscription is required before buying the Manufacturing Extension — [Autodesk blog (search excerpt)](https://www.autodesk.com/products/fusion-360/blog/advanced-manufacturing-autodesk-fusion-manufacturing-extension/)
- Older terms allowed enabling the Extension for 125 cloud credits per month per user. These are likely outdated — [Machinery.co.uk (older)](https://www.machinery.co.uk/machinery-products/latest-autodesk-fusion-360-has-manufacturing-extension-and-cost-insights-for-generative-design)
- The Automation API costs 3.0 Flex tokens per processing hour — [APS blog May 2025](https://aps.autodesk.com/blog/design-automation-api-fusion-now-generally-available)

### Gaps
- I could not fetch a live Autodesk store page to confirm the October 2026 US list price of the standalone Extension. Prices vary by region and promotion.
