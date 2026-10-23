# Commercial CAM Systems for 3/4/5-Axis Milling (as of Oct 2026)

> Method note: about 17 web searches and fetches, run in October 2026. Items tagged **[BK]** come from the researcher's background knowledge (training data up to about mid-2026). They are widely known product facts, but I did **not** re-verify them in this session, so treat them as "likely true, verify before quoting". Every untagged claim has an inline source. Where a source is a vendor press release, it is marked (vendor).

## Q1. Ownership, deployment, and product status (2023–2026 changes)

### Takeaway
The main CAM players in 2026 group under four owners: **Sandvik** (Mastercam, plus Cimatron and GibbsCAM through Cambrio), **Hexagon Manufacturing Intelligence** (ESPRIT EDGE, Edgecam, WorkNC, VISI, NCSIMUL), **Autodesk** (PowerMill and FeatureCAM; Inventor CAM and HSMWorks are being wound down in favour of Fusion), and **Siemens** (NX CAM). The rest are independents: OPEN MIND/hyperMILL, SolidCAM, Tebis, Missler/TopSolid, HCL/CAMWorks, SprutCAM, BobCAD. The brief has two premises wrong. CAMWorks did **not** become Cambrio. And the Hexagon "Octave" spin-off did **not** take the CAM products.

### Cited Findings
- **Mastercam / CNC Software**: Sandvik completed the acquisition on **30 Sep 2021**, not 2023. At the time CNC Software had about USD 60M revenue (2020), a 25–30% EBIT margin, about 60% recurring revenue, and about 270,000 installed licenses. It was reported in Sandvik's Design & Planning Automation division. — [Sandvik/Nasdaq PR](https://www.nasdaq.com/press-release/sandvik-completes-the-acquisition-of-leading-cam-software-company-cnc-software-inc.); [Sandvik news](https://www.home.sandvik/en/news-and-media/news/2021/09/sandvik-completes-the-acquisition-of-leading-cam-software-company-cnc-software-inc.-creators-of-mastercam)
- Mastercam has since been buying its own resellers. It bought FASTech (Ohio/Kentucky reseller) as an asset deal reported in January 2025, and in January 2026 renamed AME AB to Mastercam Sverige after buying it. — [DE247](https://www.digitalengineering247.com/article/mastercam-acquires-fastech/news); [DE247](https://www.digitalengineering247.com/article/mastercam-grows-worldwide-footprint-via-swedish-acquisition/news)
- Mastercam now claims "over 450,000 installations" and is "part of Sandvik Group". — [Mastercam 2026.R2 PR via Bolsamania](https://www.bolsamania.com/nota-de-prensa/mercados/mastercam-2026r2-delivers-gpu-accelerated-simulation-and-ai-powered-assistance--21635723.html) (vendor)
- **Cambrio** is the Ohio-based company that held **GibbsCAM, Cimatron and SigmaNEST**. Sandvik agreed to buy it from Battery Ventures in **July 2021**; it had about 375 employees. Cambrio is *not* a rename of CAMWorks. — [MMS](https://mmsonline.com/news/sandvik-acquiring-cadcam-company-cambrio); [Cimatron news](https://www.cimatron.com/news/sandvik-to-acquire-leading-cam-software-company-cambrio); [SME](https://www.sme.org/technologies/articles/2021/july/sandvik-to-acquire-cam-software-company-cambrio/)
- **Product positioning:** GibbsCAM covers production milling and turning, 2D to 5-axis milling, multi-task (mill-turn) machining and wire EDM. Cimatron is an integrated CAD/CAM system aimed at mold and die toolmakers. — [Cimatron/Cambrio PR](https://www.cimatron.com/news/sandvik-to-acquire-leading-cam-software-company-cambrio)
- **CAMWorks** is still owned by **HCLSoftware** (part of HCLTech) and is now branded "HCL CAMWorks". There was **no 2025 ownership change**. CAMWorks 2025 added a CAMWorks for 3DEXPERIENCE SOLIDWORKS edition, and CAMWorks 2026 (Jan 2026) added Swiss machining. — [HCL news](https://www.hcl-software.com/news/23-1-26-hclsoftware-releases-swiss-machining-in-hcl-camworks-2026); [DE247](https://www.digitalengineering247.com/article/camworks-for-3dexperience-solidworks-launched-with-camworks-2025); [SOLIDWORKS partner page](https://www.solidworks.com/partner-product/camworks)
- **Hexagon / Octave:** Octave was formed from Hexagon's Asset Lifecycle Intelligence and Safety, Infrastructure & Geospatial divisions, plus ETQ (moved out of Manufacturing Intelligence) and Bricsys. Hexagon's page says Octave became an independent public company in **May 2026**. CAM products are not listed as part of Octave. — [Hexagon PR](https://hexagon.com/company/newsroom/press-releases/2025/octave-unveiledhexagon-reveals-name-for-potential-2026-company-spin-off); [Geo Week News](https://www.geoweeknews.com/articles/hexagon-spin-off-to-be-called-octave/)
- **ESPRIT** was bought by Hexagon in 2020 and still ships as **ESPRIT EDGE** under Hexagon. It has four releases planned for 2026. The first (April 2026) introduced **ProPlanAI** for turning, mill-turn and Swiss programming. — [Hexagon PR Apr 2026](https://hexagon.com/company/newsroom/press-releases/2026/hexagon-introduces-new-esprit-edge-capabilities-to-reduce-cnc-programming-effort-and-support-high-complexity-machining); [Schnitger 2020](https://schnitgercorp.com/2020/10/29/quickie-hexagon-snaps-up-esprit-cam-solutions/)
- Hexagon's Production Software portfolio includes VISI, WORKNC, WORKPLAN, EDGECAM, NCSIMUL and others. (This source is from 2022 and may be stale.) — [Interempresas 2022](https://www.interempresas.net/Informatica_Industrial/371487-Hexagon-PS-presentara-en-MetalMadrid-el-nuevo-software-ESPRIT-y-las-actualizaciones-2022.html)
- **Autodesk HSMWorks** (the CAM embedded in SolidWorks) was removed from new commercial Fusion subscriptions on **25 Mar 2025**. It reaches end of life on **25 Mar 2028**. — [Autodesk HSMWorks page](https://autodesk.com/products/hsmworks/overview)
- **Inventor CAM Express and HSMXpress** (the free plugins) are no longer in development; Inventor CAM Express works only with Inventor 2021 or earlier. **Inventor CAM** (the paid version) is still listed inside the Product Design & Manufacturing Collection. — [Autodesk Inventor CAM page](https://www.autodesk.com/products/inventor-cam/free-download)
- **CAMplete** (Autodesk): according to a reposted notice on a forum, end of sale is 7 Jun 2026, end of renewal 7 Jun 2028 and end of life 7 Jun 2029. I could not verify this on an official page. — [mastercamforums](https://mastercamforums.com/forum/index.php?msg=111194); [Autodesk CAMplete page](https://www.autodesk.com/ca-en/products/camplete)
- **FeatureCAM:** FeatureCAM 2025 was released in 2024. I found no official retirement notice. — [Autodesk forum](https://forums.autodesk.com/t5/featurecam-forum/featurecam-2025/td-p/12759892)
- **PowerMill 2026** has been released. Forum users describe it as mostly bug fixes and stability work ("only bug fixes... nothing new"; "runs quicker... so stable now"). — [Autodesk forum](https://forums.autodesk.com/t5/powermill-forum/powermill-2026/m-p/13886787)
- **Siemens NX CAM** ships on a half-yearly cadence (release 2606 is June 2026). Siemens now sells CAM-only "NX X Manufacturing" packages without feature or solid modeling. — [Siemens NX Mfg blog](https://blogs.sw.siemens.com/nx-manufacturing/nx-x-manufacturing-cam/); [Janus Engineering](https://blog.janus-engineering.com/en_en/details/article/siemens-nx-release-2606/)

### Inferences
- **Autodesk is consolidating onto Fusion.** HSMWorks is set to retire, CAMplete reportedly so, the free Inventor CAM plugins are frozen, and PowerMill 2026 is described by users as a maintenance release. Shops on Inventor CAM, HSMWorks or FeatureCAM are therefore likely to be in migration over 2025–2028. That group is an obvious target market for a Fusion plugin that adds high-end 5-axis features.
- **Sandvik is now the biggest single CAM owner**, holding Mastercam, GibbsCAM and Cimatron. Mastercam's habit of buying its resellers suggests it is pulling its sales channel in-house.

### Gaps
- I could not confirm whether Sandvik has merged GibbsCAM or Cimatron into Mastercam, or retired the Cambrio name, after 2023. The search results were mostly 2021 acquisition coverage.
- I did not verify current 2026 ownership of Edgecam or WorkNC (they are presumably still Hexagon MI), nor any change in Hexagon MI's own structure after Octave.
- Autodesk's retired-products page returned HTTP 403, so I could not check official FeatureCAM or Inventor CAM end-of-sale dates.
- I did not research BobCAD-CAM, OneCNC, ZW3D CAM, Creo NC, CATIA/DELMIA or TopSolid'Cam ownership or news in this session; only background knowledge is available (see the table).

---

## Q2. Product-by-product comparison: capabilities, simultaneous 5-axis leaders, and kernel (ModuleWorks vs in-house)

### Takeaway
Simultaneous 5-axis is led by **hyperMILL, NX CAM, PowerMill and Tebis**, which all have in-house 5-axis engines with strong automatic tilting and collision avoidance, and by **Mastercam** (native multiaxis plus a ModuleWorks-derived suite). SolidCAM, SprutCAM, GibbsCAM, CAMWorks, BobCAD and most mid-market systems get their simultaneous 5-axis from the **ModuleWorks** components. ModuleWorks says more than 60% of leading CAM vendors use its 5-axis component. Recent releases from the leaders concentrate on automatic tool orientation (mixing simultaneous and indexed moves), shank-clearance control, 5-axis rest machining, and deburring.

### Comparison table

Legend: S = standalone; I = integrated (host named); MW = ModuleWorks-based 5-axis; IH = in-house 5-axis. [BK] = background knowledge, not re-verified in this session.

| Product | Owner (2026) | Deployment | Target market | 5-axis kernel | Signature 5-axis / automation (cited where possible) | Pricing tier |
|---|---|---|---|---|---|---|
| **Mastercam** (+ Mastercam for SOLIDWORKS) | Sandvik (CNC Software) | S; I (SOLIDWORKS) | Broad job shop to aerospace; largest installed base | Native multiaxis + MW-based suite (forum claim, [eMastercam](https://www.emastercam.com/forums/topic/91770-mastercam-and-gibbscam-use-the-same-5-axis-engine/)) | 2026.R2: auto tilting with bullnose tools, 4-axis swarf improvements, automatic linking clearance, GPU-accelerated simulation, Mastercam Copilot (beta) ([PR](https://www.bolsamania.com/nota-de-prensa/mercados/mastercam-2026r2-delivers-gpu-accelerated-simulation-and-ai-powered-assistance--21635723.html)). [BK] Dynamic Motion 3D HSM, Accelerated Finishing (barrel tools), Port/Blade Expert, Unified multiaxis toolpath | Modular by axis count, quote-only ([Accio](https://www.accio.com/business/mastercam-2023-price)); [BK] about $10k–$40k+ perpetual plus maintenance |
| **hyperMILL** (+ for SOLIDWORKS / Autodesk Inventor; hyperCAD-S) | OPEN MIND (Mensch und Maschine group [BK]) | S (hyperCAD-S); I (SW, Inventor) | Mold/die, aerospace, medical, high-end 5-axis | IH | 2025: 5-axis auto tool orientation (pre-analysis mixing simultaneous and indexed), 5-axis hole deburring with auto collision avoidance ([newequipment](https://newequipment.com/home/product/55286315/open-mind-technologies-usa-inc-open-mind-technologies-hypermill-2025-cam-software-enhances-5-axis-toolpath-efficiency)). 2026: rebuilt 5-axis rest machining (Z-level/parallel/normal), "Minimum Clearance Angle" for shank, auto-sequenced deburring, VIRTUAL Machining digital twin, angle-head support ([OPEN MIND 2026](https://www.openmind-tech.com/en/cam/hypermill-2026/); [metal-am](https://www.metal-am.com/open-mind-launches-hypermill-2026-with-new-cam-features/)). [BK] MAXX Machining (barrel/conical tools, tangent plane), impeller/blisk, turbine blade, tube, port packages, AUTOMATION Center | Quote-only; "high cost" noted by G2 reviewer ([G2](https://www.g2.com/products/hypermill/pricing)) |
| **PowerMill** | Autodesk | S | Mold/die, complex 5-axis, robotics, high-speed | IH (Delcam heritage) | Automatic Tool Tilting (since 2019), tool-axis editing in a region ([MoldMaking Tech](https://www.moldmakingtechnology.com/products/new-feature-on-cam-software-simplifies-five-axis-programming); [Machinery](https://www.machinery.co.uk/content/product-launches/powermill-8-offers-5-axis-developments-and-more)). 2026 is mostly a stability release ([forum](https://forums.autodesk.com/t5/powermill-forum/powermill-2026/m-p/13886787)). [BK] Vortex, blade/blisk/port modules, macro automation | [BK] subscription, Standard/Ultimate tiers |
| **FeatureCAM** | Autodesk | S | Feature-based production | IH (Delcam) | Feature recognition / automation-first. 2025 release confirmed ([forum](https://forums.autodesk.com/t5/featurecam-forum/featurecam-2025/td-p/12759892)) | Subscription [BK]; future uncertain |
| **Inventor CAM / HSMWorks** | Autodesk | I (Inventor / SW) | Inventor & SW users | Fusion/HSM kernel | HSMWorks EOL 25 Mar 2028; free Express versions frozen ([Autodesk](https://autodesk.com/products/hsmworks/overview); [Autodesk](https://www.autodesk.com/products/inventor-cam/free-download)) | Bundled in PDMC |
| **NX CAM** | Siemens | I (NX); CAM-only NX X Manufacturing packages | Aerospace, auto, large OEMs | IH | 2606: AI Make Machining Suggestion now adds a third, generative-AI suggestion per feature, auto-creates parametric tools, plus background toolpath generation and fixture automation ([Siemens blog](https://blogs.sw.siemens.com/nx-manufacturing/?p=10125); [Janus](https://blog.janus-engineering.com/en_en/details/article/siemens-nx-release-2606/)). NX CAM Copilot claims "80% or more" time saving, starting with 2.5/3-axis ([Siemens blog](https://blogs.sw.siemens.com/nx-manufacturing/nx-x-manufacturing-cam/)). 5-axis adaptive milling and guiding curves added in 2020 ([DE247](https://www.digitalengineering247.com/article/siemens-nx-cam-unveils-updated-capabilities-in-new-release)). [BK] Turbomachinery Milling (blisk/impeller), feature-based machining (FBM), Integrated Simulation & Verification (ISV) with G-code-driven simulation, Post Builder | G2 lists "from $220/month" ([G2](https://www.g2.com/compare/mastercam-vs-solidcam-vs-hypermill-vs-nx-cam)); enterprise-tier in practice [BK] |
| **CATIA / DELMIA Machining** | Dassault Systèmes | I (3DEXPERIENCE) | Aerospace/automotive OEM | IH [BK] | [BK] Multi-Axis Surface Machining, prismatic machining, machining-process templates, DELMIA simulation | Enterprise [BK] |
| **SOLIDWORKS CAM** | Dassault (CAMWorks engine licensed from HCL) [BK] | I (SOLIDWORKS) | Entry-level SW users, 2.5/3-axis | n/a (3-axis) | [BK] Standard/Professional; knowledge-based rules, feature recognition (AFR); no simultaneous 5-axis | Std included with SW subscription [BK] |
| **HCL CAMWorks** | HCLSoftware | I (SOLIDWORKS, 3DEXPERIENCE SW, Solid Edge) | SW-centric job shops | MW for multiaxis [BK] | Edition for 3DEXPERIENCE SW (2025); Swiss (2026) ([HCL](https://www.hcl-software.com/news/23-1-26-hclsoftware-releases-swiss-machining-in-hcl-camworks-2026)); "enhanced five-axis" in 2026 ([MMS](https://www.mmsonline.com/products/software-offers-enhanced-five-axis-capabilities)). [BK] AFR, TechDB knowledge base, VoluMill | Mid [BK] |
| **SolidCAM** (incl. iMachining) | SolidCAM Ltd (independent) | I (SOLIDWORKS, Inventor, Solid Edge [BK]) | Job shops, production | MW (partnership noted by [ModuleWorks](https://www.moduleworks.com/software-components/toolpath/)) | iMachining with claimed ≥70% cycle-time cut and ≥5× tool life (vendor); simultaneous 5-axis: barrel, SWARF, port, multi-blade ([MACH 2026](https://machexhibition.com/news/solidcam-uk-to-showcase-the-future-of-integrated-cam-at-mach-2026/)). 2026: feature-driven levels for iMachining 2D, multi-step chamfers ([Solid Solutions](https://www.solidsolutions.co.uk/whats-new-in-solidcam-2026/)) | Quote-only ([G2](https://www.g2.com/compare/mastercam-vs-solidcam-vs-hypermill-vs-nx-cam)); mid [BK] |
| **ESPRIT EDGE** | Hexagon MI | S | Mill-turn, Swiss, production | IH + MW [BK] | ProPlanAI (turning, mill-turn, Swiss), four releases a year in 2026 ([Hexagon](https://hexagon.com/company/newsroom/press-releases/2026/hexagon-introduces-new-esprit-edge-capabilities-to-reduce-cnc-programming-effort-and-support-high-complexity-machining)). [BK] Machine-builder-certified posts, digital-twin simulation | Mid-high [BK] |
| **Edgecam** | Hexagon MI | S; SW/Inventor interop [BK] | Production milling/turning | MW-based 5-axis [BK] | [BK] Waveform roughing, Strategy Manager automation, feature finder | Mid [BK] |
| **WorkNC** | Hexagon MI | S | Mold/die, automatic 3-axis | IH [BK] | [BK] Highly automated 3-axis and 5-axis-from-3-axis conversion, Auto 5 | Mid-high [BK] |
| **Cimatron** | Sandvik (Cambrio) | S CAD/CAM | Mold & die toolmakers ([Cimatron](https://www.cimatron.com/news/sandvik-to-acquire-leading-cam-software-company-cambrio)) | MW [BK] | [BK] Electrode, mold-specific 2.5–5-axis | Mid [BK] |
| **GibbsCAM** | Sandvik (Cambrio) | S | Production milling, MTM | MW (forum: same 5x engine as Mastercam, [eMastercam](https://www.emastercam.com/forums/topic/91770-mastercam-and-gibbscam-use-the-same-5-axis-engine/)) | 2D–5-axis, MTM, wire EDM ([Cimatron/Cambrio](https://www.cimatron.com/news/sandvik-to-acquire-leading-cam-software-company-cambrio)) | Mid [BK] |
| **Tebis** | Tebis AG (independent) | S CAD/CAM | Automotive tooling, large molds/dies | IH | SmartOps end-to-end automation, templates, curve-driven tool orientation (4.1 R10); 3-axis to 5-axis simultaneous roughing conversion in "Simultaneous avoidance" mode (R5); 5-axis adaptive roughing and collision avoidance (R7) ([Tebis](https://www.tebis.com/en/software/products/tebis-4.1-release-10); [Composites UK](https://compositesuk.co.uk/?p=5127)) | High [BK] |
| **TopSolid'Cam** | Missler Software | I (TopSolid CAD) | Integrated CAD/CAM/PDM shops | IH + MW [BK] | [BK] 5-axis, mill-turn, tool management | Mid-high [BK] |
| **Creo NC / Creo Mold Machining** | PTC | I (Creo) | Creo OEMs | MW for HSM/5x [BK] | [BK] Complete Machining extension, ModuleWorks-based HSM | Add-on [BK] |
| **ZW3D CAM** | ZWSOFT | I (ZW3D) | Budget CAD/CAM | MW [BK] | [BK] 2–5-axis | Low [BK] |
| **SprutCAM X** | SprutCAM Tech (Cyprus) | S | Small shops, robotics | MW (partnership from 2011, [ModuleWorks](https://www.moduleworks.com/2011/06/)) | Base license covers 3+2; continuous 4/5-axis is an add-on option ([Fitgap](https://us.fitgap.com/products/sprutcam)); robotics and P3D slicer ([SprutCAM](https://sprutcam.com/powered-by-sprutcam/p3d-slicer-for-robots-and-5-axis-machines/)) | About $249/month, plus about $100/month for continuous 4/5-axis ([Fitgap](https://us.fitgap.com/products/sprutcam)); €1,000 one-time on Capterra (conflicting) ([Capterra](https://www.capterra.com/p/163681/SprutCAM/)) |
| **BobCAD-CAM** | BobCAD-CAM Inc. | S; I (SOLIDWORKS) | Hobby to small job shop | MW [BK] | [BK] 4/5-axis add-ons | Low [BK]; no source found |
| **OneCNC** | OneCNC (AU) | S | Small shops | [BK] | [BK] | Low [BK] |

### Cited Findings (beyond the table)
- ModuleWorks says its 5-axis component is used by more than 60% of the world's leading CAM vendors (vendor claim, licensees not named). — [ModuleWorks](https://www.moduleworks.com/software-components/toolpath/)
- A forum post says Mastercam added a ModuleWorks-based 5-axis suite alongside its older native toolpaths, and that Mastercam and GibbsCAM share the same 5-axis engine (unofficial). — [eMastercam](https://www.emastercam.com/forums/topic/91770-mastercam-and-gibbscam-use-the-same-5-axis-engine/)
- hyperMILL 2025's automatic tool orientation pre-analyzes the toolpath and mixes simultaneous and indexed moves to cut manual 5-axis input. — [newequipment.com](https://newequipment.com/home/product/55286315/open-mind-technologies-usa-inc-open-mind-technologies-hypermill-2025-cam-software-enhances-5-axis-toolpath-efficiency) (vendor)
- In 2026, Mastercam and hyperMILL both shipped automatic tilting / shank-clearance improvements: Mastercam with auto tilting for bullnose tools, hyperMILL with Minimum Clearance Angle. — [Mastercam PR](https://www.bolsamania.com/nota-de-prensa/mercados/mastercam-2026r2-delivers-gpu-accelerated-simulation-and-ai-powered-assistance--21635723.html); [OPEN MIND](https://www.openmind-tech.com/en/cam/hypermill-2026/)

### Inferences
- In 2026 the competitive frontier for 5-axis is **automatic tool-axis management**: auto-tilt from a 3-axis definition, mixed indexed/simultaneous output, shank and holder clearance angles, and 5-axis rest and deburring. It is no longer new surface strategies. A Fusion plugin probably needs a strong "3-axis toolpath → automatic collision-free 5-axis" capability plus tool-axis controls to compete. The standard controls are lead/lag, tilt toward point/line/curve, normal to drive surface, interpolation between vectors, and smoothing; these are [BK] industry-standard and not verified here per vendor.
- Since ModuleWorks powers most mid-market 5-axis, a plugin could differentiate either by licensing MW (which gives parity with SolidCAM, SprutCAM and others) or by building in-house where the leaders are (hyperMILL, NX, PowerMill, Tebis).
- Barrel/circle-segment tool support has become table stakes at the top end: Mastercam Accelerated Finishing [BK], hyperMILL MAXX [BK], and SolidCAM "barrel cutting" ([MACH 2026](https://machexhibition.com/news/solidcam-uk-to-showcase-the-future-of-integrated-cam-at-mach-2026/)).

### Gaps
- I found no authoritative list of ModuleWorks licensees. The MW/IH labels in the table for ESPRIT, Edgecam, Cimatron, CAMWorks, Creo, ZW3D, BobCAD and TopSolid are background knowledge only.
- I found no 2026 release-note detail for PowerMill, WorkNC, Edgecam, Cimatron, GibbsCAM, CATIA/DELMIA, TopSolid, Creo, ZW3D, BobCAD or OneCNC.
- I found no CIMdata market-share figures. Paywalled analyst reports did not appear in the search results.

---

## Q3. Automation and AI-assisted programming

### Takeaway
Every leader now ships some kind of AI or automation layer:
- Siemens NX: AI Make Machining Suggestion with a GenAI option, plus NX CAM Copilot.
- Mastercam: Copilot (beta).
- Hexagon ESPRIT: ProPlanAI.
- Tebis: SmartOps templates.
- OPEN MIND: hyperMILL AUTOMATION Center [BK].

Classic feature-based machining (CAMWorks/SOLIDWORKS CAM AFR and TechDB, FeatureCAM, NX FBM, SolidCAM feature-driven iMachining) is still the base most of these build on.

### Cited Findings
- NX 2606 adds a generative-AI third suggestion per machining feature. The first two suggestions come from historical machining data and tool libraries, and the system can create parametric tools automatically. AI Make Machining Suggestion is available only in NX X Manufacturing. — [Siemens blog](https://blogs.sw.siemens.com/nx-manufacturing/?p=10125); [Janus](https://blog.janus-engineering.com/en_en/details/article/siemens-nx-release-2606/); [Siemens NX X Mfg](https://blogs.sw.siemens.com/nx-manufacturing/nx-x-manufacturing-cam/)
- NX CAM Copilot claims "80% or more" savings in programming time, starting with 2.5/3-axis. — [Siemens blog](https://blogs.sw.siemens.com/nx-manufacturing/nx-x-manufacturing-cam/) (vendor)
- Mastercam Copilot (beta) has a Help function for natural-language questions and a Command function that drives the user interface. — [Mastercam PR](https://www.bolsamania.com/nota-de-prensa/mercados/mastercam-2026r2-delivers-gpu-accelerated-simulation-and-ai-powered-assistance--21635723.html)
- ESPRIT EDGE ProPlanAI covers turning, mill-turn and Swiss process planning. — [Hexagon PR](https://hexagon.com/company/newsroom/press-releases/2026/hexagon-introduces-new-esprit-edge-capabilities-to-reduce-cnc-programming-effort-and-support-high-complexity-machining)
- Tebis SmartOps provides end-to-end CAD/CAM process automation with templates. — [Tebis](https://www.tebis.com/en/software/products/tebis-4.1-release-10)
- SolidCAM 2026 takes iMachining 2D levels directly from recognized hole features. — [Solid Solutions](https://www.solidsolutions.co.uk/whats-new-in-solidcam-2026/)
- Mastercam 2026.R2 GPU simulation cut one verification run from about 90 minutes to about 22 minutes. — [Mastercam PR](https://www.bolsamania.com/nota-de-prensa/mercados/mastercam-2026r2-delivers-gpu-accelerated-simulation-and-ai-powered-assistance--21635723.html)

### Inferences
- The AI features shipping so far are mainly **assistants for choosing operations and tools** and **natural-language help**. They are not AI-generated 5-axis tool axes. NX's Copilot starts at 2.5/3-axis, and ProPlanAI targets turning. AI-assisted *multi-axis* strategy selection appears to be open ground.

### Gaps
- I found no independent benchmarks of these AI features.
- I did not research details of hyperMILL AUTOMATION Center, PowerMill automation, or Mastercam's feature-based tools this session.

---

## Q4. Pricing tiers

### Takeaway
Almost all high-end CAM is sold by quote. Public numbers exist only for subscription-led products (SprutCAM, NX X listings on G2). Rough tiers: **low** (BobCAD, SprutCAM, ZW3D, OneCNC); **mid** (SolidCAM, CAMWorks, Edgecam, GibbsCAM, Cimatron, Mastercam 3-axis); **high** (hyperMILL 5-axis, PowerMill Ultimate, Mastercam multiaxis, ESPRIT, Tebis, TopSolid); **enterprise** (NX, CATIA/DELMIA). These tiers are [BK]; only the figures below are sourced.

### Cited Findings
- SprutCAM X: about $249/month; continuous 4/5-axis option about +$100/month; annual term saves about 33%; perpetual license available after 3 years of subscription. — [Fitgap](https://us.fitgap.com/products/sprutcam). Conflicts with Capterra's "€1000 one-time" — [Capterra](https://www.capterra.com/p/163681/SprutCAM/)
- NX CAM: G2 lists "Starting at $220/month". Fusion: $680/year. — [G2](https://www.g2.com/compare/mastercam-vs-solidcam-vs-hypermill-vs-nx-cam)
- hyperMILL, SolidCAM and Mastercam publish no prices. Mastercam licensing is modular, with cost scaling by axes. — [G2 hyperMILL](https://www.g2.com/products/hypermill/pricing); [Accio](https://www.accio.com/business/mastercam-2023-price)

### Inferences
- A Fusion plugin sits on a host that costs about $680/year (Fusion, per G2) and would compete against 5-axis add-ons that typically run five figures [BK]. That leaves room for a mid-priced 5-axis add-on.

### Gaps
- I found no reliable published prices for Mastercam, hyperMILL, PowerMill, ESPRIT, Tebis, BobCAD or CAMWorks multiaxis. Reseller quotes would be needed.
