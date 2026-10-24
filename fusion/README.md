# Pat add-in for Autodesk Fusion

Pat's verified toolpaths inside Fusion's Manufacture workspace.

> **Status:** the Fusion-independent code (`pat_fusion`: tool conversion,
> STL export, scene writing, preview geometry, the engine session) is unit
> tested in CI, including an end-to-end export → rough → verify → save run.
> The Fusion glue (`patcam_glue.py`) has not been run inside Fusion yet.
> Please report anything that breaks.

## Install

1. Get `PatCAM-<platform>.zip` from the latest CI run (artifact `fusion-addin-*`),
   or build it yourself:

   ```sh
   pip install maturin
   python fusion/build_addin.py               # add --universal2 on macOS for Intel + Apple Silicon
   ```

2. Unzip it so you have a `PatCAM` folder, then either:
   - in Fusion, open **Utilities ▸ Add-Ins ▸ Scripts and Add-Ins**, press **+**
     and choose the `PatCAM` folder; or
   - copy it into Fusion's AddIns folder:
     - macOS: `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/`
     - Windows: `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\`
3. Run **PatCAM** from the Add-Ins dialog (tick *Run on Startup* to keep it).

## Use

The commands appear in the Manufacture workspace (Add-Ins panel, or a **PAT**
panel):

- **Pat: Generate & Verify**
  1. Pick a setup, a machine (Pat machine files are bundled; *Browse…* for
     your own), a strategy, a tool number and the parameters.
  2. Optionally select stock bodies (otherwise a box around the part, grown
     by the stock offset) and fixture bodies.
  3. Pat exports the setup to a job folder (`~/Documents/Pat Jobs/…`):
     - part, stock and fixture meshes as STL in the setup's work coordinate
       system;
     - tools from the setup's operations, converted from Fusion's tool JSON;
     - the machine file.
  4. It generates the toolpath, replays it on the machine model with
     collision and stock simulation, draws it in the canvas (blue feed, red
     rapid, green tool-axis ticks), and shows the verification report.
  5. Programs that fail verification can only be saved after an explicit
     warning.
- **Pat: Check G-code**: simulate any NC file (for example one posted by
  Fusion) against the machine and the setup.
- **Pat: Clear Preview**.

Strategies: adaptive rough (fixed axis or automatic 3+2), rotary rough and
finish (4th axis), and 5-axis surface finish (normal plus lead).

## Machines

Pat needs a machine definition (`machines/*.toml` in the repository) that
matches your machine's kinematics and limits. Part zero (`wcs_origin`) must
correspond to the setup's WCS origin. Add machine geometry (`[[components]]`)
to get spindle, table and trunnion collision checks.

## Layout

| File | Role |
|---|---|
| `PatCAM.manifest`, `PatCAM.py` | Add-in entry point |
| `patcam_glue.py` | Fusion API: commands, setup export, custom-graphics preview |
| `pat_fusion/` | Fusion-free logic, tested in CI (`fusion/tests`) |
| `lib/` | Compiled `pat` engine (built, not committed) |
| `resources/machines/` | Bundled machine files (built, not committed) |
