"""Everything in the add-in that talks to the Fusion API.

Commands (Manufacture workspace):

- **Pat: Generate & Verify**: export the chosen CAM setup and run a Pat
  strategy. The result is shown with its verification report and previewed
  in the canvas, and G-code can be saved.
- **Pat: Check G-code**: simulate an existing NC file against the machine
  and the setup (collisions, gouges, limits, singularities).
- **Pat: Clear Preview**: remove Pat's custom graphics.

This module is kept thin: it pulls plain data out of Fusion and hands it to
``pat_fusion``, which is tested outside Fusion. API calls that vary between
Fusion versions are wrapped defensively, and failures are reported to the
user rather than ignored.
"""

import json
import os
import shutil
import traceback

import adsk.cam
import adsk.core
import adsk.fusion

from pat_fusion import geometry, scene, stl, tools
from pat_fusion.engine import Job, PatError, verification_text

HERE = os.path.dirname(os.path.abspath(__file__))
SETTINGS = os.path.join(os.path.expanduser("~"), ".pat_fusion.json")
GRAPHICS_ID = "PatPreview"

CMD_GENERATE = "PatGenerateCmd"
CMD_CHECK = "PatCheckCmd"
CMD_CLEAR = "PatClearCmd"
PANEL_CANDIDATES = ["CAMScriptsAddinsPanel", "SolidScriptsAddinsPanel"]

STRATEGIES = [
    ("Adaptive rough (fixed axis)", "rough"),
    ("Adaptive rough (automatic 3+2)", "rough-auto"),
    ("Rotary rough (4th axis)", "rotary-rough"),
    ("Rotary finish (4th axis)", "rotary-finish"),
    ("5-axis surface finish (normal + lead)", "surface"),
]

_handlers = []  # keep event handlers alive
_last = {}  # last job and program, for saving


def app():
    return adsk.core.Application.get()


def ui():
    return app().userInterface


def load_settings():
    try:
        with open(SETTINGS, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def save_settings(d):
    try:
        with open(SETTINGS, "w", encoding="utf-8") as f:
            json.dump(d, f, indent=1)
    except Exception:
        pass


def bundled_machines():
    folder = os.path.join(HERE, "resources", "machines")
    if not os.path.isdir(folder):
        return []
    return sorted(os.path.join(folder, f) for f in os.listdir(folder) if f.endswith(".toml"))


# ------------------------------------------------------------------ export


def cam_product():
    doc = app().activeDocument
    cam = adsk.cam.CAM.cast(doc.products.itemByProductType("CAMProductType")) if doc else None
    if not cam:
        raise PatError("Open a design with a Manufacture setup first.")
    return cam


def wcs_matrix(setup):
    """The setup's work coordinate system as a 16-element list (cm)."""
    try:
        m = setup.workCoordinateSystem
        if m:
            return list(m.asArray())
    except Exception:
        pass
    return list(geometry.IDENTITY)


def bodies_of(entity):
    """BRep bodies of a model entry (body, occurrence or component)."""
    body = adsk.fusion.BRepBody.cast(entity)
    if body:
        return [body]
    occ = adsk.fusion.Occurrence.cast(entity)
    if occ:
        out = list(occ.bRepBodies)
        for child in occ.childOccurrences:
            out += bodies_of(child)
        return out
    comp = adsk.fusion.Component.cast(entity)
    if comp:
        return list(comp.bRepBodies)
    return []


def body_to_stl(body, m, path):
    calc = body.meshManager.createMeshCalculator()
    calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.HighQualityTriangleMesh)
    mesh = calc.calculate()
    verts, tris = geometry.mesh_to_wcs(m, list(mesh.nodeCoordinatesAsDouble), list(mesh.nodeIndices))
    stl.write_binary_stl(path, verts, tris, body.name)
    return verts


def setup_tools(setup):
    """Pat tool definitions for every tool used in the setup."""
    out, notes, seen = [], [], set()
    for op in setup.allOperations:
        try:
            t = op.tool
            data = json.loads(t.toJson())
        except Exception:
            continue
        try:
            td, n = tools.fusion_tool_to_pat(data)
        except ValueError as e:
            notes.append("%s: %s" % (op.name, e))
            continue
        if td["number"] in seen:
            continue
        seen.add(td["number"])
        out.append(td)
        notes += ["T%d: %s" % (td["number"], x) for x in n]
    return out, notes


def export_job(setup, machine_path, stock_bodies, fixture_bodies, stock_offset_mm, folder):
    """Write machine, STL bodies and scene for a setup; returns (Job, notes)."""
    m = wcs_matrix(setup)
    job = Job(folder)
    job.write_machine(machine_path)
    bodies, lo, hi = [], [1e18] * 3, [-1e18] * 3
    k = 0
    for entity in setup.models:
        for b in bodies_of(entity):
            k += 1
            name = "part_%d.stl" % k
            for v in body_to_stl(b, m, job.path(name)):
                lo = [min(a, c) for a, c in zip(lo, v)]
                hi = [max(a, c) for a, c in zip(hi, v)]
            bodies.append({"name": b.name or name, "role": "part", "stl": name})
    if not bodies:
        raise PatError("The setup has no model bodies.")
    if stock_bodies:
        for i, b in enumerate(stock_bodies):
            name = "stock_%d.stl" % (i + 1)
            body_to_stl(b, m, job.path(name))
            bodies.append({"name": b.name or name, "role": "stock", "stl": name})
    else:
        o = stock_offset_mm
        bodies.append({"name": "stock box", "role": "stock", "box": ([lo[0] - o, lo[1] - o, lo[2]], [hi[0] + o, hi[1] + o, hi[2] + o])})
    for i, b in enumerate(fixture_bodies):
        name = "fixture_%d.stl" % (i + 1)
        body_to_stl(b, m, job.path(name))
        bodies.append({"name": b.name or name, "role": "fixture", "stl": name})
    tool_defs, notes = setup_tools(setup)
    if not tool_defs:
        raise PatError("No tools found: add at least one operation with a tool to the setup.")
    job.write_scene(scene.scene_toml(tool_defs, bodies))
    job.load()
    return job, notes, m


# ----------------------------------------------------------------- preview


def clear_preview():
    design = adsk.fusion.Design.cast(app().activeProduct) or adsk.fusion.Design.cast(
        app().activeDocument.products.itemByProductType("DesignProductType")
    )
    if not design:
        return
    groups = design.rootComponent.customGraphicsGroups
    for i in range(groups.count - 1, -1, -1):
        g = groups.item(i)
        if g.id == GRAPHICS_ID:
            g.deleteMe()
    app().activeViewport.refresh()


def draw_preview(moves, m):
    design = adsk.fusion.Design.cast(app().activeDocument.products.itemByProductType("DesignProductType"))
    if not design:
        return
    clear_preview()
    group = design.rootComponent.customGraphicsGroups.add()
    group.id = GRAPHICS_ID
    colors = {"feed": (30, 110, 230), "rapid": (230, 60, 40), "axis": (40, 170, 90)}
    for kind, coords in geometry.preview_polylines(moves, m, axis_every=25):
        cg = adsk.fusion.CustomGraphicsCoordinates.create(coords)
        lines = group.addLines(cg, [], kind != "axis")
        r, g, b = colors[kind]
        lines.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(adsk.core.Color.create(r, g, b, 255))
        lines.weight = 1.0 if kind == "feed" else 0.5
    app().activeViewport.refresh()


# ------------------------------------------------------------------ commands


def pick_file(title, filt, save=False, initial=None):
    dlg = ui().createFileDialog()
    dlg.title = title
    dlg.filter = filt
    if initial:
        dlg.initialFilename = initial
    res = dlg.showSave() if save else dlg.showOpen()
    return dlg.filename if res == adsk.core.DialogResults.DialogOK else None


def job_folder(setup_name):
    root = load_settings().get("job_root") or os.path.join(os.path.expanduser("~"), "Documents", "Pat Jobs")
    doc = app().activeDocument.name if app().activeDocument else "design"
    safe = "".join(c if c.isalnum() or c in "-_ " else "_" for c in "%s - %s" % (doc, setup_name))
    return os.path.join(root, safe)


def offer_save(job, result):
    pid = result.get("program_id")
    if pid is None:
        return
    force = False
    if not result.get("verified"):
        r = ui().messageBox(
            "This program FAILED verification:\n\n%s\n\nSave it anyway? Only do this if you have checked "
            "the issues and accept the risk." % verification_text(result),
            "Pat: unverified program",
            adsk.core.MessageBoxButtonTypes.YesNoButtonType,
            adsk.core.MessageBoxIconTypes.WarningIconType,
        )
        if r != adsk.core.DialogResults.DialogYes:
            return
        force = True
    target = pick_file("Save G-code", "NC files (*.nc);;All files (*.*)", save=True, initial="pat_%d.nc" % pid)
    if not target:
        return
    saved = job.call("save_program", id=pid, path="program_%d.nc" % pid, force=force)
    shutil.copyfile(saved["written"], target)
    ui().messageBox("Wrote %s (%d blocks, %s)." % (target, saved["blocks"], saved["note"]), "Pat")


class GenerateCreated(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            inputs = cmd.commandInputs
            settings = load_settings()
            cam = cam_product()
            dd = inputs.addDropDownCommandInput("setup", "Setup", adsk.core.DropDownStyles.TextListDropDownStyle)
            for i in range(cam.setups.count):
                dd.listItems.add(cam.setups.item(i).name, i == 0)
            mdd = inputs.addDropDownCommandInput("machine", "Machine", adsk.core.DropDownStyles.TextListDropDownStyle)
            last = settings.get("machine")
            choices = bundled_machines()
            if last and last not in choices:
                choices.insert(0, last)
            for p in choices:
                mdd.listItems.add(os.path.basename(p), p == last or (not last and p == choices[0]))
            mdd.listItems.add("Browse...", not choices)
            sdd = inputs.addDropDownCommandInput("strategy", "Strategy", adsk.core.DropDownStyles.TextListDropDownStyle)
            for i, (label, _) in enumerate(STRATEGIES):
                sdd.listItems.add(label, i == 0)
            inputs.addIntegerSpinnerCommandInput("tool", "Tool number", 1, 999, 1, int(settings.get("tool", 1)))
            st = inputs.addSelectionInput("stock", "Stock bodies", "Optional: bodies to use as stock")
            st.addSelectionFilter("Bodies")
            st.setSelectionLimits(0, 0)
            fx = inputs.addSelectionInput("fixtures", "Fixtures", "Optional: vises, clamps, plates")
            fx.addSelectionFilter("Bodies")
            fx.setSelectionLimits(0, 0)
            mm = lambda v: adsk.core.ValueInput.createByReal(v / 10.0)  # noqa: E731 (cm internally)
            inputs.addValueInput("stock_offset", "Stock box offset", "mm", mm(2.0))
            inputs.addValueInput("stepdown", "Stepdown", "mm", mm(2.0))
            inputs.addValueInput("stepover", "Stepover / pitch", "mm", mm(1.0))
            inputs.addValueInput("leave", "Stock to leave", "mm", mm(0.3))
            inputs.addValueInput("resolution", "Resolution", "mm", mm(0.5))
            inputs.addValueInput("lead", "Lead angle (5-axis)", "deg", adsk.core.ValueInput.createByString("10 deg"))
            on_exec = GenerateExecute()
            cmd.execute.add(on_exec)
            _handlers.append(on_exec)
        except Exception:
            ui().messageBox(traceback.format_exc(), "Pat")


class GenerateExecute(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            inputs = args.command.commandInputs
            cam = cam_product()
            setup = cam.setups.item(inputs.itemById("setup").selectedItem.index)
            choice = inputs.itemById("machine").selectedItem.name
            machine = next((p for p in bundled_machines() + [load_settings().get("machine") or ""] if p and os.path.basename(p) == choice), None)
            if choice == "Browse..." or not machine:
                machine = pick_file("Pat machine file", "Pat machine (*.toml)")
                if not machine:
                    return
            strategy = STRATEGIES[inputs.itemById("strategy").selectedItem.index][1]
            tool = inputs.itemById("tool").value
            sel = lambda i: [inputs.itemById(i).selection(k).entity for k in range(inputs.itemById(i).selectionCount)]  # noqa: E731
            val = lambda i: inputs.itemById(i).value * 10.0  # noqa: E731 (cm -> mm)
            save_settings(dict(load_settings(), machine=machine, tool=tool))

            stock = [adsk.fusion.BRepBody.cast(e) for e in sel("stock") if adsk.fusion.BRepBody.cast(e)]
            fixtures = [adsk.fusion.BRepBody.cast(e) for e in sel("fixtures") if adsk.fusion.BRepBody.cast(e)]
            job, notes, m = export_job(setup, machine, stock, fixtures, val("stock_offset"), job_folder(setup.name))
            stepdown, stepover, leave, res = val("stepdown"), val("stepover"), val("leave"), val("resolution")
            if strategy == "rough":
                r = job.call("rough", tool=tool, params={"stepdown": stepdown, "stepover": stepover, "leave": leave, "resolution": res})
            elif strategy == "rough-auto":
                r = job.call("rough", tool=tool, auto=True, params={"stepdown": stepdown, "stepover": stepover, "leave": leave, "resolution": res})
            elif strategy in ("rotary-rough", "rotary-finish"):
                r = job.call("rotary", tool=tool, mode=strategy.split("-")[1], params={"stepdown": stepdown, "stepover": stepover, "leave": leave, "dx": res})
            else:
                lead = inputs.itemById("lead").value * 57.29577951308232  # rad -> deg
                r = job.call("surface_finish", tool=tool, params={
                    "stepover": stepover, "step": max(stepover, res), "leave": leave,
                    "axis": {"control": {"type": "normal", "lead_deg": lead, "tilt_deg": 0.0}},
                })
            prog = job.call("get_program", id=r["program_id"], gcode_lines=0, include_cl=True)
            draw_preview(prog.get("cl_moves", []), m)
            _last.update(job=job, result=r)
            text = verification_text(r)
            if notes:
                text += "\n\nNotes:\n" + "\n".join(notes[:8])
            ui().messageBox(text + "\n\nJob folder: " + job.folder, "Pat")
            offer_save(job, r)
        except PatError as e:
            ui().messageBox(str(e), "Pat")
        except Exception:
            ui().messageBox(traceback.format_exc(), "Pat")


class CheckCreated(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            on_exec = CheckExecute()
            args.command.execute.add(on_exec)
            _handlers.append(on_exec)
            inputs = args.command.commandInputs
            cam = cam_product()
            dd = inputs.addDropDownCommandInput("setup", "Setup (for collision checks)", adsk.core.DropDownStyles.TextListDropDownStyle)
            for i in range(cam.setups.count):
                dd.listItems.add(cam.setups.item(i).name, i == 0)
            inputs.addBoolValueInput("stock", "Simulate stock removal", True, "", True)
        except Exception:
            ui().messageBox(traceback.format_exc(), "Pat")


class CheckExecute(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            inputs = args.command.commandInputs
            nc = pick_file("G-code to check", "NC files (*.nc *.ngc *.tap *.gcode);;All files (*.*)")
            if not nc:
                return
            machine = load_settings().get("machine") or pick_file("Pat machine file", "Pat machine (*.toml)")
            if not machine:
                return
            setup = cam_product().setups.item(inputs.itemById("setup").selectedItem.index)
            job, _, _ = export_job(setup, machine, [], [], 2.0, job_folder(setup.name))
            local = job.path("checked.nc")
            shutil.copyfile(nc, local)
            v = job.call("check_gcode", path="checked.nc", simulate_stock=inputs.itemById("stock").value)
            ui().messageBox(verification_text(v), "Pat: %s" % os.path.basename(nc))
        except PatError as e:
            ui().messageBox(str(e), "Pat")
        except Exception:
            ui().messageBox(traceback.format_exc(), "Pat")


class ClearCreated(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            clear_preview()
        except Exception:
            ui().messageBox(traceback.format_exc(), "Pat")


def _panel():
    ws = ui().workspaces.itemById("CAMEnvironment")
    if not ws:
        return None
    for pid in PANEL_CANDIDATES:
        p = ws.toolbarPanels.itemById(pid)
        if p:
            return p
    return ws.toolbarPanels.itemById("PatPanel") or ws.toolbarPanels.add("PatPanel", "PAT")


def start():
    defs = ui().commandDefinitions
    panel = _panel()
    for cid, name, tip, created in [
        (CMD_GENERATE, "Pat: Generate & Verify", "Generate a verified toolpath with Pat for a setup", GenerateCreated),
        (CMD_CHECK, "Pat: Check G-code", "Simulate an NC program against the machine and setup", CheckCreated),
        (CMD_CLEAR, "Pat: Clear Preview", "Remove Pat toolpath previews", ClearCreated),
    ]:
        d = defs.itemById(cid) or defs.addButtonDefinition(cid, name, tip)
        handler = created()
        d.commandCreated.add(handler)
        _handlers.append(handler)
        if panel and not panel.controls.itemById(cid):
            panel.controls.addCommand(d)


def stop():
    panel = _panel()
    for cid in (CMD_GENERATE, CMD_CHECK, CMD_CLEAR):
        if panel:
            c = panel.controls.itemById(cid)
            if c:
                c.deleteMe()
        d = ui().commandDefinitions.itemById(cid)
        if d:
            d.deleteMe()
    try:
        clear_preview()
    except Exception:
        pass
    _handlers.clear()
