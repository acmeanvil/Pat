"""Run Pat on an exported job folder.

A job folder holds ``machine.toml``, ``scene.toml`` and the STL files the
scene refers to. :class:`Job` wraps ``pat.Session`` (the same tools the MCP
server gives AI agents) with dict-in / dict-out calls.
"""

import json
import os
import shutil


def _pat():
    import pat  # the compiled engine, vendored into the add-in's lib/ folder

    return pat


class PatError(Exception):
    pass


class Job:
    def __init__(self, folder):
        self.folder = os.path.abspath(folder)
        os.makedirs(self.folder, exist_ok=True)
        self.session = _pat().Session(self.folder)

    def path(self, name):
        return os.path.join(self.folder, name)

    def write_machine(self, source_path):
        """Copy a Pat machine file into the job (STL components next to it
        are copied too)."""
        dst = self.path("machine.toml")
        shutil.copyfile(source_path, dst)
        src_dir = os.path.dirname(os.path.abspath(source_path))
        with open(source_path, "r", encoding="utf-8") as f:
            text = f.read()
        for line in text.splitlines():
            if 'type = "stl"' in line and "path" in line:
                rel = line.split("path", 1)[1].split('"')[1]
                src = os.path.join(src_dir, rel)
                if os.path.exists(src):
                    os.makedirs(os.path.dirname(self.path(rel)) or self.folder, exist_ok=True)
                    shutil.copyfile(src, self.path(rel))
        return dst

    def write_scene(self, toml_text):
        with open(self.path("scene.toml"), "w", encoding="utf-8") as f:
            f.write(toml_text)

    def call(self, name, **args):
        try:
            return json.loads(self.session.call(name, json.dumps(args)))
        except ValueError as e:
            raise PatError(str(e))

    def load(self):
        self.call("load_machine", path="machine.toml")
        return self.call("load_scene", path="scene.toml")


def verification_text(result):
    """Short human summary of a generated program's verification."""
    v = result.get("verification", result)
    lines = []
    if result.get("program_id") is not None:
        lines.append("Program %s: %s" % (result["program_id"], "VERIFIED" if result.get("verified") else "NOT VERIFIED"))
    lines.append("%d errors, %d warnings, cycle time %.1f min" % (v.get("errors", 0), v.get("warnings", 0), v.get("cycle_time_s", 0) / 60.0))
    for code, n in (v.get("issue_counts") or {}).items():
        lines.append("  %s x%d: %s" % (code, n, (v.get("remedies") or {}).get(code, "")))
    summary = result.get("summary")
    if isinstance(summary, dict) and summary.get("summary"):
        lines.append(str(summary["summary"]))
    return "\n".join(lines)
