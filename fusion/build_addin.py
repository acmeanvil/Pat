"""Build the Fusion add-in: bundle the compiled engine and machines, zip it.

    python fusion/build_addin.py [--universal2]

1. Builds the ``pat`` Python extension with maturin (an abi3 wheel works with
   any Python >= 3.10, including the one embedded in Fusion). With
   ``--universal2`` on macOS it builds for Intel and Apple Silicon.
2. Extracts the wheel's module into ``fusion/PatCAM/lib/``.
3. Copies ``machines/*.toml`` into ``fusion/PatCAM/resources/machines/``.
4. Writes ``dist/PatCAM-<platform>.zip``, ready to unzip into Fusion's
   AddIns folder.
"""

import glob
import os
import platform
import shutil
import subprocess
import sys
import zipfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ADDIN = os.path.join(REPO, "fusion", "PatCAM")


def main():
    universal = "--universal2" in sys.argv
    wheels = os.path.join(REPO, "target", "wheels")
    shutil.rmtree(wheels, ignore_errors=True)
    cmd = [sys.executable, "-m", "maturin", "build", "--release", "-m", os.path.join(REPO, "crates", "pat-py", "Cargo.toml")]
    if universal and sys.platform == "darwin":
        cmd += ["--target", "universal2-apple-darwin"]
    subprocess.check_call(cmd, cwd=REPO)
    wheel = sorted(glob.glob(os.path.join(wheels, "*.whl")))[-1]

    lib = os.path.join(ADDIN, "lib")
    for entry in os.listdir(lib) if os.path.isdir(lib) else []:
        if entry != ".gitkeep":
            p = os.path.join(lib, entry)
            shutil.rmtree(p) if os.path.isdir(p) else os.remove(p)
    os.makedirs(lib, exist_ok=True)
    with zipfile.ZipFile(wheel) as z:
        for name in z.namelist():
            if ".dist-info/" in name:
                continue
            z.extract(name, lib)
    print("engine:", ", ".join(sorted(os.listdir(lib))))

    machines = os.path.join(ADDIN, "resources", "machines")
    os.makedirs(machines, exist_ok=True)
    for f in glob.glob(os.path.join(REPO, "machines", "*.toml")):
        shutil.copy(f, machines)

    tag = {"darwin": "macos", "win32": "windows"}.get(sys.platform, sys.platform)
    arch = "universal2" if universal and sys.platform == "darwin" else platform.machine().lower()
    dist = os.path.join(REPO, "dist")
    os.makedirs(dist, exist_ok=True)
    out = os.path.join(dist, "PatCAM-%s-%s.zip" % (tag, arch))
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for root, dirs, files in os.walk(ADDIN):
            dirs[:] = [d for d in dirs if d != "__pycache__"]
            for f in files:
                if f.endswith(".pyc") or f == ".gitkeep":
                    continue
                full = os.path.join(root, f)
                z.write(full, os.path.join("PatCAM", os.path.relpath(full, ADDIN)))
    print("wrote", out)


if __name__ == "__main__":
    main()
