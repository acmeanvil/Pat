"""Binary STL output for meshes extracted from Fusion bodies."""

import struct


def write_binary_stl(path, vertices, triangles, name="pat"):
    """Write ``vertices`` (list of (x, y, z) in mm) and ``triangles`` (list of
    index triples) to a binary STL file."""
    with open(path, "wb") as f:
        f.write(name.encode("ascii", "replace")[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(triangles)))
        for a, b, c in triangles:
            p, q, r = vertices[a], vertices[b], vertices[c]
            u = (q[0] - p[0], q[1] - p[1], q[2] - p[2])
            v = (r[0] - p[0], r[1] - p[1], r[2] - p[2])
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            ln = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5 or 1.0
            f.write(struct.pack("<3f", n[0] / ln, n[1] / ln, n[2] / ln))
            for pt in (p, q, r):
                f.write(struct.pack("<3f", *pt))
            f.write(b"\x00\x00")


def box_mesh(lo, hi):
    """Closed box mesh (for tests and simple stock): (vertices, triangles)."""
    v = [
        (lo[0] if i & 1 == 0 else hi[0], lo[1] if i & 2 == 0 else hi[1], lo[2] if i & 4 == 0 else hi[2])
        for i in range(8)
    ]
    quads = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    t = []
    for a, b, c, d in quads:
        t += [(a, b, c), (a, c, d)]
    return v, t
