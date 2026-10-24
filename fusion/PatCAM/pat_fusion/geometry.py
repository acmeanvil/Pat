"""Coordinate handling between Fusion (design space, cm) and Pat (WCS, mm).

A setup's work coordinate system is a 4x4 matrix in Fusion's convention
(``Matrix3D.asArray()``, row-major, translation in elements 3, 7, 11, in
cm): its columns are the WCS axes and origin in design space.
"""

CM = 10.0  # mm per cm

IDENTITY = [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0]


def design_to_wcs(m, p_cm):
    """Design-space point (cm) -> WCS point (mm)."""
    o = (m[3], m[7], m[11])
    d = (p_cm[0] - o[0], p_cm[1] - o[1], p_cm[2] - o[2])
    # Inverse of an orthonormal rotation: the transpose.
    return (
        (m[0] * d[0] + m[4] * d[1] + m[8] * d[2]) * CM,
        (m[1] * d[0] + m[5] * d[1] + m[9] * d[2]) * CM,
        (m[2] * d[0] + m[6] * d[1] + m[10] * d[2]) * CM,
    )


def wcs_to_design(m, p_mm):
    """WCS point (mm) -> design-space point (cm)."""
    x, y, z = p_mm[0] / CM, p_mm[1] / CM, p_mm[2] / CM
    return (
        m[0] * x + m[1] * y + m[2] * z + m[3],
        m[4] * x + m[5] * y + m[6] * z + m[7],
        m[8] * x + m[9] * y + m[10] * z + m[11],
    )


def mesh_to_wcs(m, node_coords_cm, node_indices):
    """Flat Fusion mesh arrays (cm, design space) -> (vertices mm WCS,
    triangles)."""
    verts = [design_to_wcs(m, node_coords_cm[i:i + 3]) for i in range(0, len(node_coords_cm), 3)]
    tris = [tuple(node_indices[i:i + 3]) for i in range(0, len(node_indices), 3)]
    return verts, tris


def preview_polylines(moves, m, axis_every=0, axis_length=10.0):
    """Turn CL moves (Pat JSON: type, tip, axis) into polylines for custom
    graphics: a list of (kind, flat_coords_cm) where kind is "feed", "rapid"
    or "axis". Consecutive moves of one kind form one strip. With
    ``axis_every`` > 0, every n-th feed point also gets a tool-axis tick."""
    out = []
    kind, strip = None, []
    last = None
    count = 0
    for mv in moves:
        k = "rapid" if mv["type"] == "rapid" else "feed"
        p = wcs_to_design(m, mv["tip"])
        if k != kind:
            if len(strip) >= 6:
                out.append((kind, strip))
            strip = list(last) if last is not None else []
            kind = k
        strip += list(p)
        last = p
        if k == "feed" and axis_every > 0:
            count += 1
            if count % axis_every == 0:
                a = mv["axis"]
                q = wcs_to_design(m, (mv["tip"][0] + a[0] * axis_length, mv["tip"][1] + a[1] * axis_length, mv["tip"][2] + a[2] * axis_length))
                out.append(("axis", list(p) + list(q)))
    if len(strip) >= 6:
        out.append((kind, strip))
    return out
