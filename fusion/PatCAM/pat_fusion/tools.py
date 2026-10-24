"""Fusion tool-library JSON -> Pat tool definitions.

Fusion's tool JSON (``adsk.cam.Tool.toJson()`` or a ``.json`` tool library)
keeps cutter geometry under ``geometry`` with ISO-style keys:

    DC    cutting diameter       LCF   flute (cutting) length
    LB    body length (stick-out from the holder)
    RE    corner radius          SFDM  shaft diameter

plus ``holder.segments`` (height, lower-diameter, upper-diameter, listed
from the tool end up), ``post-process.number`` and ``unit``.
"""

INCH = 25.4

# Holder assumed when the tool has none, so collision checks still guard
# the spindle nose region.
DEFAULT_HOLDER = {"height": 40.0, "lower-diameter": 32.0, "upper-diameter": 40.0}


def _num(d, key, default=None):
    v = d.get(key, default)
    if isinstance(v, dict):  # some exports wrap values: {"value": 6, "unit": "mm"}
        v = v.get("value", default)
    return default if v is None else float(v)


def fusion_tool_to_pat(tool, gauge_length=None):
    """Convert one Fusion tool (dict) to a Pat tool definition.

    Returns ``(tool_def, notes)``: ``tool_def`` is a dict matching Pat's
    ``[[tools]]`` table, ``notes`` lists assumptions made.
    """
    notes = []
    unit = str(tool.get("unit", "millimeters")).lower()
    k = INCH if unit.startswith("inch") else 1.0
    g = tool.get("geometry", {})
    kind = str(tool.get("type", "flat end mill")).lower()

    dc = _num(g, "DC")
    if dc is None or dc <= 0:
        raise ValueError("tool has no cutting diameter (geometry.DC)")
    dc *= k
    lcf = (_num(g, "LCF") or dc * 2.0) * k
    lb = (_num(g, "LB") or lcf * 1.5) * k
    sfdm = (_num(g, "SFDM") or dc / k) * k

    if "ball" in kind:
        corner = dc / 2.0
    elif "bull" in kind or "radius" in kind:
        corner = min((_num(g, "RE") or 0.0) * k, dc / 2.0)
    elif "flat" in kind or "end mill" in kind:
        corner = 0.0
    else:
        corner = min((_num(g, "RE") or 0.0) * k, dc / 2.0)
        notes.append("tool type '%s' is treated as an end mill" % kind)

    segments = [{"kind": "cutter", "length": round(lcf, 4), "diameter": round(dc, 4)}]
    if corner > 0:
        segments[0]["corner_radius"] = round(corner, 4)
    shank_len = lb - lcf
    if shank_len > 1e-6:
        segments.append({"kind": "shank", "length": round(shank_len, 4), "diameter": round(sfdm, 4)})

    holder = (tool.get("holder") or {}).get("segments") or []
    if not holder:
        holder = [DEFAULT_HOLDER]
        notes.append("no holder in the tool definition; assumed a 40 mm tall Ø32-40 holder")
        hk = 1.0
    else:
        hk = k
    holder_height = 0.0
    for s in holder:
        h = _num(s, "height", 0.0) * hk
        lo = _num(s, "lower-diameter", 0.0) * hk
        hi = _num(s, "upper-diameter", lo / hk if hk else lo) * hk
        if h <= 0:
            continue
        holder_height += h
        if abs(lo - hi) < 1e-9:
            segments.append({"kind": "holder", "length": round(h, 4), "diameter": round(lo, 4)})
        else:
            segments.append({"kind": "holder", "length": round(h, 4), "diameter_bottom": round(lo, 4), "diameter_top": round(hi, 4)})

    pp = tool.get("post-process", {})
    number = int(pp.get("number", 1) or 1)
    if gauge_length is None:
        gauge_length = lb + holder_height
        notes.append("gauge length taken as stick-out + holder height = %.2f mm" % gauge_length)

    return (
        {
            "number": number,
            "name": str(tool.get("description", kind)),
            "gauge_length": round(float(gauge_length), 4),
            "segments": segments,
        },
        notes,
    )
