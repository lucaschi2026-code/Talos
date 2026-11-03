"""Technical drawing set for Talos, generated from the CAD in talos_arm.py.

One A3 sheet per printed part, plus an assembly / BOM sheet, the STS3215
reference sheet and a sheet of common features (clevis, clip, screw holes).
Each sheet has third-angle front / top / right views with hidden lines, an
isometric view, key dimensions and a numbered feature list: the modelling
steps, in order, with every number needed to rebuild the part by hand.

    python drawings.py      # full set, the two work sets, and a PDF + PNG per sheet

All dimensions come from talos_arm.py, so the drawings follow the model.
"""

import datetime
import math
import textwrap
from pathlib import Path

import numpy as np
import cadquery as cq
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon, Rectangle
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepLib import BRepLib
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

import talos_arm as ta

OUT = Path(__file__).with_name("drawings")
PAPER = (420.0, 297.0)                      # A3 landscape, mm
REGION = (14.0, 14.0, 272.0, 283.0)         # view area (x0, y0, x1, y1)
COL_X = 278.0                               # right column: feature list + title block
# Dimensions go on the outer sides only (front: below/left, top: above, right view: right),
# so the gaps between views stay small. Margins leave room for up to 3 dimension levels.
M_LEFT, M_BOTTOM, M_TOP, M_RIGHT = 31.0, 35.0, 33.0, 22.0
GAP_FT, GAP_FR = 18.0, 30.0                 # front -> top, front -> right
PT = 72 / 25.4                              # points per mm
SCALES = [(5, "5:1"), (4, "4:1"), (3, "3:1"), (2, "2:1"), (1.5, "3:2"), (1, "1:1"), (2 / 3, "2:3"), (0.5, "1:2"),
          (0.4, "1:2.5"), (1 / 3, "1:3"), (0.25, "1:4"), (0.2, "1:5")]
VIEWS = {"front": ((0, -1, 0), (1, 0, 0)), "top": ((0, 0, 1), (1, 0, 0)),
         "right": ((1, 0, 0), (0, 1, 0)), "iso": ((1, -1, 1), (1, 1, 0))}
LABELS = {"front": "FRONT", "top": "TOP", "right": "RIGHT", "iso": "ISOMETRIC (NOT TO SCALE)"}
TODAY = datetime.date.today().isoformat()
REV = "A"                                   # bump when a released drawing changes
DIM_PT, NOTE_PT = 8.5, 7.5                  # dimension and leader-note text size, points

# Part numbering: TAL-0xx = assembly and reference sheets, TAL-1xx = printed parts,
# numbered base to tip. Files are named <part no>_<slug>_rev-<rev>.

# The parts are split into two work sets so two people can model them in parallel.
# The reference sheets (TAL-0xx) go in both. The split is balanced on modelling steps
# (A 40, B 36). The sets only meet at the elbow (upper arm shell -> forearm clevis), and
# both sides of it are defined on the shared sheets (servo pocket P, clevis).
WORK_SETS = {
    "A": ("base-to-upper-arm", "base, shoulder, upper arm, hub caps",
          ["TAL-101", "TAL-102", "TAL-103", "TAL-108", "TAL-109"]),
    "B": ("forearm-to-gripper", "forearm, wrist, gripper, moving jaw",
          ["TAL-104", "TAL-105", "TAL-106", "TAL-107"]),
}


def work_set(part_no):
    """Work set letter of a part ("A"/"B"), or None for a shared reference sheet."""
    return next((k for k, (_, _, parts) in WORK_SETS.items() if part_no in parts), None)

RHO, T_SHELL, INFILL = 1.24e-3, 1.2, 0.20   # same mass model as calcs/design_check.py

GENERAL = [
    "All dimensions mm. Third-angle projection. Do not scale the drawing.",
    "Part frame = the frame the part is built in, in talos_arm.py (given on each sheet).",
    "Printed tolerance +/-0.2 unless noted; servo pocket clearances are already included.",
]


def fmt(v):
    s = f"{v:.2f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def unit(v):
    v = np.array(v, float)
    return v / np.linalg.norm(v)


def proj(view, p):
    n, x = (unit(a) for a in VIEWS[view])
    y = np.cross(n, x)
    p = np.array(p, float)
    return np.array([p @ x, p @ y])


def polylines(shape):
    out = []
    if shape is None or shape.IsNull():
        return out
    BRepLib.BuildCurves3d_s(shape)
    exp = TopExp_Explorer(shape, TopAbs_EDGE)
    while exp.More():
        edge = TopoDS.Edge_s(exp.Current())
        try:
            d = GCPnts_QuasiUniformDeflection(BRepAdaptor_Curve(edge), 0.03)
            if d.IsDone() and d.NbPoints() >= 2:
                out.append(np.array([(d.Value(i).X(), d.Value(i).Y())
                                     for i in range(1, d.NbPoints() + 1)]))
        except Exception:
            pass
        exp.Next()
    return out


def hlr(shape, view, hidden=True):
    n, x = VIEWS[view]
    algo = HLRBRep_Algo()
    algo.Add(shape.wrapped)
    algo.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*unit(n)), gp_Dir(*unit(x)))))
    algo.Update()
    algo.Hide()
    h = HLRBRep_HLRToShape(algo)
    vis = polylines(h.VCompound()) + polylines(h.OutLineVCompound())
    hid = (polylines(h.HCompound()) + polylines(h.OutLineHCompound())) if hidden else []
    return vis, hid


def extents(lines):
    pts = np.vstack(lines)
    return pts.min(axis=0), pts.max(axis=0)


def printed_mass(shape):
    V, A = shape.Volume(), shape.Area()
    shell = min(V, A * T_SHELL)
    return RHO * (shell + INFILL * (V - shell))


def as_shape(obj):
    return obj.val() if isinstance(obj, cq.Workplane) else obj


# ---------------------------------------------------------------- sheet

class Sheet:
    def __init__(self, number, title, part_no, slug, qty="1", material="PLA, 20 % infill, 3 perimeters",
                 frame=""):
        self.number, self.title, self.part_no, self.slug = number, title, part_no, slug
        self.qty, self.material, self.frame = qty, material, frame
        self.fig = plt.figure(figsize=(PAPER[0] / 25.4, PAPER[1] / 25.4))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        ax = self.ax
        ax.set_xlim(0, PAPER[0])
        ax.set_ylim(0, PAPER[1])
        ax.set_aspect("equal")
        ax.axis("off")
        ax.add_patch(Rectangle((10, 10), 400, 277, fill=False, lw=0.7 * PT))
        self.place = {}   # view -> (offset, scale, model min, paper bbox)
        self.low = {}     # view -> lowest paper y used (view or its dimensions), for the label
        self.scale_label = "-"
        self.mass = None
        self.n_features = 0

    # ---- views
    def views(self, shape, hidden=True, iso=True, iso_shape=None, region=REGION, gap_right=0.0):
        shape = as_shape(shape)
        data = {v: hlr(shape, v, hidden) for v in ("front", "top", "right")}
        ext = {v: extents(data[v][0]) for v in data}
        size = {v: ext[v][1] - ext[v][0] for v in data}
        x0, y0, x1, y1 = region
        w_need = size["front"][0] + size["right"][0]
        h_need = size["front"][1] + size["top"][1]
        w_free = x1 - x0 - M_LEFT - GAP_FR - M_RIGHT - gap_right
        h_free = y1 - y0 - M_BOTTOM - GAP_FT - M_TOP
        s, label = SCALES[-1]
        for sc, lab in SCALES:
            if sc * w_need <= w_free and sc * h_need <= h_free:
                s, label = sc, lab
                break
        self.scale_label = label
        left, bottom = x0 + M_LEFT, y0 + M_BOTTOM
        origin = {"front": (left, bottom),
                  "top": (left, bottom + s * size["front"][1] + GAP_FT),
                  "right": (left + s * size["front"][0] + GAP_FR + gap_right, bottom)}
        for v in ("front", "top", "right"):
            mn = ext[v][0].copy()
            if v == "top":
                mn[0] = ext["front"][0][0]
            if v == "right":
                mn[1] = ext["front"][0][1]
            self._draw(v, data[v], origin[v], s, mn)
        if iso:
            vis, _ = hlr(as_shape(iso_shape) if iso_shape is not None else shape, "iso", hidden=False)
            mn, mx = extents(vis)
            cx0 = left + s * size["front"][0] + 27          # clear of the top view's right dims
            cy0 = max(origin["top"][1], bottom + s * size["front"][1] + 8)
            if x1 - cx0 < 60:                                # no room beside the top view: go above it
                cx0, cy0 = origin["right"][0], origin["top"][1]
            cw, ch = x1 - cx0 - 4, y1 - cy0 - 8
            si = min(0.9 * min(cw / (mx - mn)[0], ch / (mx - mn)[1]), 1.6 * s)  # keep it in proportion
            off = (cx0 + (cw - si * (mx - mn)[0]) / 2, cy0 + (ch - si * (mx - mn)[1]) / 2 + 4)
            self._draw("iso", (vis, []), off, si, mn)

    def _draw(self, view, data, origin, s, mn):
        vis, hid = data
        tr = lambda pts: np.column_stack([origin[0] + s * (pts[:, 0] - mn[0]),
                                          origin[1] + s * (pts[:, 1] - mn[1])])
        if hid:
            self.ax.add_collection(LineCollection([tr(p) for p in hid], colors="0.55",
                                                  linewidths=0.25 * PT, linestyles=(0, (3, 2))))
        self.ax.add_collection(LineCollection([tr(p) for p in vis], colors="k", linewidths=0.3 * PT))
        allp = tr(np.vstack(vis))
        bb = (allp[:, 0].min(), allp[:, 1].min(), allp[:, 0].max(), allp[:, 1].max())
        self.place[view] = (origin, s, mn, bb)
        self.low[view] = bb[1]

    def _labels(self):
        for view, (_, _, _, bb) in self.place.items():
            self.ax.text((bb[0] + bb[2]) / 2, self.low[view] - 4.0, LABELS[view],
                         ha="center", va="top", fontsize=8.5, weight="bold")

    def P(self, view, p):
        origin, s, mn, _ = self.place[view]
        u = proj(view, p)
        return np.array([origin[0] + s * (u[0] - mn[0]), origin[1] + s * (u[1] - mn[1])])

    # ---- annotation
    def _line(self, a, b, lw=0.18, color="k", ls="-"):
        self.ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=lw * PT, ls=ls,
                     solid_capstyle="butt")

    def _arrow(self, tip, direction):
        d = unit(direction)
        n = np.array([-d[1], d[0]])
        base = np.array(tip) - 2.8 * d
        self.ax.add_patch(Polygon([tip, base + 0.75 * n, base - 0.75 * n], closed=True,
                                  color="k", lw=0))

    def dim(self, view, p1, p2, axis, side, level=1, text=None, prefix="", suffix=""):
        """Linear dimension between two model points, drawn outside the view's bbox."""
        a, b = self.P(view, p1), self.P(view, p2)
        u1, u2 = proj(view, p1), proj(view, p2)
        _, _, _, bb = self.place[view]
        k = 0 if axis == "h" else 1
        value = abs(u2[k] - u1[k])
        label = text if text is not None else f"{prefix}{fmt(value)}{suffix}"
        step = 9.0
        if axis == "h":
            y = bb[3] + step * level if side == "above" else bb[1] - step * level
            sgn = 1 if side == "above" else -1
            for p in (a, b):
                self._line((p[0], p[1] + sgn * 1.0), (p[0], y + sgn * 1.2))
            self._line((a[0], y), (b[0], y))
            if abs(b[0] - a[0]) > 0.5:
                self._arrow((a[0], y), (a[0] - b[0], 0))
                self._arrow((b[0], y), (b[0] - a[0], 0))
            self.ax.text((a[0] + b[0]) / 2, y + 0.9, label, ha="center", va="bottom", fontsize=DIM_PT)
            if side == "below":
                self.low[view] = min(self.low[view], y - 1.5)
        else:
            x = bb[2] + step * level if side == "right" else bb[0] - step * level
            sgn = 1 if side == "right" else -1
            for p in (a, b):
                self._line((p[0] + sgn * 1.0, p[1]), (x + sgn * 1.2, p[1]))
            self._line((x, a[1]), (x, b[1]))
            if abs(b[1] - a[1]) > 0.5:
                self._arrow((x, a[1]), (0, a[1] - b[1]))
                self._arrow((x, b[1]), (0, b[1] - a[1]))
            self.ax.text(x - 0.9, (a[1] + b[1]) / 2, label, ha="right", va="center", fontsize=DIM_PT,
                         rotation=90)

    def note(self, view, p, text, dx, dy):
        a = self.P(view, p)
        b = a + np.array([dx, dy])
        self._line(a, b)
        self._arrow(a, a - b)
        end = b + np.array([3.0 if dx >= 0 else -3.0, 0])
        self._line(b, end)
        self.ax.text(end[0] + (0.8 if dx >= 0 else -0.8), end[1], text, fontsize=NOTE_PT,
                     ha="left" if dx >= 0 else "right", va="center")
        self.low[view] = min(self.low[view], end[1] - 2.5)

    def center(self, view, p, r=4.0):
        a = self.P(view, p)
        for d in ((1, 0), (0, 1)):
            d = np.array(d) * (r + 2)
            self._line(a - d, a + d, lw=0.15, color="0.3", ls=(0, (6, 1.5, 1, 1.5)))

    def axis_line(self, view, p1, p2):
        self._line(self.P(view, p1), self.P(view, p2), lw=0.15, color="0.3",
                   ls=(0, (6, 1.5, 1, 1.5)))

    # ---- right column
    def features(self, heading, items, y_top=283.0, width=96):
        ax, y = self.ax, y_top
        ax.text(COL_X, y, heading, fontsize=7.2, weight="bold", va="top")
        y -= 4.6
        count = 0
        for item in items:
            numbered = not item.startswith(("-", "*"))
            count += numbered
            self.n_features += numbered
            prefix = f"{count:>2}. " if numbered else "    "
            body = item.lstrip("-* ")
            lines = textwrap.wrap(body, width - len(prefix)) or [""]
            for j, line in enumerate(lines):
                ax.text(COL_X, y, (prefix if j == 0 else " " * len(prefix)) + line, fontsize=5.6,
                        va="top", family="DejaVu Sans Mono")
                y -= 2.5
            y -= 0.5
        return y

    def notes(self, items, y):
        self.ax.text(COL_X, y, "NOTES", fontsize=6.4, weight="bold", va="top")
        y -= 3.6
        for line in items:
            for k, w in enumerate(textwrap.wrap(line, 100)):
                self.ax.text(COL_X, y, ("- " if k == 0 else "  ") + w, fontsize=5.0, va="top")
                y -= 2.2
        return y

    def title_block(self, total):
        self._labels()
        ax = self.ax
        x0, y0, w, h = COL_X, 14.0, 128.0, 44.0
        ax.add_patch(Rectangle((x0, y0), w, h, fill=False, lw=0.5 * PT))
        rows = [("PROJECT", "TALOS 5-axis arm"), ("PART", self.title), ("PART NO.", self.part_no),
                ("QTY / ARM", self.qty), ("MATERIAL", self.material),
                ("EST. MASS", f"{self.mass:.0f} g" if self.mass is not None else "-"),
                ("SCALE", self.scale_label), ("UNITS / REV", f"mm / {REV}"), ("DATE", TODAY),
                ("SHEET", f"{self.number} of {total}")]
        rh = h / 5
        for i, (k, v) in enumerate(rows):
            col, row = divmod(i, 5)
            cx = x0 + col * w / 2
            cy = y0 + h - (row + 1) * rh
            ax.add_patch(Rectangle((cx, cy), w / 2, rh, fill=False, lw=0.25 * PT))
            ax.text(cx + 1.2, cy + rh - 1.0, k, fontsize=4.2, va="top", color="0.35")
            ax.text(cx + 1.2, cy + 1.3, v, fontsize=6.0 if len(v) < 30 else 4.8, va="bottom",
                    weight="bold" if k in ("PART", "PART NO.") else "normal")
        # third-angle symbol
        sx, sy = x0 + w - 16, y0 + h + 3.5
        ax.add_patch(Polygon([(sx, sy - 2.2), (sx + 5, sy - 3.2), (sx + 5, sy + 3.2), (sx, sy + 2.2)],
                             fill=False, lw=0.3 * PT))
        for r in (1.6, 3.2):
            ax.add_patch(plt.Circle((sx + 11, sy), r, fill=False, lw=0.3 * PT))
        ax.text(sx - 2, sy, "THIRD ANGLE", fontsize=4.5, ha="right", va="center")
        # work-set tag, so a loose sheet says whose it is
        ws = work_set(self.part_no)
        tag = f"WORK SET {ws}: {WORK_SETS[ws][1]}" if ws else "REFERENCE: in work sets A and B"
        ax.add_patch(Rectangle((x0, sy - 3.0), 66, 6.0, fill=ws is not None, fc="0.88",
                               lw=0.3 * PT))
        ax.text(x0 + 1.5, sy, tag, fontsize=5.6 if ws else 5.2, va="center",
                weight="bold" if ws else "normal")


# ---------------------------------------------------------------- shared feature text

def servo_place(origin, shaft, body_x, body_y):
    return (f"Servo pocket P (sheet TAL-001) with the servo frame origin at ({', '.join(fmt(v) for v in origin)}); "
            f"servo +Z (shaft) -> {shaft}, servo +X -> {body_x}, servo +Y -> {body_y}.")


def case_screw_text(where):
    return (f"Case screw holes (sheet TAL-002): {where}. Each is a dia 2.4 hole from the servo face to the "
            "seat, then a dia 4.2 counterbore from the seat outward through the part.")


def octagon_text(w, h, c):
    return f"{fmt(w)} wide x {fmt(h)} tall, corners chamfered {fmt(c)}"


def arm_windows(L, h0, h1, w_mid, h_mid):
    """Hex window vertices in (x, z), the same rule as talos_arm.arm_link."""
    x0, x1 = 20.0, L - 36.0
    xm = (x0 + x1) / 2
    st = ((x0, h0, 6.0), (xm, h_mid, 5.0), (x1, h1, 6.0))
    wins = []
    for (xa, ha, ca), (xb, hb, cb), ia, ib in ((*st[:2], 8, 3), (*st[1:], 3, 4)):
        def half(x):
            t = (x - xa) / (xb - xa)
            return (ha + t * (hb - ha)) / 2 - (ca + t * (cb - ca)) - 1.5
        a, b = xa + ia, xb - ib
        ka, kb = half(a), half(b)
        wins.append([(a, 0), (a + ka, half(a + ka)), (b - kb, half(b - kb)), (b, 0),
                     (b - kb, -half(b - kb)), (a + ka, -half(a + ka))])
    return xm, wins


def pts_text(pts):
    return " ".join(f"({fmt(x)},{fmt(z)})" for x, z in pts)


# ---------------------------------------------------------------- sheets

def sheet_assembly(n, total):
    s = Sheet(n, "General assembly and parts list", "TAL-000", slug="general-assembly", qty="-", material="-",
              frame="world: base bottom centre, Z up, arm reaches +X")
    zero = cq.Compound.makeCompound([sh for _, sh, _ in ta.flatten(ta.build(ta.POSES["zero"]))])
    ready = cq.Compound.makeCompound([sh for _, sh, _ in ta.flatten(ta.build(ta.POSES["ready"]))])
    s.views(zero, hidden=False, iso_shape=ready, gap_right=26)
    h = [("pan horn face", ta.H_PAN), ("shoulder axis", ta.H_PAN + ta.H_SH),
         ("elbow axis", ta.H_PAN + ta.H_SH + ta.L1), ("wrist flex axis", ta.H_PAN + ta.H_SH + ta.L1 + ta.L2),
         ("roll horn face", ta.H_PAN + ta.H_SH + ta.L1 + ta.L2 + ta.X_ROLL)]
    for k, (lab, z) in enumerate(h, start=1):
        s.dim("front", (66, 0, 0), (24, 0, z), "v", "right", level=k)
        if k > 1:
            s.center("front", (0, 0, z), 3)
    s.dim("front", (-100, 0, 0), (66, 0, 0), "h", "below")
    s.dim("top", (0, -66, 0), (0, 66, 0), "v", "left", text="132 (plinth, across corners)")
    bom = [
        f"TAL-101  Base ........................ 1   PLA   set {work_set('TAL-101')}",
        f"TAL-102  Shoulder (turntable+cradle) . 1   PLA   set {work_set('TAL-102')}",
        f"TAL-103  Upper arm ................... 1   PLA   set {work_set('TAL-103')}",
        f"TAL-104  Forearm ..................... 1   PLA   set {work_set('TAL-104')}",
        f"TAL-105  Wrist ....................... 1   PLA   set {work_set('TAL-105')}",
        f"TAL-106  Gripper body ................ 1   PLA   set {work_set('TAL-106')}",
        f"TAL-107  Moving jaw .................. 1   PLA   set {work_set('TAL-107')}",
        f"TAL-108  Hub cap, large .............. 3   PLA   set {work_set('TAL-108')}",
        f"TAL-109  Hub cap, small .............. 3   PLA   set {work_set('TAL-109')}",
        "-",
        "Feetech STS3215 servo (12 V) ......... 6   bought",
        "Waveshare Bus Servo Adapter (A) ...... 1   bought",
        "M3 x 6 button head (servo horns) .... 24   bought",
        "M3 x 8 button head (passive horns) ... 3   bought",
        "M2 x 5 self-tapping (servo cases) ... 12   bought",
        "M2 x 6 self-tapping (servo cases) ... 8   bought",
        "M2.5 x 6 self-tapping (board) ........ 4   bought",
        "M4 bolts to the table (user's) ....... 4   bought",
        "3-pin bus servo cables ............... 6   with servos",
    ]
    layout = [
        f"Pan axis: vertical through the base centre. Pan horn face {fmt(ta.H_PAN)} above the table.",
        f"Shoulder axis (along Y): {fmt(ta.H_SH)} above the pan horn face, on the pan axis.",
        f"Upper arm {fmt(ta.L1)}, forearm {fmt(ta.L2)} (axis to axis). Wrist flex axis to roll horn "
        f"face {fmt(ta.X_ROLL)}. Jaw pivot {fmt(ta.XJ)} ahead of the roll horn face, {fmt(ta.ZJ)} "
        "above the roll axis.",
        "Clevis plates on every pitch joint: output plate +Y (y 18.9..22.9), idler plate -Y (y -22.9..-18.9).",
        "Front view: zero pose (all joints 0, arm straight up). Isometric: 'ready' pose.",
    ]
    y = s.features("PARTS LIST (set = work set, A or B)", bom)
    y = s.features("JOINT LAYOUT", layout, y_top=y - 2)
    s.notes(GENERAL[:1] + ["Part sheets give each part in its own frame; this sheet shows where those frames sit."], y - 2)
    s.title_block(total)
    return s


def sheet_servo(n, total):
    s = Sheet(n, "STS3215 servo + pocket envelope (reference)", "TAL-001", slug="servo-pocket-reference", qty="6", material="bought part",
              frame="servo: origin on shaft axis at mid-plane, +Z = shaft")
    body, horns, plugs = ta.sts3215()
    servo = body.union(horns).union(plugs)
    s.views(servo)
    s.dim("front", (-35.2, 0, -14.4), (10.2, 0, -14.4), "h", "below", 1)
    s.dim("front", (0, 0, -18.3), (10.2, 0, -15.9), "h", "below", 2)
    s.dim("front", (-35.2, 0, 0), (-14.1, 0, -26.0), "v", "left", 1)
    s.dim("front", (-35.2, 0, -15.9), (-35.2, 0, 15.9), "v", "left", 2)
    s.dim("front", (-10, 0, -18.3), (-10, 0, 18.7), "v", "left", 3)
    s.dim("top", (-35.2, -12.4, 0), (-35.2, 12.4, 0), "v", "left", 1)
    s.center("front", (0, 0, 0), 3)
    s.center("top", (0, 0, 0), 11)
    s.note("top", (6.93 * math.cos(math.radians(45)), 6.93 * math.sin(math.radians(45)), 0),
           "horn: 4x dia 2.5 (M3) on PCD 13.86, 45 deg", 14, 12)
    s.note("front", (-14.1, 0, -26.0), "mated bus plugs", -12, -6)
    feats = [
        "Body: box x -35.2..10.2, y -12.4..12.4, z -14.4..14.4 (45.4 x 24.8 x 28.8). +X end vertical "
        "corners chamfered 3, -X end corners R1.",
        "Caps: 44.6 x 24.4 x 1.5, corners R1.5, centred on x = -12.5, at z 14.4..15.9 and -15.9..-14.4.",
        "Back pad (connector face): x -29.9..-19.9, y -9.2..9.2, z -17.7..-15.9.",
        "Bus connector headers: x -16.6..-11.6, y 0.6..9.8 and -9.8..-0.6, z -17.4..-15.9. Mated plugs: "
        "x -16.9..-11.3, y 0.4..10.0 and -10.0..-0.4, z -26.0..-17.4.",
        "Spline boss dia 9, z 14.7..16.2. Idler boss dia 6, z -18.3..-14.4, tapped dia 2.5 (M3).",
        "Output horn dia 20, z 16.2..18.7: 4x dia 2.5 (M3) on PCD 13.86 at 45/135/225/315 deg, centre dia 3.2.",
        "Passive horn dia 20, z -17.7..-15.9, centre dia 7.",
        "Case holes dia 1.5 x 4 deep (M2 self-tapping): shaft face at (x, y) = (-29.0, +/-10.2); "
        "connector face at (-32.8, +/-10.2).",
        "* SERVO POCKET P (cut this into a part, in the servo frame; 0.3 clearance included):",
        "- a) box x -35.5..10.5, y -12.7..12.7, z -16.2..16.2",
        "- b) back pad box x -30.2..-19.6, y -9.5..9.5, z -18.0..-15.5",
        "- c) plug bay box x -17.6..-10.6, y -10.6..10.6, z -28.5..-15.5",
        "- d) idler clearance cylinder dia 21.6, z -18.5..-15.0, on the shaft axis",
        "- e) horn clearance cylinder dia 21.6, z 15.0..19.5, on the shaft axis",
    ]
    y = s.features("FEATURES (servo frame)", feats)
    s.notes(["Measured from the SO-ARM100 STS3215 reference model and Feetech's drawing. Check the "
             "connector position and case-hole depth on a real servo.",
             "Each part sheet says where its pocket's servo frame sits and how it is rotated."], y - 2)
    s.title_block(total)
    return s


def sheet_common(n, total):
    s = Sheet(n, "Common features: clevis, clip, screw holes", "TAL-002", slug="common-features", qty="-",
              frame="link: origin on joint axis, Y = axis")
    s.views(ta.clevis())
    for v in ("front",):
        s.center(v, (0, 0, 0), 16)
    s.dim("front", (0, 0, -15), (30, 0, -15), "h", "below", 1)
    s.dim("front", (-16, 0, -15), (-16, 0, 15), "v", "left", 1)
    s.note("front", (16 * math.cos(math.radians(-120)), 0, 16 * math.sin(math.radians(-120))),
           "R16 output hub (+Y plate)", -6, -20)
    s.note("front", (10.5 * math.cos(math.radians(-60)), 0, 10.5 * math.sin(math.radians(-60))),
           "R10.5 idler hub (-Y plate)", 16, -34)
    s.dim("top", (-16, ta.CLV_IN, 0), (-16, ta.CLV_OUT, 0), "v", "left", 1)
    s.dim("top", (-16, -ta.CLV_IN, 0), (-16, ta.CLV_IN, 0), "v", "left", 2)
    s.dim("top", (-16, -ta.CLV_OUT, 0), (-16, ta.CLV_OUT, 0), "v", "left", 3)
    feats = [
        "* CLEVIS (every pitch joint; drawn for the arm links: reach R = 30, half-height H = 15; "
        "wrist uses R = 24, H = 12). Plates are 4 thick, normal to Y.",
        "Output plate, y 18.9..22.9: circle R16 at the origin + rectangle x 0..R, z -H..H.",
        "Output plate holes, through: centre dia 7 (horn spline screw access) + 4x dia 3.2 at "
        "(x, z) = (+/-4.9, +/-4.9) (PCD 13.86, 45 deg).",
        "Idler plate, y -22.9..-18.9: circle R10.5 + neck polygon (0,10.5) (6,8.5) (0.6R,8.5) (R,H) "
        "(R,-H) (0.6R,-8.5) (6,-8.5) (0,-10.5). The neck clears the parent servo's bus plugs.",
        "Idler plate: pad dia 10 on the inner face, y -18.9..-18.35; centre hole dia 3.4 through.",
        "Clevis gap in the link body: remove a cylinder R21 about Y between y -18.9..18.9, and "
        "everything at x < 0 between those planes.",
        "* CASE SCREW HOLE (servo case, M2 self-tapping): dia 2.4 from the servo face to the seat, "
        "dia 4.2 counterbore from the seat outward. Seat = distance from the servo mid-plane: arm "
        "shells and gripper 17.3, shoulder 18.5, base 18.4, wrist 18.0.",
        "* HORN SCREW PATTERN: 4x on PCD 13.86 at 45/135/225/315 deg + centre. Clearance dia 3.2 "
        "(M3); counterbore dia 6.5 where the head sits inside a part.",
        "* CABLE CLIP (one bus cable; local u = along cable, v = across, w = out of the face): block "
        "u -3..3, v -5..5, w -1..5 (1 mm sunk into the face); tunnel u through, v -2.5..2.5, "
        "w 0.8..4.3; slit v -0.75..0.75, w 4.2..6; chamfer 1 on the two outer long edges.",
        "* HEX WINDOW H(xa, xb, zc, k): hexagon (xa,zc) (xa+k,zc-k) (xb-k,zc-k) (xb,zc) (xb-k,zc+k) "
        "(xa+k,zc+k), cut through the part along Y.",
    ]
    y = s.features("COMMON FEATURES", feats)
    s.notes(GENERAL, y - 2)
    s.title_block(total)
    return s


def sheet_base(n, total):
    parts, _ = ta.base_parts()
    s = Sheet(n, "Base", "TAL-101", slug="base", frame="origin plinth bottom centre, Z up, +X = arm front")
    s.views(parts["base"])
    s.mass = printed_mass(as_shape(parts["base"]))
    top = ta.H_PAN - 2.8
    s.dim("front", (-100, 0, 0), (-100, 0, 10), "v", "left", 1)
    s.dim("front", (-100, 0, 0), (-40, 0, top), "v", "left", 2)
    s.note("front", (40, 0, top - 8), "column dia 80", 14, 6)
    s.note("front", (45, 0, 12), "collar dia 90 x 4", 18, 10)
    s.dim("top", (-85.5, 18.5, 0), (-57.5, 18.5, 0), "h", "above", 1)
    s.dim("top", (-57.5, 18.5, 0), (0, 18.5, 0), "h", "above", 2)
    s.dim("top", (-100, 0, 0), (66, 0, 0), "h", "above", 3)
    s.dim("top", (-88, -32, 0), (-88, 32, 0), "v", "left", 1)
    s.dim("top", (-85.5, -18.5, 0), (-85.5, 18.5, 0), "v", "left", 2)
    s.center("top", (0, 0, 0), 40)
    a = math.radians(45)
    s.note("top", (54 * math.cos(a), 54 * math.sin(a), 0), "4x dia 4.4 thru, cbore dia 8.4 x 3.5, on R54", 10, 8)
    s.note("top", (41.4 * math.cos(math.radians(-50)), 41.4 * math.sin(math.radians(-50)), 0),
           "18x flute R2.6 on R41.4", 12, -10)
    feats = [
        "Plinth: regular 12-gon, circumscribed dia 132 (a corner on +X), extrude z 0..10.",
        "Board deck: polygon (-40,-32) (-88,-32) (-100,-20) (-100,20) (-88,32) (-40,32), extrude z 0..10; "
        "union with the plinth.",
        "Chamfer 2.5 on all top outer edges (z = 10).",
        "Mounting holes: 4x dia 4.4 through on R54 at 45/135/225/315 deg; counterbore dia 8.4 from z 6.5 up.",
        "Collar: cylinder dia 90, z 10..14; chamfer 2 on its top edge.",
        f"Column: cylinder dia 80, z 10..{fmt(top)}; chamfer 1.5 on its top edge.",
        "Board standoffs: 4x cylinder dia 6.4, z 9..15, at (-57.5,+/-18.5) and (-85.5,+/-18.5); "
        "pilot dia 2.1 from z 8 to 16 (M2.5 self-tapping).",
        "Flutes: 18x vertical cylinder dia 5.2 on R41.4 at 10 deg + 20 deg steps, z 19..52, with "
        "dia 5.2 spheres at both ends; cut.",
        servo_place((0, 0, ta.H_PAN - 18.7), "+Z", "+X", "+Y"),
        f"Top opening: cut box x -36..11, y -12.7..12.7, z {fmt(top - 1)}..{fmt(top + 5)}.",
        "Cable tunnel (plug bay -> board deck): cut box x -60..-13, y -6..6, z 14.8..24.5.",
        case_screw_text("connector face only, screws from below: (x, y) = (-32.8, +/-10.2); hole "
                        "z 24.9..27.5, counterbore z -1.7..24.9 (opens on the bottom)"),
    ]
    y = s.features("FEATURES (in order)", feats)
    s.notes(GENERAL + ["The pan servo drops in from the top; its plugs hang into the bay under it."], y - 2)
    s.title_block(total)
    return s


def sheet_shoulder(n, total):
    parts, _ = ta.shoulder_parts()
    s = Sheet(n, "Shoulder (turntable + cradle)", "TAL-102", slug="shoulder",
              frame="origin on pan axis at the turntable bottom (= pan horn face), Z up")
    s.views(parts["shoulder"])
    s.mass = printed_mass(as_shape(parts["shoulder"]))
    s.dim("front", (-37, 0, 0), (37, 0, 0), "h", "below", 1, prefix="dia ")
    s.dim("front", (-37, 0, 0), (-37, 0, 6), "v", "left", 1)
    s.dim("front", (-37, 0, 0), (-17, 0, ta.H_SH), "v", "left", 2)
    s.dim("front", (-37, 0, 0), (-17, 0, ta.H_SH + 4), "v", "left", 3)
    s.center("front", (0, 0, ta.H_SH), 6)
    s.dim("top", (-12.9, 18.4, 0), (12.9, 18.4, 0), "h", "above", 1)
    s.dim("top", (-17, 18.4, 0), (17, 18.4, 0), "h", "above", 2)
    s.dim("top", (-24, 25, 0), (24, 25, 0), "h", "above", 3)
    s.dim("top", (-24, -25, 0), (-24, 25, 0), "v", "left", 1)
    s.dim("top", (-17, -18.4, 0), (-17, 18.4, 0), "v", "left", 2)
    s.center("top", (0, 0, 0), 37)
    s.note("top", (30, 0, 0), "4x arc window R27-R33, 36 deg", 18, -14)
    feats = [
        "Turntable: cylinder dia 74, z 0..6; chamfer 2 top edge, 0.8 bottom edge.",
        "Cradle: box x -17..17, y -18.4..18.4, z 5..34; chamfer 3 on the 4 edges parallel to Y.",
        f"Posts: boxes x -17..-12.9 and 12.9..17, y -18.4..18.4, z 30..{fmt(ta.H_SH + 4)}.",
        "Cheeks: trapezoid (-24,6) (24,6) (14,21) (-14,21) in XZ, extruded y 18.9..25 and y -25..-18.9.",
        "Cable clip (TAL-002) on the back face: centre (-17, -9, 21), cable along Z, face normal -X.",
        servo_place((0, 0, ta.H_SH), "+Y", "+Z (body hangs down)", "+X"),
        "Servo opening: cut box x -12.7..12.7, y -16.2..16.2, z 10..80.",
        "Cable hole: dia 8 along Y, through all, at x = 0, z = 14.",
        "Pan horn: centre dia 7, z -1..11. 4x dia 3.2 at (+/-4.9, +/-4.9), z -1..4, counterbore "
        "dia 6.5 z 4..11 (M3 x 6 from inside the cradle).",
        "Turntable windows: 4x arc slot between R27 and R33, -18..+18 deg about 0/90/180/270 deg, through.",
        case_screw_text("seat 18.5, through the cheeks: shaft face at (x, z) = (+/-10.2, 17), y +15.9..+18.5; "
                        "connector face at (+/-10.2, 13.2), y -18.5..-15.9"),
    ]
    y = s.features("FEATURES (in order)", feats)
    s.notes(GENERAL, y - 2)
    s.title_block(total)
    return s


def sheet_arm(n, total, upper=True):
    L = ta.L1 if upper else ta.L2
    p = dict(h0=30.0, h1=30.8, w_mid=34.0, h_mid=25.0) if upper else \
        dict(h0=28.0, h1=30.8, w_mid=32.0, h_mid=24.0)
    body, _, _ = ta.arm_link(L, **p)
    name, no, axis = ("Upper arm", "TAL-103", "shoulder") if upper else ("Forearm", "TAL-104", "elbow")
    distal = "elbow" if upper else "wrist flex"
    s = Sheet(n, name, no, slug=name.lower().replace(" ", "-"),
              frame=f"origin on the {axis} axis, X to the {distal} axis, Y = axis")
    s.views(body)
    s.mass = printed_mass(as_shape(body))
    x0, x1 = 20.0, L - 36.0
    xm, wins = arm_windows(L, **p)
    s.dim("front", (0, 0, -16), (L, 0, -15.4), "h", "below", 2)
    s.dim("front", (L - 38.2, 0, -15.4), (L, 0, -15.4), "h", "below", 1)
    s.dim("front", (L - 20, 0, -15.4), (L - 20, 0, 15.4), "v", "right", 1)
    s.dim("top", (0, ta.CLV_OUT, 0), (x0, ta.CLV_OUT, 0), "h", "above", 1)
    s.dim("top", (x0, ta.CLV_OUT, 0), (xm, p["w_mid"] / 2, 0), "h", "above", 2)
    s.dim("top", (xm, p["w_mid"] / 2, 0), (x1, ta.SHELL_OUT, 0), "h", "above", 2)
    s.center("front", (0, 0, 0), 16)
    s.center("front", (L, 0, 0), 18)
    s.note("front", (L + 18 * math.cos(math.radians(40)), 0, 18 * math.sin(math.radians(40))),
           "R18 shell end", 8, 10)
    s.dim("top", (0, -ta.CLV_OUT, 0), (0, ta.CLV_OUT, 0), "v", "left", 1)
    s.dim("top", (L, -ta.SHELL_OUT, 0), (L, ta.SHELL_OUT, 0), "v", "right", 1)
    s.dim("top", (xm, -p["w_mid"] / 2, 0), (xm, p["w_mid"] / 2, 0), "v", "right", 2)
    feats = [
        "Clevis (TAL-002), reach 30, half-height 15.",
        f"Beam: ruled loft of YZ sections, x {fmt(x0)}: {octagon_text(45.8, p['h0'], 6)}; x {fmt(xm)}: "
        f"{octagon_text(p['w_mid'], p['h_mid'], 5)}; x {fmt(x1)}: {octagon_text(36.8, p['h1'], 6)}.",
        f"Servo shell: XZ polygon ({fmt(L - 34.2)},-15.4) ({fmt(L)},-15.4) ({fmt(L)},15.4) ({fmt(L - 34.2)},15.4) "
        f"({fmt(L - 38.2)},11.4) ({fmt(L - 38.2)},-11.4), extruded y -18.4..18.4; plus a cylinder R18 "
        f"about Y at x = {fmt(L)}, y -18.4..18.4.",
        "Spine: ruled loft of the ridge profile through the same three stations, then trimmed to "
        f"x {fmt(x0 + 8)}..{fmt(x1 - 4)}. Ridge profile at beam height h: (-4,h/2-3) (4,h/2-3) "
        "(4,h/2+0.2) (2.4,h/2+1.8) (-2.4,h/2+1.8) (-4,h/2+0.2).",
        f"Window 1, cut through Y: {pts_text(wins[0])}.",
        f"Window 2, cut through Y: {pts_text(wins[1])}.",
        f"Cable clip (TAL-002) on the -Y flank at the waist: centre ({fmt(xm)}, {fmt(-p['w_mid'] / 2 - 0.2)}, 0), "
        "cable along X.",
        "Clevis gap cut (TAL-002).",
        servo_place((L, 0, 0), "+Y", "+X", "-Z"),
        f"Servo opening (drops in from below): cut box x {fmt(L - 35.5)}..{fmt(L - 4)}, y -16.2..16.2, z -40..-12.",
        case_screw_text(f"seat 17.3; shaft face (+Y) at (x, z) = ({fmt(L - 29)}, +/-10.2); connector "
                        f"face (-Y) at ({fmt(L - 32.8)}, +/-10.2)"),
    ]
    y = s.features("FEATURES (in order)", feats)
    s.notes(GENERAL + [f"The {distal} servo's bus plugs come out through the -Y shell wall."], y - 2)
    s.title_block(total)
    return s


def sheet_wrist(n, total):
    parts, _ = ta.wrist_parts()
    xs = ta.X_ROLL - 18.7
    s = Sheet(n, "Wrist", "TAL-105", slug="wrist", frame="origin on the wrist flex axis, X along the roll axis, Y = flex axis")
    s.views(parts["wrist"])
    s.mass = printed_mass(as_shape(parts["wrist"]))
    s.dim("front", (20.5, 0, -38.5), (ta.X_ROLL - 2.8, 0, -38.5), "h", "below", 1)
    s.dim("front", (0, 0, 0), (ta.X_ROLL, 0, 0), "h", "below", 2)
    s.dim("front", (ta.X_ROLL - 2.8, 0, -38.5), (ta.X_ROLL - 2.8, 0, 13.6), "v", "right", 1)
    s.dim("top", (0, ta.CLV_OUT, 0), (20.5, ta.CLV_OUT, 0), "h", "above", 1)
    s.dim("top", (20.5, ta.CLV_OUT, 0), (31, ta.CLV_OUT, 0), "h", "above", 1)
    s.center("front", (0, 0, 0), 11)
    s.axis_line("front", (-12, 0, 0), (ta.X_ROLL + 6, 0, 0))
    s.dim("top", (0, -ta.CLV_OUT, 0), (0, ta.CLV_OUT, 0), "v", "left", 1)
    s.dim("top", (ta.X_ROLL - 2.8, -15, 0), (ta.X_ROLL - 2.8, 15, 0), "v", "right", 1)
    feats = [
        "Clevis (TAL-002), reach 24, half-height 12.",
        "Bridge lugs: box x 20.5..31, y -22.9..22.9, z -12..12; chamfer 3 on edges parallel to Y.",
        f"Cradle: box x 20.5..{fmt(ta.X_ROLL - 2.8)}, y -15..15, z -38.5..13.6; chamfer 4 on edges parallel to X.",
        "Cable clip (TAL-002) on the -Y face: centre (46, -15.2, -30), cable along X.",
        "Clevis gap cut (TAL-002).",
        servo_place((xs, 0, 0), "+X", "+Z (body hangs down)", "-Y"),
        f"Cradle interior: cut box x {fmt(xs - 16)}..80, y -12.7..12.7, z -36..11.",
        f"Plug bay exit to -Y: cut box x {fmt(xs - 28.5)}..{fmt(xs - 15.5)}, y -18.6..0, z -17.6..-10.6, "
        f"and box x {fmt(xs - 28.5)}..{fmt(xs - 15.5)}, y -30..0, z -17.6..-12.4.",
        f"Side windows: hex window H({fmt(xs - 8)}, 57, -12, 8) through Y (TAL-002).",
        case_screw_text(f"connector face only, seat 18.0: (y, z) = (+/-10.2, -32.8); hole x "
                        f"{fmt(xs - 18)}..{fmt(xs - 15.9)}, counterbore from x {fmt(xs - 18)} toward -X"),
    ]
    y = s.features("FEATURES (in order)", feats)
    s.notes(GENERAL + ["Fit the roll servo's case screws before the gripper goes on."], y - 2)
    s.title_block(total)
    return s


def sheet_gripper(n, total):
    parts, _ = ta.gripper_parts()
    s = Sheet(n, "Gripper body (with fixed jaw)", "TAL-106", slug="gripper-body",
              frame="origin on the roll axis at the roll horn face, X forward, Y = jaw pivot axis")
    s.views(parts["gripper"])
    s.mass = printed_mass(as_shape(parts["gripper"]))
    s.dim("front", (0, 0, -17), (108, 0, -6), "h", "below", 2)
    s.dim("front", (0, 0, -9.4), (ta.XJ, 0, ta.ZJ), "h", "below", 1)
    s.dim("front", (0, 0, 0), (ta.XJ, 0, ta.ZJ), "v", "left", 1)
    s.dim("front", (0, 0, ta.ZJ - 15.4), (0, 0, ta.ZJ + 15.4), "v", "left", 2)
    s.dim("front", (108, 0, -6), (108, 0, -0.5), "v", "right", 1)
    s.center("front", (ta.XJ, 0, ta.ZJ), 18)
    s.axis_line("front", (-6, 0, 0), (20, 0, 0))
    s.note("front", (87, 0, -0.5), "6x serration 3 wide x 1.6 deep, pitch 6", 6, 16)
    s.dim("top", (0, -ta.SHELL_OUT, 0), (0, ta.SHELL_OUT, 0), "v", "left", 1)
    s.dim("top", (108, -ta.FINGER_HW, 0), (108, ta.FINGER_HW, 0), "v", "right", 1)
    feats = [
        "Disc: cylinder dia 31 along X, x 0..4; chamfer 0.6 on the front edge.",
        f"Housing: box x 0..{fmt(ta.XJ)}, y -18.4..18.4, z {fmt(ta.ZJ - 15.4)}..{fmt(ta.ZJ + 15.4)}; "
        f"chamfer 3 on edges parallel to Y; plus cylinder R18 about Y at (x, z) = ({fmt(ta.XJ)}, {fmt(ta.ZJ)}).",
        "Keel, y -10..10: XZ polygon (24,-9.4) (56,-9.4) (56,-17) (30,-17) (24,-13).",
        "Fixed finger, y -10..10: (48,-9.4) (54,-9.4) (66,-0.5) (108,-0.5) (108,-6) (64,-17) (48,-17).",
        servo_place((ta.XJ, 0, ta.ZJ), "+Y", "+X", "-Z"),
        f"Servo opening (from the top): cut box x {fmt(ta.XJ - 35.5)}..{fmt(ta.XJ - 4)}, y -16.2..16.2, "
        f"z {fmt(ta.ZJ + 12)}..{fmt(ta.ZJ + 30)}.",
        f"Side windows: hex window H(9, 33, {fmt(ta.ZJ)}, 7) through Y (TAL-002).",
        "Grip serrations: 6x triangle (x,0) (x+1.5,-1.6) (x+3,0) for x = 72, 78 ... 102, through Y.",
        "Roll horn: centre dia 7, x -1..7. 4x dia 3.2 at (y, z) = (+/-4.9, +/-4.9), x -1..3.5, "
        "counterbore dia 6.5 x 3.5..7 (M3 x 6 from inside the housing).",
        case_screw_text("seat 17.3; shaft face (+Y) at (x, z) = (13, -4.2) and (13, 16.2); connector "
                        "face (-Y) at (9.2, -4.2) and (9.2, 16.2)"),
    ]
    y = s.features("FEATURES (in order)", feats)
    s.notes(GENERAL + ["The jaw servo's bus plugs come out through the -Y housing wall."], y - 2)
    s.title_block(total)
    return s


def sheet_jaw(n, total):
    jaw = ta.moving_jaw()
    s = Sheet(n, "Moving jaw", "TAL-107", slug="moving-jaw", frame="origin on the jaw pivot, X along the closed jaw, Y = pivot axis")
    s.views(jaw)
    s.mass = printed_mass(as_shape(jaw))
    s.dim("front", (0, 0, -13), (64, 0, -5.5), "h", "below", 2)
    s.dim("front", (0, 0, -13), (22, 0, -5.5), "h", "below", 1)
    s.dim("front", (64, 0, -5.5), (64, 0, -1.5), "v", "right", 1)
    s.dim("front", (64, 0, -5.5), (30, 0, 10), "v", "right", 2)
    s.center("front", (0, 0, 0), 13)
    s.dim("top", (64, -ta.FINGER_HW, 0), (64, ta.FINGER_HW, 0), "v", "right", 1)
    s.dim("top", (0, ta.CLV_IN, 0), (0, ta.CLV_OUT, 0), "v", "left", 1)
    s.dim("top", (0, -ta.FINGER_HW, 0), (0, ta.CLV_OUT, 0), "v", "left", 2)
    feats = [
        "Plate, y 18.9..22.9: circle R13 + polygon (0,-13) (34,-5.5) (34,9.1) (0,13).",
        "Finger, y -10..10: (22,-5.5) (64,-5.5) (64,-1.5) (52,5) (30,10) (22,9).",
        "Knuckle, y 10..18.9: (22,-5.5) (34,-5.5) (34,9.1) (30,10) (22,9).",
        "Grip serrations: 6x triangle (x,-5.5) (x+1.5,-3.9) (x+3,-5.5) for x = 30, 36 ... 60, through Y.",
        "Horn holes through the plate: centre dia 7 + 4x dia 3.2 at (x, z) = (+/-4.9, +/-4.9).",
    ]
    y = s.features("FEATURES (in order)", feats)
    s.notes(GENERAL + ["Bolts to the jaw servo's output horn with 4x M3 x 6; closed position is "
                       "jaw X along the gripper X."], y - 2)
    s.title_block(total)
    return s


def sheet_cap(n, total, large=True):
    cap = ta.hub_cap(large)
    r, recess, groove, boss = (12.5, 10.4, 9.0, 6.0) if large else (10.2, 3.4, 7.0, 4.5)
    name, no, slug, side = (("Hub cap, large", "TAL-108", "hub-cap-large", "output side, covers 4 horn screws")
                            if large else
                            ("Hub cap, small", "TAL-109", "hub-cap-small", "idler side, covers 1 centre screw"))
    s = Sheet(n, name, no, slug=slug, qty="3", frame="origin centre of the inner face, +Y outward")
    s.views(cap)
    s.mass = printed_mass(as_shape(cap))
    s.dim("front", (-r, 0, 0), (r, 0, 0), "h", "below", 1, prefix="dia ")
    s.dim("front", (-recess, 0, 0), (recess, 0, 0), "h", "below", 2, prefix="recess dia ")
    s.dim("top", (-r, 0, 0), (-r, 3.4, 0), "v", "left", 1)
    s.dim("top", (-r, 0, 0), (-r, 1.8, 0), "v", "left", 2)
    s.center("front", (0, 0, 0), r)
    feats = [
        f"* {name.upper()} ({side}):",
        f"Disc dia {fmt(2 * r)}, y 0..3.4; chamfer 0.8 on the outer edge.",
        f"Recess dia {fmt(2 * recess)}, y 0..1.8 (clears M3 button heads, 1.65 high).",
        f"Face groove: annulus between dia {fmt(2 * boss)} and dia {fmt(2 * groove)}, 0.4 deep (y 3.0..3.4).",
    ]
    y = s.features("FEATURES", feats)
    s.notes(GENERAL + ["Press-fit or glue onto the clevis plate over the screw heads. Print face down."], y - 2)
    s.title_block(total)
    return s


SHEETS = [sheet_assembly, sheet_servo, sheet_common, sheet_base, sheet_shoulder,
          lambda n, t: sheet_arm(n, t, True), lambda n, t: sheet_arm(n, t, False),
          sheet_wrist, sheet_gripper, sheet_jaw,
          lambda n, t: sheet_cap(n, t, True), lambda n, t: sheet_cap(n, t, False)]


def main():
    """Write the full set and the two work sets as PDFs, plus a PDF and a PNG of every sheet.

    Sheet numbers ("sheet n of 12") follow the full set in every file, so a sheet is
    identical wherever it appears.
    """
    sheets_dir, preview_dir, sets_dir = OUT / "sheets", OUT / "previews", OUT / "work-sets"
    for d in (OUT, sheets_dir, preview_dir, sets_dir):
        d.mkdir(exist_ok=True)
    total = len(SHEETS)
    full_path = OUT / f"TAL-000_drawing-set_rev-{REV}.pdf"
    set_paths = {k: sets_dir / f"TAL-SET-{k}_{slug}_rev-{REV}.pdf" for k, (slug, _, _) in WORK_SETS.items()}
    pdfs = {k: PdfPages(p) for k, p in set_paths.items()}
    work = {k: [0, 0] for k in WORK_SETS}            # parts, feature steps
    try:
        with PdfPages(full_path) as full:
            for i, make in enumerate(SHEETS, start=1):
                s = make(i, total)
                stem = f"{s.part_no}_{s.slug}_rev-{REV}"
                ws = work_set(s.part_no)
                full.savefig(s.fig)
                for k in ([ws] if ws else WORK_SETS):
                    pdfs[k].savefig(s.fig)
                if ws:
                    work[ws][0] += 1
                    work[ws][1] += s.n_features
                s.fig.savefig(sheets_dir / f"{stem}.pdf")
                s.fig.savefig(preview_dir / f"{stem}.png", dpi=110)
                plt.close(s.fig)
                print(f"sheet {i}/{total}: {stem}  [{'set ' + ws if ws else 'both sets'}]")
    finally:
        for pdf in pdfs.values():
            pdf.close()
    print(f"wrote {full_path}")
    for k, p in set_paths.items():
        print(f"wrote {p}  ({work[k][0]} parts, {work[k][1]} modelling steps)")


if __name__ == "__main__":
    main()
