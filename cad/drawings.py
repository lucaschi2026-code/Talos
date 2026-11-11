"""Technical drawing set for Talos, generated from the CAD in talos_arm.py.

One or two A3 sheets per printed part, plus an assembly / BOM sheet, the STS3215
reference sheet and the common-features sheets (clevis, clip, screw holes).
Every feature is dimensioned on a view: third-angle front / top / right views,
section views where depth matters, hole callouts, and tabulated coordinates for
polygon outlines. The numbered feature list gives the modelling order.

    python drawings.py      # full set, the two work sets, and a PDF + PNG per sheet

All dimensions come from talos_arm.py, so the drawings follow the model.
Lettering is ISOCPEUR (ISO 3098) when it is installed, with GENISO for the
counterbore and depth symbols; otherwise DejaVu Sans.
"""

import datetime
import math
import os
from pathlib import Path

import numpy as np
import cadquery as cq
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager, rcParams
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.collections import LineCollection
from matplotlib.font_manager import FontProperties
from matplotlib.patches import PathPatch, Polygon, Rectangle
from matplotlib.path import Path as MPath
from matplotlib.textpath import TextToPath
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepLib import BRepLib
from OCP.BRepTools import BRepTools_WireExplorer
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.TopAbs import TopAbs_EDGE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

import talos_arm as ta

# Servo case screw holes (servo frame): x of the top-face pair, of the bottom-face pair and of the
# bottom-face pair beside the shaft, and their half spacing in y.
HXF, HXB, HXS, HY = ta.CASE_FRONT[0][0], ta.CASE_BACK[0][0], ta.CASE_BACK_SHAFT[0][0], ta.CASE_FRONT[0][1]
ZB = ta.H_SH + HXB                                # shoulder: height of the -Y cheek's case screws
HK = round(ta.HORN_PCD_R / math.sqrt(2), 2)       # horn hole offset along each axis: 4.95

# ---------------------------------------------------------------- lettering

FONT_FILES = (("ISOCPEUR", "isocpeur.ttf"), ("GENISO", "GENISO.ttf"))  # ISO 3098; GENISO has the ⌴ ↧ glyphs
FONT_DIRS = (Path(os.environ.get("WINDIR", r"C:\Windows")) / "Fonts",
             Path(os.environ.get("LOCALAPPDATA", "")) / "Microsoft" / "Windows" / "Fonts")
FONTS = []
for _fam, _fn in FONT_FILES:
    for _d in FONT_DIRS:
        if (_d / _fn).exists():
            font_manager.fontManager.addfont(str(_d / _fn))
            FONTS.append(_fam)
            break
if "ISOCPEUR" not in FONTS:
    print("warning: ISOCPEUR not installed, lettering falls back to DejaVu Sans")
rcParams["font.family"] = FONTS + ["DejaVu Sans"]
rcParams["pdf.fonttype"] = 42
rcParams["hatch.linewidth"] = 0.4
rcParams["hatch.color"] = "0.3"
DIA = "\u2300"                                                        # ⌀
CBORE, DEPTH = ("\u2334", "\u21a7") if "GENISO" in FONTS else ("CBORE ", "DP ")
_T2P = TextToPath()
_FP = {}


def text_w(s, size):
    """Width of a line of text on paper, mm."""
    fp = _FP.get(size)
    if fp is None:
        fp = _FP[size] = FontProperties(family=rcParams["font.family"], size=size)
    return _T2P.get_text_width_height_descent(s, fp, ismath=False)[0] / PT if s else 0.0


# ---------------------------------------------------------------- sheet layout

OUT = Path(__file__).with_name("drawings")
PAPER = (420.0, 297.0)                      # A3 landscape, mm
REGION = (14.0, 14.0, 272.0, 283.0)         # view area (x0, y0, x1, y1)
COL_X = 278.0                               # right column: feature list + title block
PT = 72 / 25.4                              # points per mm
DIM_PT, NOTE_PT = 16.0, 12.0                # dimension and leader-note text size, points
LABEL_PT = 12.0                             # view labels
DIM_STEP = DIM_PT / PT + 6.0                # spacing between dimension levels, mm
# Dimensions go on the outer sides of the standard views. Margins leave room for 3 levels.
M_LEFT, M_BOTTOM, M_TOP, M_RIGHT = (3 * DIM_STEP + 6, 3 * DIM_STEP + 12, 3 * DIM_STEP + 6,
                                    2 * DIM_STEP + 6)
GAP_FT, GAP_FR = 20.0, 32.0                 # front -> top, front -> right
SCALES = [(10, "10:1"), (8, "8:1"), (5, "5:1"), (4, "4:1"), (3, "3:1"), (2, "2:1"), (1.5, "3:2"),
          (1, "1:1"), (2 / 3, "2:3"), (0.5, "1:2"), (0.4, "1:2.5"), (1 / 3, "1:3"), (0.25, "1:4"),
          (0.2, "1:5")]
SCALE_NAME = {sc: lab for sc, lab in SCALES}
VIEWS = {"front": ((0, -1, 0), (1, 0, 0)), "top": ((0, 0, 1), (1, 0, 0)),
         "right": ((1, 0, 0), (0, 1, 0)), "iso": ((1, -1, 1), (1, 1, 0)),
         "bottom": ((0, 0, -1), (1, 0, 0)), "rear": ((0, 1, 0), (-1, 0, 0)),
         "left": ((-1, 0, 0), (0, -1, 0))}
LABELS = {"front": "FRONT", "top": "TOP", "right": "RIGHT", "iso": "ISOMETRIC (NOT TO SCALE)",
          "bottom": "BOTTOM", "rear": "REAR", "left": "LEFT"}
TODAY = datetime.date.today().isoformat()
REV = "A"                                   # bump when a released drawing changes
# Right column: the feature list is set as large as fits, from COL_PT_MAX down to COL_PT_MIN.
COL_PT_MAX, COL_PT_MIN = 8.6, 6.0
COL_W = 128.0                               # right column width, mm
TB_H = 62.0                                 # title block height, mm (incl. the work-set row)
COL_FLOOR = 14.0 + TB_H + 12.0              # lowest y the column text may reach (above the symbol)
ARROW = 3.6                                 # arrowhead length, mm
CHAIN = (0, (6, 1.5, 1, 1.5))               # centre / cutting-plane line pattern

# Part numbering: TAL-0xx = assembly and reference sheets, TAL-1xx = printed parts,
# numbered base to tip. Files are named <part no>_<slug>_rev-<rev>.

# The parts are split into two work sets so two people can model them in parallel.
# The reference sheets (TAL-0xx) go in both. The split is balanced on modelling steps
# (A 45, B 47). The sets only meet at the elbow (upper arm shell -> forearm clevis), and
# both sides of it are defined on the shared sheets (servo pocket P, clevis).
WORK_SETS = {
    "A": ("base-to-upper-arm", "base, shoulder, upper arm, hub caps, fit gauge",
          ["TAL-101", "TAL-102", "TAL-103", "TAL-108", "TAL-111"]),
    "B": ("forearm-to-gripper", "forearm, wrist, gripper, moving jaw, idler plates",
          ["TAL-104", "TAL-105", "TAL-106", "TAL-107", "TAL-110"]),
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



def fmt(v):
    s = f"{v:.2f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def unit(v):
    v = np.array(v, float)
    return v / np.linalg.norm(v)


def proj(dirs, p):
    """Model point -> view-plane coordinates for a view looking along -n with screen x = x."""
    n, x = (unit(a) for a in dirs)
    y = np.cross(n, x)
    p = np.array(p, float)
    return np.array([p @ x, p @ y])


def edge_pts(edge, defl=0.03):
    d = GCPnts_QuasiUniformDeflection(BRepAdaptor_Curve(edge), defl)
    if not d.IsDone() or d.NbPoints() < 2:
        return None
    return np.array([(d.Value(i).X(), d.Value(i).Y(), d.Value(i).Z()) for i in range(1, d.NbPoints() + 1)])


def polylines(shape):
    out = []
    if shape is None or shape.IsNull():
        return out
    BRepLib.BuildCurves3d_s(shape)
    exp = TopExp_Explorer(shape, TopAbs_EDGE)
    while exp.More():
        try:
            pts = edge_pts(TopoDS.Edge_s(exp.Current()))
            if pts is not None:
                out.append(pts[:, :2])
        except Exception:
            pass
        exp.Next()
    return out


def hlr(shape, dirs, hidden=True):
    n, x = dirs
    algo = HLRBRep_Algo()
    algo.Add(shape.wrapped)
    algo.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*unit(n)), gp_Dir(*unit(x)))))
    algo.Update()
    algo.Hide()
    h = HLRBRep_HLRToShape(algo)
    vis = polylines(h.VCompound()) + polylines(h.OutLineVCompound())
    hid = (polylines(h.HCompound()) + polylines(h.OutLineHCompound())) if hidden else []
    return vis, hid


def wire_pts(wire):
    """Ordered 3D points round a wire."""
    out = []
    exp = BRepTools_WireExplorer(wire.wrapped)
    while exp.More():
        e = exp.Current()
        pts = edge_pts(e, 0.02)
        if pts is not None:
            if e.Orientation() == TopAbs_REVERSED:
                pts = pts[::-1]
            out.extend(pts if not out else pts[1:])
        exp.Next()
    return np.array(out)


def extents(lines):
    pts = np.vstack(lines)
    return pts.min(axis=0), pts.max(axis=0)


def printed_mass(shape):
    V, A = shape.Volume(), shape.Area()
    shell = min(V, A * T_SHELL)
    return RHO * (shell + INFILL * (V - shell))


def as_shape(obj):
    return obj.val() if isinstance(obj, cq.Workplane) else obj


def fit_scale(w, h, w_free, h_free, max_scale=10):
    for sc, lab in SCALES:
        if sc <= max_scale and sc * w <= w_free and sc * h <= h_free:
            return sc
    return SCALES[-1][0]


# ---------------------------------------------------------------- sheet

class Sheet:
    def __init__(self, title, part_no, slug, qty="1", material="PLA, 20 % infill, 3 perimeters",
                 frame="", page=1, pages=1):
        self.title, self.part_no, self.slug = title, part_no, slug
        self.qty, self.material, self.frame = qty, material, frame
        self.page, self.pages = page, pages
        self.number = self.total = None
        self.fig = plt.figure(figsize=(PAPER[0] / 25.4, PAPER[1] / 25.4))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        ax = self.ax
        ax.set_xlim(0, PAPER[0])
        ax.set_ylim(0, PAPER[1])
        ax.set_aspect("equal")
        ax.axis("off")
        ax.add_patch(Rectangle((10, 10), 400, 277, fill=False, lw=0.7 * PT))
        self.place = {}   # view -> (offset, scale, model min, paper bbox)
        self.dirs = {}    # view -> (n, x)
        self.labels = {}  # view -> label text
        self.low = {}     # view -> lowest paper y used (view or its dimensions), for the label
        self.scale_label = "-"
        self.mass = None
        self.n_features = 0
        self.column = []      # right-column blocks, laid out in title_block() once all are known
        self.note_texts = []  # (view, text) of leader notes, for the layout check

    # ---- views
    def views(self, shape, hidden=True, iso=True, iso_shape=None, region=REGION, gap_right=0.0,
              scale=None, right=True, iso_scale=1.0, gap_ft=GAP_FT):
        """Front, top and (optionally) right views in third-angle layout, plus an isometric."""
        shape = as_shape(shape)
        names = ("front", "top", "right") if right else ("front", "top")
        data = {v: hlr(shape, VIEWS[v], hidden) for v in names}
        ext = {v: extents(data[v][0]) for v in data}
        size = {v: ext[v][1] - ext[v][0] for v in data}
        x0, y0, x1, y1 = region
        w_need = size["front"][0] + (size["right"][0] if right else 0)
        h_need = size["front"][1] + size["top"][1]
        w_free = x1 - x0 - M_LEFT - (GAP_FR if right else 0) - M_RIGHT - gap_right
        h_free = y1 - y0 - M_BOTTOM - gap_ft - M_TOP
        s = scale or fit_scale(w_need, h_need, w_free, h_free)
        self.scale_label = SCALE_NAME[s]
        left, bottom = x0 + M_LEFT, y0 + M_BOTTOM
        origin = {"front": (left, bottom),
                  "top": (left, bottom + s * size["front"][1] + gap_ft),
                  "right": (left + s * size["front"][0] + GAP_FR + gap_right, bottom)}
        for v in names:
            mn = ext[v][0].copy()
            if v == "top":
                mn[0] = ext["front"][0][0]
            if v == "right":
                mn[1] = ext["front"][0][1]
            self.dirs[v] = VIEWS[v]
            self.labels[v] = LABELS[v]
            self._draw(v, data[v], origin[v], s, mn)
        if iso:
            cx0 = left + s * size["front"][0] + 2 * DIM_STEP + 4   # clear of the top view's right dims
            cy0 = max(origin["top"][1], bottom + s * size["front"][1] + 8)
            if x1 - cx0 < 60:                                # no room beside the top view: go above it
                cx0, cy0 = origin["right"][0], origin["top"][1]
            self.iso(iso_shape if iso_shape is not None else shape, (cx0, cy0, x1 - 4, y1 - 8), iso_scale * s)
        return s

    def iso(self, shape, box, max_scale=10.0):
        vis, _ = hlr(as_shape(shape), VIEWS["iso"], hidden=False)
        mn, mx = extents(vis)
        cx0, cy0, cx1, cy1 = box
        cw, ch = cx1 - cx0, cy1 - cy0
        si = min(0.9 * min(cw / (mx - mn)[0], ch / (mx - mn)[1]), max_scale)
        off = (cx0 + (cw - si * (mx - mn)[0]) / 2, cy0 + (ch - si * (mx - mn)[1]) / 2 + 4)
        self.dirs["iso"] = VIEWS["iso"]
        self.labels["iso"] = LABELS["iso"]
        self._draw("iso", (vis, []), off, si, mn)

    def add_view(self, name, shape, dirs, box, scale=None, hidden=True, label=None,
                 margins=(DIM_STEP * 2 + 6, DIM_STEP * 2 + 10, DIM_STEP * 2 + 6, DIM_STEP * 2 + 6)):
        """A view of `shape` looking along -n, centred in the paper box (x0, y0, x1, y1).

        margins (left, bottom, right, top) are kept free round the view for dimensions.
        """
        dirs = VIEWS[dirs] if isinstance(dirs, str) else dirs
        vis, hid = hlr(as_shape(shape), dirs, hidden)
        mn, mx = extents(vis)
        size = mx - mn
        x0, y0, x1, y1 = box
        ml, mb, mr, mt = margins
        w_free, h_free = x1 - x0 - ml - mr, y1 - y0 - mb - mt
        s = scale or fit_scale(size[0], size[1], w_free, h_free)
        origin = (x0 + ml + (w_free - s * size[0]) / 2, y0 + mb + (h_free - s * size[1]) / 2)
        self.dirs[name] = dirs
        self.labels[name] = label or LABELS.get(name, name.upper())
        self._draw(name, (vis, hid), origin, s, mn)
        return s

    def section(self, name, shape, origin, sight, xdir, box, scale=None, letter="A", parent=None,
                ends=None, label=None, margins=None, depth=1000.0):
        """Section view: the material beyond the plane through `origin`, seen along `sight`.

        The cut faces are hatched. With `parent` and `ends` (two model points on the plane),
        the cutting-plane line is drawn on the parent view.
        """
        d = unit(sight)
        o = cq.Vector(*map(float, origin))
        dv = cq.Vector(*map(float, d))
        plane = cq.Plane(origin=o, xDir=cq.Vector(*map(float, xdir)), normal=dv)
        half = cq.Workplane(plane).rect(2000, 2000).extrude(depth).val()   # small depth = cut face only
        kept = as_shape(shape).intersect(half)
        dirs = (tuple(-d), tuple(xdir))
        kw = {} if margins is None else dict(margins=margins)
        s = self.add_view(name, kept, dirs, box, scale, hidden=False,
                          label=label or f"SECTION {letter}-{letter}", **kw)
        for f in kept.Faces():
            c = f.Center()
            try:
                nrm = f.normalAt(c)
            except Exception:
                continue
            if abs(abs(nrm.dot(dv)) - 1) < 1e-6 and abs((c - o).dot(dv)) < 1e-4:
                verts, codes = [], []
                for w in [f.outerWire()] + f.innerWires():
                    ring = np.array([self.P(name, p) for p in wire_pts(w)])
                    verts.extend(ring)
                    verts.append(ring[0])
                    codes.extend([MPath.MOVETO] + [MPath.LINETO] * (len(ring) - 1) + [MPath.CLOSEPOLY])
                self.ax.add_patch(PathPatch(MPath(verts, codes), fc="none", ec="0.3", lw=0, hatch="///"))
        if parent is not None:
            self.cut_line(parent, ends, d, letter)
        return s

    def cut_line(self, view, ends, sight, letter):
        a, b = self.P(view, ends[0]), self.P(view, ends[1])
        u = unit(b - a)
        a, b = a - 2 * u, b + 2 * u
        self._line(a, b, lw=0.18, color="k", ls=CHAIN)
        for p, sgn in ((a, -1), (b, 1)):
            self._line(p, p + 6 * u * -sgn, lw=0.7)
        s2 = self.P(view, np.array(ends[0], float) + sight) - self.P(view, ends[0])
        s2 = unit(s2)
        for p in (a, b):
            tip = p + 8 * s2
            self._line(p, tip, lw=0.3)
            self._arrow(tip, s2)
            self.ax.text(*(tip + 4.5 * s2), letter, fontsize=DIM_PT, ha="center", va="center")

    def _draw(self, view, data, origin, s, mn):
        vis, hid = data
        tr = lambda pts: np.column_stack([origin[0] + s * (pts[:, 0] - mn[0]),
                                          origin[1] + s * (pts[:, 1] - mn[1])])
        if hid:
            self.ax.add_collection(LineCollection([tr(p) for p in hid], colors="0.55",
                                                  linewidths=0.25 * PT, linestyles=(0, (3, 2))))
        self.ax.add_collection(LineCollection([tr(p) for p in vis], colors="k", linewidths=0.35 * PT))
        allp = tr(np.vstack(vis))
        bb = (allp[:, 0].min(), allp[:, 1].min(), allp[:, 0].max(), allp[:, 1].max())
        self.place[view] = (origin, s, mn, bb)
        self.low[view] = bb[1]

    def _labels(self):
        for view, (_, s, _, bb) in self.place.items():
            lab = self.labels.get(view, view.upper())
            if view != "iso" and SCALE_NAME.get(s) != self.scale_label and s in SCALE_NAME:
                lab += f"   SCALE {SCALE_NAME[s]}"
            self.ax.text((bb[0] + bb[2]) / 2, self.low[view] - 5.0, lab,
                         ha="center", va="top", fontsize=LABEL_PT)

    def P(self, view, p):
        origin, s, mn, _ = self.place[view]
        u = proj(self.dirs[view], p)
        return np.array([origin[0] + s * (u[0] - mn[0]), origin[1] + s * (u[1] - mn[1])])

    def U(self, view, p):
        """Model point -> the view's own 2D model coordinates (for reading values off a view)."""
        return proj(self.dirs[view], p)

    # ---- annotation
    def _line(self, a, b, lw=0.18, color="k", ls="-"):
        self.ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=lw * PT, ls=ls,
                     solid_capstyle="butt")

    def _arrow(self, tip, direction):
        d = unit(direction)
        n = np.array([-d[1], d[0]])
        base = np.array(tip) - ARROW * d
        self.ax.add_patch(Polygon([tip, base + 1.0 * n, base - 1.0 * n], closed=True,
                                  color="k", lw=0))

    def dim(self, view, p1, p2, axis, side, level=1, text=None, prefix="", suffix="", pos=None):
        """Linear dimension between two model points, drawn outside the view's bbox.

        pos, when given, is the paper coordinate of the dimension line (y for "h", x for "v")
        and replaces side/level placement.
        """
        a, b = self.P(view, p1), self.P(view, p2)
        u1, u2 = self.U(view, p1), self.U(view, p2)
        _, _, _, bb = self.place[view]
        k = 0 if axis == "h" else 1
        value = abs(u2[k] - u1[k])
        label = text if text is not None else f"{prefix}{fmt(value)}{suffix}"
        step = DIM_STEP
        span = abs(b[k] - a[k])
        tw = text_w(label, DIM_PT)
        flip = span < 2 * ARROW + 1.0             # too short for inside arrows: put them outside
        outside = tw + 3.0 > span                 # label doesn't fit between the arrows: put it past the end
        lo, hi = sorted((a[k], b[k]))
        if axis == "h":
            if pos is not None:
                y = pos
                side = "above" if y > max(a[1], b[1]) else "below"
            else:
                y = bb[3] + step * level if side == "above" else bb[1] - step * level
            for p in (a, b):
                sg = 1 if y > p[1] else -1
                self._line((p[0], p[1] + sg * 1.0), (p[0], y + sg * 1.2))
            ext = 5.0 if flip else 0.0
            self._line((lo - ext, y), (hi + (max(ext, tw + 5.0) if outside else ext), y))
            if span > 0.5:
                for tip, other in ((lo, hi), (hi, lo)):
                    self._arrow((tip, y), ((other - tip) if flip else (tip - other), 0))
            if outside:
                self.ax.text(hi + (6.0 if flip else 2.5), y + 1.2, label, ha="left", va="bottom",
                             fontsize=DIM_PT)
            else:
                self.ax.text((lo + hi) / 2, y + 1.2, label, ha="center", va="bottom", fontsize=DIM_PT)
            if y < bb[1]:
                self.low[view] = min(self.low[view], y - 1.5)
        else:
            if pos is not None:
                x = pos
            else:
                x = bb[2] + step * level if side == "right" else bb[0] - step * level
            for p in (a, b):
                sg = 1 if x > p[0] else -1
                self._line((p[0] + sg * 1.0, p[1]), (x + sg * 1.2, p[1]))
            ext = 5.0 if flip else 0.0
            self._line((x, lo - (max(ext, tw + 5.0) if outside else ext)), (x, hi + ext))
            if span > 0.5:
                for tip, other in ((lo, hi), (hi, lo)):
                    self._arrow((x, tip), (0, (other - tip) if flip else (tip - other)))
            if outside:                            # below the lower end
                self.ax.text(x - 1.2, lo - (6.0 if flip else 2.5), label, ha="right", va="top",
                             fontsize=DIM_PT, rotation=90)
            else:
                self.ax.text(x - 1.2, (lo + hi) / 2, label, ha="right", va="center", fontsize=DIM_PT,
                             rotation=90)

    def ordinate(self, view, axis, pts, side, gap=4.0, extra=None):
        """Ordinate dimensions from the part origin.

        axis "h" measures the view's horizontal coordinate (labels in a row above/below the
        view), "v" the vertical one (labels in a column left/right). `extra` adds labelled
        values that have no leader to a point (e.g. the 0 datum) as (point, text).
        """
        _, _, _, bb = self.place[view]
        k = 0 if axis == "h" else 1
        n_, x_ = (unit(a) for a in self.dirs[view])
        ax_vec = x_ if k == 0 else np.cross(n_, x_)
        sgn_m = float(np.sign(ax_vec[int(np.argmax(np.abs(ax_vec)))]))   # screen axis = +/- a model axis
        items = {}
        for p in pts:
            val = round(sgn_m * float(self.U(view, p)[k]), 2)
            q = self.P(view, p)
            if val not in items or abs(q[1 - k] - (bb[1] if side in ("below", "left") else bb[3])) < \
                    abs(items[val][1 - k] - (bb[1] if side in ("below", "left") else bb[3])):
                items[val] = q
        vals = sorted(items, key=lambda v: items[v][k])
        pos = [items[v][k] for v in vals]
        g = DIM_PT / PT + 1.8
        lab = list(pos)
        for i in range(1, len(lab)):                      # spread labels to a minimum pitch
            lab[i] = max(lab[i], lab[i - 1] + g)
        shift = (sum(pos) - sum(lab)) / len(lab) if lab else 0
        lab = [l + shift for l in lab]
        for i in range(1, len(lab)):
            lab[i] = max(lab[i], lab[i - 1] + g)
        if axis == "h":
            sgn = -1 if side == "below" else 1
            edge = bb[1] if side == "below" else bb[3]
            r0, r1, r2 = edge + sgn * gap, edge + sgn * (gap + 5), edge + sgn * (gap + 8)
            for v, q, t in zip(vals, pos, lab):
                pt = items[v]
                self._line((q, pt[1] + sgn * 1.0), (q, r0))
                self._line((q, r0), (t, r1))
                self._line((t, r1), (t, r2))
                self.ax.text(t, r2 + sgn * 1.0, fmt(v), rotation=90, ha="center",
                             va="top" if sgn < 0 else "bottom", fontsize=DIM_PT)
            if side == "below":
                depth = max(text_w(fmt(v), DIM_PT) for v in vals) if vals else 0
                self.low[view] = min(self.low[view], r2 - 1.5 - depth)
        else:
            sgn = -1 if side == "left" else 1
            edge = bb[0] if side == "left" else bb[2]
            r0, r1, r2 = edge + sgn * gap, edge + sgn * (gap + 5), edge + sgn * (gap + 8)
            for v, q, t in zip(vals, pos, lab):
                pt = items[v]
                self._line((pt[0] + sgn * 1.0, q), (r0, q))
                self._line((r0, q), (r1, t))
                self._line((r1, t), (r2, t))
                self.ax.text(r2 + sgn * 1.0, t, fmt(v), ha="right" if sgn < 0 else "left", va="center",
                             fontsize=DIM_PT)

    def angle(self, view, center, p_a, p_b, r, text=None, text_r=None):
        """Angular dimension between the rays centre->p_a and centre->p_b, arc radius r (paper mm)."""
        c = self.P(view, center)
        a, b = self.P(view, p_a) - c, self.P(view, p_b) - c
        ta_, tb_ = math.atan2(a[1], a[0]), math.atan2(b[1], b[0])
        if tb_ < ta_:
            tb_ += 2 * math.pi
        if tb_ - ta_ > math.pi:
            ta_, tb_ = tb_, ta_ + 2 * math.pi
        ts = np.linspace(ta_, tb_, 60)
        arc = np.column_stack([c[0] + r * np.cos(ts), c[1] + r * np.sin(ts)])
        self.ax.plot(arc[:, 0], arc[:, 1], color="k", lw=0.18 * PT)
        for t, sgn in ((ta_, -1), (tb_, 1)):
            tip = c + r * np.array([math.cos(t), math.sin(t)])
            self._arrow(tip, sgn * np.array([-math.sin(t), math.cos(t)]))
        for v in (a, b):                                   # extension along the rays
            d = np.linalg.norm(v)
            if d < r - 1:
                self._line(c + unit(v) * (d + 1), c + unit(v) * (r + 1.5))
        tm = (ta_ + tb_) / 2
        deg = math.degrees(tb_ - ta_)
        label = text or f"{fmt(deg)}°"
        rr = (text_r or r + 5.0)
        self.ax.text(c[0] + rr * math.cos(tm), c[1] + rr * math.sin(tm), label, fontsize=DIM_PT,
                     ha="center", va="center")

    def note(self, view, p, text, dx, dy, size=NOTE_PT):
        a = self.P(view, p)
        b = a + np.array([dx, dy])
        self._line(a, b)
        self._arrow(a, a - b)
        end = b + np.array([3.0 if dx >= 0 else -3.0, 0])
        self._line(b, end)
        t = self.ax.text(end[0] + (0.8 if dx >= 0 else -0.8), end[1], text, fontsize=size,
                         ha="left" if dx >= 0 else "right", va="center", linespacing=1.15)
        self.note_texts.append((view, t))
        # leader notes never move the view label: place them clear of it

    def callout(self, view, p, text, dx, dy):
        """Dimension-size leader: hole callouts, diameters, radii, chamfers."""
        self.note(view, p, text, dx, dy, size=DIM_PT)

    def on_circle(self, view, center, r, ang_deg):
        """Model point on a circle of radius r about `center`, at a screen angle in the view."""
        c = np.array(center, float)
        n, x = (unit(a) for a in self.dirs[view])
        y = np.cross(n, x)
        t = math.radians(ang_deg)
        return c + r * (math.cos(t) * x + math.sin(t) * y)

    def tag(self, view, p, label, dx=2.0, dy=2.0):
        """Small point label (P1, P2 ...) for a coordinate table."""
        a = self.P(view, p)
        self.ax.add_patch(plt.Circle(a, 0.7, color="k"))
        self.ax.text(a[0] + dx, a[1] + dy, label, fontsize=NOTE_PT * 0.85,
                     ha="left" if dx >= 0 else "right", va="bottom" if dy >= 0 else "top")

    def center(self, view, p, r=4.0):
        a = self.P(view, p)
        for d in ((1, 0), (0, 1)):
            d = np.array(d) * (r * self.place[view][1] + 2)
            self._line(a - d, a + d, lw=0.15, color="0.3", ls=CHAIN)

    def axis_line(self, view, p1, p2):
        self._line(self.P(view, p1), self.P(view, p2), lw=0.15, color="0.3", ls=CHAIN)

    def circle_line(self, view, center, r):
        """Thin chain circle (pitch circle)."""
        c = self.P(view, center)
        rr = r * self.place[view][1]
        ts = np.linspace(0, 2 * math.pi, 120)
        self.ax.plot(c[0] + rr * np.cos(ts), c[1] + rr * np.sin(ts), color="0.3", lw=0.15 * PT, ls=CHAIN)

    def free_text(self, x, y, text, size=NOTE_PT, **kw):
        t = self.ax.text(x, y, text, fontsize=size, linespacing=1.15, **kw)
        return t

    # ---- right column
    def features(self, heading, items, y_top=None):
        """Queue a numbered list for the right column. Items starting '-' or '*' are unnumbered."""
        self.n_features += sum(not it.startswith(("-", "*")) for it in items)
        self.column.append(("list", heading, items))

    def notes(self, items, y=None):
        self.column.append(("notes", "NOTES", items))

    def table(self, heading, header, rows, widths):
        """Tabulated values (e.g. point coordinates) in the right column."""
        self.column.append(("table", heading, (header, rows, widths)))

    @staticmethod
    def _wrap(text, width, size):
        words, lines, cur = text.split(), [], ""
        for w in words:
            trial = (cur + " " + w).strip()
            if cur and text_w(trial, size) > width:
                lines.append(cur)
                cur = w
            else:
                cur = trial
        if cur:
            lines.append(cur)
        return lines or [""]

    def _column_lines(self, pt):
        """Lay out the column at body size pt: [(x offset, text, size, y_step)], total height mm."""
        mm = pt / PT
        out, h = [], 0.0
        for k, (kind, heading, items) in enumerate(self.column):
            if k:
                h += 1.4 * mm
            hpt = 1.25 * pt
            out.append((0.0, heading, hpt, h))
            h += 1.5 * hpt / PT
            if kind == "table":
                header, rows, widths = items
                for r_i, row in enumerate([header] + rows):
                    x = 0.0
                    for cell, w in zip(row, widths):
                        out.append((x, cell, pt, h))
                        x += w
                    h += 1.3 * mm
                    if r_i == 0:
                        out.append((None, "rule", pt, h - 0.25 * mm))
                        h += 0.2 * mm
                continue
            count = 0
            indent = text_w("00. ", pt)
            for item in items:
                if kind == "list":
                    numbered = not item.startswith(("-", "*"))
                    count += numbered
                    body = item.lstrip("-* ")
                    lines = self._wrap(body, COL_W - indent - 1, pt)
                    if numbered:
                        out.append((0.0, f"{count}.", pt, h))
                    for line in lines:
                        out.append((indent, line, pt, h))
                        h += 1.28 * mm
                    h += 0.35 * mm
                else:
                    npt = 0.92 * pt
                    bi = text_w("- ", npt)
                    for j, line in enumerate(self._wrap(item, COL_W - bi - 1, npt)):
                        if j == 0:
                            out.append((0.0, "-", npt, h))
                        out.append((bi, line, npt, h))
                        h += 1.28 * npt / PT
        return out, h

    def _draw_column(self, y_top=283.0):
        pt = COL_PT_MAX
        while pt > COL_PT_MIN:
            if y_top - self._column_lines(pt)[1] >= COL_FLOOR:
                break
            pt = round(pt - 0.2, 1)
        lines, h = self._column_lines(pt)
        if y_top - h < COL_FLOOR:
            print(f"  warning: {self.part_no} p{self.page} column runs "
                  f"{COL_FLOOR - (y_top - h):.1f} mm into the title block")
        for dx, text, size, dy in lines:
            if dx is None:
                self._line((COL_X, y_top - dy), (COL_X + COL_W, y_top - dy), lw=0.18)
            else:
                self.ax.text(COL_X + dx, y_top - dy, text, fontsize=size, va="top")
        self.column_pt = pt

    def title_block(self):
        self._labels()
        self._draw_column()
        ax = self.ax
        x0, y0, w, h = COL_X, 14.0, COL_W, TB_H
        tag_h = 9.0
        ax.add_patch(Rectangle((x0, y0), w, h, fill=False, lw=0.5 * PT))
        # work-set row across the top, so a loose sheet says whose it is
        ws = work_set(self.part_no)
        tag = f"WORK SET {ws}:  {WORK_SETS[ws][1]}" if ws else "REFERENCE SHEET:  in work sets A and B"
        ax.add_patch(Rectangle((x0, y0 + h - tag_h), w, tag_h, fill=ws is not None, fc="0.88",
                               lw=0.25 * PT))
        ax.text(x0 + 2.0, y0 + h - tag_h / 2, tag, fontsize=10.0, va="center")
        title = self.title if self.pages == 1 else f"{self.title}  (part sheet {self.page} of {self.pages})"
        rows = [("PROJECT", "TALOS 5-axis arm"), ("PART", title), ("PART NO.", self.part_no),
                ("QTY / ARM", self.qty), ("MATERIAL", self.material),
                ("EST. MASS", f"{self.mass:.0f} g" if self.mass is not None else "-"),
                ("SCALE", self.scale_label), ("UNITS / REV", f"mm / {REV}"), ("DATE", TODAY),
                ("SHEET", f"{self.number} of {self.total}")]
        rh = (h - tag_h) / 5
        for i, (k, v) in enumerate(rows):
            col, row = divmod(i, 5)
            cx = x0 + col * w / 2
            cy = y0 + h - tag_h - (row + 1) * rh
            ax.add_patch(Rectangle((cx, cy), w / 2, rh, fill=False, lw=0.25 * PT))
            ax.text(cx + 1.5, cy + rh - 1.0, k, fontsize=6.4, va="top", color="0.35")
            size = 10.0
            while size > 6.0 and text_w(v, size) > w / 2 - 3:
                size -= 0.4
            ax.text(cx + 1.5, cy + 1.4, v, fontsize=size, va="bottom")
        # third-angle symbol
        sx, sy = x0 + w - 16, y0 + h + 5.0
        ax.add_patch(Polygon([(sx, sy - 2.2), (sx + 5, sy - 3.2), (sx + 5, sy + 3.2), (sx, sy + 2.2)],
                             fill=False, lw=0.3 * PT))
        for r in (1.6, 3.2):
            ax.add_patch(plt.Circle((sx + 11, sy), r, fill=False, lw=0.3 * PT))
        ax.text(sx - 2, sy, "THIRD ANGLE PROJECTION", fontsize=7.5, ha="right", va="center")

# ---------------------------------------------------------------- sheets

def P3(x, z, y=0.0):
    """Point in a part's XZ profile plane."""
    return (x, y, z)


def point_table(s, view, pts, heading="PROFILE POINTS (X, Z)", tag_offsets=None, used=None):
    """Tag points on a view and list their coordinates in the right column."""
    rows = []
    for i, (name, (x, z)) in enumerate(pts):
        dx, dy = (tag_offsets or {}).get(name, (2.0, 2.0))
        s.tag(view, P3(x, z), name, dx, dy)
        rows.append([name, fmt(x), fmt(z)] + ([used[name]] if used else []))
    header = ["PT", "X", "Z"] + (["PROFILE"] if used else [])
    s.table(heading, header, rows, [14, 20, 20, 70] if used else [14, 22, 22])


def sheet_assembly():
    s = Sheet("General assembly and parts list", "TAL-000", slug="general-assembly", qty="-", material="-",
              frame="world: base bottom centre, Z up, arm reaches +X")
    zero = cq.Compound.makeCompound([sh for _, sh, _ in ta.flatten(ta.build(ta.POSES["zero"]))])
    ready = cq.Compound.makeCompound([sh for _, sh, _ in ta.flatten(ta.build(ta.POSES["ready"]))])
    s.views(zero, hidden=False, iso_shape=ready, gap_right=5 * DIM_STEP + 10 - GAP_FR)
    h = [ta.H_PAN, ta.H_PAN + ta.H_SH, ta.H_PAN + ta.H_SH + ta.L1, ta.H_PAN + ta.H_SH + ta.L1 + ta.L2,
         ta.H_PAN + ta.H_SH + ta.L1 + ta.L2 + ta.X_ROLL]
    for k, z in enumerate(h, start=1):
        s.dim("front", (66, 0, 0), (24, 0, z), "v", "right", level=k)
        if k > 1:
            s.center("front", (0, 0, z), 3 / s.place["front"][1])
    s.dim("front", (-100, 0, 0), (66, 0, 0), "h", "below")
    s.dim("top", (0, -66, 0), (0, 66, 0), "v", "left")
    printed = [("TAL-101", "Base", 1), ("TAL-102", "Shoulder (turntable + cradle)", 1),
               ("TAL-103", "Upper arm", 1), ("TAL-104", "Forearm", 1), ("TAL-105", "Wrist", 1),
               ("TAL-106", "Gripper body", 1), ("TAL-107", "Moving jaw", 1),
               ("TAL-108", "Hub cap", 3), ("TAL-110", "Idler plate", 2),
               ("TAL-111", "Servo fit gauge (print first)", 1)]
    bought = [
        ["Feetech STS3215 servo (12 V: C018 / C047)", "6"], ["Waveshare Bus Servo Adapter (A)", "1"],
        ["M3 x 6 button head (output and passive horns)", "36"],
        ["M3 x 10 button head (idler plates)", "4"],
        ["M2 x 5 self-tapping (servo cases)", "10"], ["M2 x 6 self-tapping (servo cases)", "12"],
        ["M2.5 x 6 self-tapping (board)", "4"], ["M4 bolts to the table (user's)", "4"],
        ["3-pin bus servo cables (with servos)", "6"],
    ]
    rows = [[no, name, str(q), work_set(no)] for no, name, q in printed]
    s.table("PRINTED PARTS", ["PART NO.", "NAME", "QTY", "SET"], rows, [24, 76, 14, 14])
    s.table("BOUGHT PARTS", ["ITEM", "QTY"], bought, [112, 16])
    layout = [
        f"Pan axis vertical through the base centre; pan horn face {fmt(ta.H_PAN)} above the table. "
        f"Plinth 132 across corners.",
        f"Shoulder axis (along Y) {fmt(ta.H_SH)} above the pan horn face. Upper arm {fmt(ta.L1)}, forearm "
        f"{fmt(ta.L2)} (axis to axis). Wrist flex axis to roll horn face {fmt(ta.X_ROLL)}.",
        "Front view: zero pose (all joints 0). Isometric: 'ready' pose.",
        "Assembly, each joint: take both horns off the servo (one centre screw each; the base and wrist "
        "roll servos may keep theirs); plug a 3-pin cable into the servo; push the servo into its part along "
        "its entry path (part sheet notes); fit its M2 case screws; refit both horns through the R10.8 "
        "openings, each with its centre screw; fit the next link with 4X M3 × 6 into the output horn and "
        "4X M3 × 6 into the passive horn.",
        "Print TAL-111 first and check the servo fit before printing the arm. Each part sheet gives "
        "the print orientation; print-ready 3MF files are in cad/print.",
    ]
    s.features("JOINT LAYOUT", layout)
    s.notes(GENERAL[:1] + ["Part sheets give each part in its own frame; this sheet shows where those frames sit."])
    return s


def print_note(part_no):
    return f"Print: {ta.PRINT_SETUP[part_no][1]}."


def sheet_cap(large=True):
    cap = as_shape(ta.hub_cap(large))
    r, recess = (12.5, 10.4) if large else (10.2, 3.4)
    name, no, slug, side = (("Hub cap", "TAL-108", "hub-cap", "output side, covers 4 horn screws")
                            if large else
                            ("Hub cap, small", "TAL-109", "hub-cap-small", "idler side, covers 1 centre screw"))
    s = Sheet(name, no, slug=slug, qty="3", frame="origin centre of the inner face, +Y outward")
    sc = s.add_view("front", cap, "front", (14, 14, 150, 150), label="FRONT (INNER FACE)",
                    margins=(8, 16, 8, 8))
    s.scale_label = SCALE_NAME[sc]
    s.center("front", (0, 0, 0), r)
    s.section("A", cap, (0, 0, 0), (0, 0, -1), (1, 0, 0), (14, 150, 272, 283), letter="A",
              parent="front", ends=((-r, 0, 0), (r, 0, 0)), margins=(30, 36, 30, 34))
    s.dim("A", (-r, 0, 0), (r, 0, 0), "h", "below", 2, prefix=DIA)
    s.dim("A", (-recess, 0, 0), (recess, 0, 0), "h", "below", 1, prefix=DIA)
    s.dim("A", (-recess, 0, 0), (-recess, 1.8, 0), "v", "left", 1)
    s.dim("A", (r - 0.8, 0, 0), (r - 0.8, 3.4, 0), "v", "right", 2)
    s.callout("A", (r - 0.4, 3.0, 0), "0.8 × 45°", 26, 24)
    s.iso(cap, (160, 14, 268, 150), max_scale=4)
    feats = [
        f"* {name.upper()} ({side}):",
        f"Disc {DIA}{fmt(2 * r)}, 3.4 thick (y 0 to 3.4); chamfer 0.8 × 45° on the outer edge of the outer face.",
        f"Recess {DIA}{fmt(2 * recess)}, 1.8 deep from the inner face (clears M3 button heads, 1.65 high).",
    ]
    s.features("FEATURES (in order)", feats)
    s.notes(GENERAL + ["Glue or tape onto the output-side clevis plate over the four horn screws (the recess is 0.55 clear "
                       "of the screw heads all round, so it does not grip them).",
                       print_note(no)])
    s.mass = printed_mass(cap)
    return s


def sheet_idler():
    idl = as_shape(ta.idler_plate(tab=True))
    s = Sheet("Idler plate", "TAL-110", slug="idler-plate", qty="2",
              frame="link frame of TAL-103 / TAL-104: origin on the joint axis, Y = axis")
    sc = s.add_view("front", idl, "front", (14, 110, 200, 283), label="FRONT (OUTER FACE, -Y)",
                    margins=(34, 34, 30, 14))
    s.scale_label = SCALE_NAME[sc]
    s.add_view("top", idl, "top", (14, 14, 200, 110), scale=sc, margins=(44, 20, 10, 10))
    s.iso(idl, (200, 20, 272, 140), max_scale=2)
    s.center("front", (0, 0, 0), 10.5)
    s.callout("front", s.on_circle("front", (0, 0, 0), 10.5, 215), "R10.5", -12, -20)
    horn_callouts(s, "front", (0, 0, 0), (1, 0, 0), (0, 0, 1), dx=(-26, 46), dy=(30, 52))
    s.callout("front", (26 - 1.2, 0, 7 + 1.2), f"2X {DIA}{fmt(TAB_HOLE_D)} THRU", -14, 22)
    for name, (x, z), off in (("N1", (0, 10.5), (-2, 2)), ("N2", (6, 8.5), (1, 2)), ("N3", (18, 8.5), (0, 2)),
                              ("N4", (30, 15), (1.5, 1.5))):
        s.tag("front", (x, 0, z), name, *off)
    s.table("NECK POINTS (X, Z), MIRRORED IN Z", ["PT", "X", "Z"],
            [["N1", "0", "±10.5"], ["N2", "6", "±8.5"], ["N3", "18", "±8.5"], ["N4", "30", "±15"]],
            [14, 34, 40])
    s.ordinate("front", "h", [(0, 0, -10.5), (6, 0, -8.5), (18, 0, -8.5), (26, 0, -7), (30, 0, -15)], "below")
    s.ordinate("front", "v", [(30, 0, -15), (30, 0, -7), (30, 0, 0), (30, 0, 7), (30, 0, 15)], "right")
    s.ordinate("top", "v", [(-10.5, -22.9, 0), (-10.5, -18.9, 0), (-9.5, -ta.PASSIVE_FACE, 0)], "left")
    s.callout("top", (5, -18.75, 0), f"{DIA}19 PAD, {fmt(ta.CLV_IN - ta.PASSIVE_FACE)} HIGH\n(ON THE PASSIVE HORN)",
              16, 14)
    s.axis_line("top", (-14, 0, 0), (34, 0, 0))
    s.features("FEATURES (in order)", [
        "Plate, y -22.9 to -18.9: circle R10.5 at the origin + neck polygon N1 to N4 (mirrored in Z). "
        "Same outline as the idler plate on TAL-002.",
        f"Pad {DIA}19 on the inner face, y -18.9 to {fmt(-ta.PASSIVE_FACE)}: it bears on the servo's passive horn.",
        f"Centre hole {DIA}7 through: clears the servo's idler boss (nothing screws into the boss).",
        f"Horn holes: 4X {DIA}{fmt(M3_HOLE_D)} through on {DIA}{fmt(PCD_D)} PCD at 45°, no counterbore (4X M3 × "
        f"{ta.PASSIVE_SCREW} into the passive horn, heads on the outer face).",
        f"Tab holes: 2X {DIA}{fmt(TAB_HOLE_D)} through at x 26, z ±7 (M3 × 10 into the arm link).",
    ])
    s.notes(GENERAL + [
        "One on each arm link (TAL-103, TAL-104), in the pocket on the link's -Y side: 2X M3 × 10 into the "
        f"link, 4X M3 × {ta.PASSIVE_SCREW} into the servo's passive horn, which turns on the idler boss (the "
        f"tip stops {fmt(ta.PASSIVE_FACE - ta.CLV_OUT + ta.PASSIVE_SCREW)} into the horn, short of the servo "
        "case). No hub cap on this side: one wide enough for the screws would hit the parent servo's plugs.",
        print_note("TAL-110"),
    ])
    s.mass = printed_mass(idl)
    return s


def sheet_gauge():
    g, origins = ta.fit_gauge()
    g = as_shape(g)
    no = "TAL-111"
    s = Sheet("Servo fit gauge", no, slug="servo-fit-gauge", qty="1",
              frame="origin at a bottom corner, Z up as printed; X along the servo length, Y across its thickness")
    sc = s.add_view("top", g, "top", (14, 80, 190, 283), label="TOP (AS PRINTED)", margins=(40, 40, 40, 14))
    s.scale_label = SCALE_NAME[sc]
    s.add_view("front", g, "front", (14, 14, 190, 80), scale=sc, margins=(40, 14, 40, 10))
    s.iso(g, (196, 20, 272, 150), max_scale=1.5)
    bb = g.BoundingBox()
    s.ordinate("top", "h", [(0, 0, 8), (bb.xmax, 0, 8)], "below")
    s.ordinate("top", "v", [(0, 0, 8), (0, origins[0][1], 8), (0, origins[1][1], 8), (0, bb.ymax, 8)], "left")
    s.ordinate("top", "h", [(origins[0][0], bb.ymax, 8)], "above")
    for k, (ox, oy) in enumerate(origins):
        frame_mark(s, "top", (ox, oy, 8), f"O{k + 1}", 4, 3)
    s.dim("front", (bb.xmax, 0, 0), (bb.xmax, 0, 8), "v", "right", 1)
    s.callout("top", (0.1, origins[1][1] - 9, 8), "NOTCHES: 1 = W1, 2 = W2\n1.5 WIDE, 1.5 DEEP", 90, -6)
    (c1, p1), (c2, p2) = ta.GAUGE_CLEAR
    rows = []
    for name, (x0, x1, y0, y1) in zip(("Case", "Rib notch", "Spline notch", "Bump notch", "Boss notch"),
                                      ta.gauge_window((0.0, 0.0))):
        rows.append([name, f"{fmt(x0)} to {fmt(x1)}", f"{fmt(y0)} to {fmt(y1)}"])
    s.table("GAUGE WINDOW AT ZERO CLEARANCE (X, Y)",
            ["RECTANGLE", "X", "Y"], rows, [40, 46, 42])
    s.features("FEATURES (in order)", [
        f"Two frames, 8 thick (z 0 to 8), 3 walls, sharing the middle wall; overall {fmt(bb.xlen)} × "
        f"{fmt(bb.ylen)}.",
        f"Window W1 about O1 ({fmt(origins[0][0])}, {fmt(origins[0][1])}): the case rectangle grown by "
        f"{fmt(c1)} and the four notches by {fmt(p1)}, the clearances every servo pocket uses. Cut through Z.",
        f"Window W2 about O2 ({fmt(origins[1][0])}, {fmt(origins[1][1])}): the same, grown by {fmt(c2)} and "
        f"{fmt(p2)} (tight).",
        "Notches on the x 0 wall's top edge: one beside W1, two beside W2.",
    ])
    s.notes(GENERAL[:1] + [
        "Use: take both horns off and push the bare servo (no cables) through each window along its width, "
        "top face (rib, spline) toward the rib notch, bottom face (bump, idler boss) toward the boss notch.",
        "It should go through W1 by hand with light friction and not through W2. If W1 binds, or W2 lets "
        "it through, tune the printer (flow, horizontal expansion) before printing the arm.",
        print_note(no),
    ])
    s.mass = printed_mass(g)
    return s


M3_HOLE_D = 2 * ta.SCREW[3]["clear"]            # M3 clearance hole, 3.4 for FDM
TAB_HOLE_D = 2 * (ta.SCREW[3]["clear"] + 0.1)   # idler plate tab holes
PCD_D = 2 * ta.HORN_PCD_R                       # horn screw circle, 14


def horn_callouts(s, view, centre, u, v, dx=(16, 22), dy=(-20, 16), thru="THRU", hole_d=M3_HOLE_D, centre_d=7.0,
                  extra=""):
    """Callouts for the horn pattern: centre ⌀7 + 4x ⌀3.4 on ⌀14 at 45° (by default).

    u, v are the model directions of the view's screen x and y through `centre`.
    """
    c = np.array(centre, float)
    u, v = np.array(u, float), np.array(v, float)
    s.circle_line(view, c, ta.HORN_PCD_R)
    k = ta.HORN_PCD_R / math.sqrt(2)
    hole = c + k * u + k * v
    s.callout(view, hole + hole_d / 2 * (u + v) / math.sqrt(2),
              f"4X {DIA}{fmt(hole_d)} {thru}{extra}\nON {DIA}{fmt(PCD_D)} PCD, 45°", dx[1], dy[1])
    s.callout(view, c - centre_d / 2 * v, f"{DIA}{fmt(centre_d)} {thru}", dx[0], dy[0])


def sheet_jaw():
    jaw = as_shape(ta.moving_jaw())
    s = Sheet("Moving jaw", "TAL-107", slug="moving-jaw",
              frame="origin on the jaw pivot, X along the closed jaw, Y = pivot axis")
    s.views(jaw, right=False, iso=False, gap_ft=40)
    s.iso(jaw, (120, 224, 272, 283), max_scale=1)
    s.mass = printed_mass(jaw)
    s.center("front", (0, 0, 0), 13)
    s.callout("front", s.on_circle("front", (0, 0, 0), 13, 140), "R13", -12, 16)
    horn_callouts(s, "front", (0, 0, 0), (1, 0, 0), (0, 0, 1), dx=(-14, 6), dy=(-30, 30))
    s.callout("front", (31.5, 0, -3.9), "6X V-NOTCH 3 WIDE × 1.6 DEEP\nPITCH 6, FIRST AT X 30", 22, -26)
    pts = [("A", (0, -13)), ("B", (34, -5.5)), ("C", (34, 9.1)), ("D", (0, 13)),
           ("F1", (22, -5.5)), ("F2", (64, -5.5)), ("F3", (64, -1.5)), ("F4", (52, 5)),
           ("F5", (30, 10)), ("F6", (22, 9))]
    used = {"A": "plate", "B": "plate, knuckle", "C": "plate, knuckle", "D": "plate",
            "F1": "finger, knuckle", "F2": "finger", "F3": "finger", "F4": "finger",
            "F5": "finger, knuckle", "F6": "finger, knuckle"}
    offs = {"A": (2, -2), "B": (2, -2), "F1": (-2, -2), "F2": (2, -2), "F6": (-2, 2), "D": (2, 2)}
    point_table(s, "front", pts, tag_offsets=offs, used=used)
    s.ordinate("top", "v", [(64, -10, 0), (64, 0, 0), (64, 10, 0), (34, 18.9, 0), (34, 22.9, 0)], "right")
    s.axis_line("top", (-16, 0, 0), (68, 0, 0))
    feats = [
        "Plate, y 18.9 to 22.9: circle R13 at the origin + polygon A B C D.",
        "Finger, y -10 to 10: polygon F1 F2 F3 F4 F5 F6.",
        "Knuckle, y 10 to 18.9: polygon F1 B C F5 F6.",
        "Grip notches: 6X V-notch on the finger's lower edge, y -10 to 18.9 (finger and knuckle; the "
        "plate is not notched), 3 wide × 1.6 deep, pitch 6, first at x 30.",
        f"Pad {DIA}19 on the plate's inner face, y 18.7 to 18.9: bears on the output horn face.",
        f"Horn holes through the plate: {DIA}7 centre + 4X {DIA}{fmt(M3_HOLE_D)} on {DIA}{fmt(PCD_D)} PCD at 45°.",
    ]
    s.features("FEATURES (in order)", feats)
    s.notes(GENERAL + ["Bolts to the jaw servo's output horn with 4x M3 x 6; closed position is "
                       "jaw X along the gripper X.", print_note("TAL-107")])
    return s


def frame_mark(s, view, origin, label, dx=4.0, dy=4.0):
    """Datum cross for a servo frame origin (pocket P placement)."""
    a = s.P(view, origin)
    for d in ((1, 0), (0, 1)):
        d = np.array(d) * 4.0
        s._line(a - d, a + d, lw=0.35)
    s.ax.add_patch(plt.Circle(a, 1.6, fill=False, lw=0.3 * PT))
    s.ax.text(a[0] + dx, a[1] + dy, label, fontsize=NOTE_PT, ha="left" if dx >= 0 else "right",
              va="bottom" if dy >= 0 else "top")


def pocket_note(origin, shaft, body_x, body_y):
    return (f"Pocket P (sheet TAL-001) with its servo frame origin P0 at ({', '.join(fmt(v) for v in origin)}): "
            f"servo +Z (shaft) along {shaft}, servo +X along {body_x}, servo +Y along {body_y}.")


def sheet_servo():
    body, horns, plugs = ta.sts3215()
    servo = as_shape(body.union(horns).union(plugs))
    s1 = Sheet("STS3215 servo (reference)", "TAL-001", slug="servo-pocket-reference", qty="6",
               material="bought part", frame="servo: origin on the shaft axis at mid-plane, +Z = shaft",
               page=1, pages=2)
    s1.views(servo, iso=False)
    s1.center("front", (0, 0, 0), 3)
    s1.axis_line("front", (0, 0, -30), (0, 0, 22))
    x0, x1 = ta.CASE_X
    rib, bump = ta.RIB, ta.BUMP
    s1.ordinate("front", "h", [(x0, 0, -ta.BOTTOM_FACE), (bump[0], 0, -bump[3]), (bump[1], 0, -bump[3]),
                               (-11.1, 0, -26), (0, 0, -ta.BOSS[1]), (x1, 0, ta.CORE_Z[0])], "below")
    s1.ordinate("front", "h", [(rib[0], 0, rib[3]), (ta.FACE_X, 0, ta.TOP_FACE)], "above")
    s1.ordinate("front", "v", [(x0, 0, -ta.BOTTOM_FACE), (x0, 0, ta.TOP_FACE), (rib[0], 0, rib[3]),
                               (bump[0], 0, -bump[3]), (-16.9, 0, -26), (0, 0, 0)], "left")
    s1.ordinate("front", "v", [(x1, 0, -ta.PASSIVE_FACE), (x1, 0, ta.CORE_Z[0]), (x1, 0, ta.CORE_Z[1]),
                               (x1, 0, ta.HORN_DISC), (x1, 0, ta.HORN_FACE)], "right")
    s1.ordinate("top", "v", [(x0, -ta.CASE_HW, 0), (HXF, -HY, 0), (x0, 0, 0), (HXF, HY, 0),
                             (x0, ta.CASE_HW, 0)], "left")
    s1.callout("top", (HXF, HY + 0.8, 0), f"2X {DIA}1.6 (M2 SELF-TAP)\nTOP FACE, X {fmt(HXF)}", 8, 26)
    s1.callout("top", s1.on_circle("top", (0, 0, 0), ta.HORN_R, 300),
               f"HORN {DIA}{fmt(2 * ta.HORN_R)} × {fmt(ta.HORN_FACE - ta.HORN_DISC)}", 18, -18)
    horn_callouts(s1, "top", (0, 0, 0), (1, 0, 0), (0, 1, 0), dx=(40, 22), dy=(-6, 26),
                  thru=f"{DEPTH}{fmt(ta.HORN_FACE - ta.HORN_DISC)}", hole_d=2.5, centre_d=3.2, extra=" (M3)")
    s1.ordinate("right", "h", [(0, -10.2, -26), (0, -0.3, -26), (0, 0.3, -26), (0, 10.2, -26)], "below")
    s1.ordinate("right", "h", [(0, -rib[2], rib[3]), (0, rib[2], rib[3])], "above")
    s1.callout("right", (0, 5.2, -26), "MATED BUS PLUGS", 14, -10)
    s1.features("SERVO (servo frame)", [
        f"Case {fmt(x1 - x0)} × {fmt(2 * ta.CASE_HW)}, x {fmt(x0)} to {fmt(x1)}, full outline z "
        f"{fmt(ta.CORE_Z[0])} to {fmt(ta.CORE_Z[1])}; +X end corners C3, -X end corners R1.",
        f"Top face (output side) z {fmt(ta.TOP_FACE)}, from x {fmt(x0)} to {fmt(ta.FACE_X)}, stopping at R11 "
        f"round the shaft. Rib on it: x {fmt(rib[0])} to {fmt(rib[1])}, y ±{fmt(rib[2])}, up to z {fmt(rib[3])}, "
        "its shaft end at R11.",
        f"Bottom face (connector side) z {fmt(-ta.BOTTOM_FACE)}, from x {fmt(x0)} to {fmt(ta.FACE_X)}, stopping "
        f"at R11. Bump on it: x {fmt(bump[0])} to {fmt(bump[1])}, y ±{fmt(bump[2])}, down to z {fmt(-bump[3])}.",
        f"Connector sockets x {fmt(ta.CONN_X[0])} to {fmt(ta.CONN_X[1])}, y 0.5 to 10 and -10 to -0.5; the "
        "plugs go in from the bottom face. Mated plugs modelled to z -26.",
        f"Output horn {DIA}{fmt(2 * ta.HORN_R)}: disc z {fmt(ta.HORN_DISC)} to {fmt(ta.HORN_FACE)}, hub "
        f"{DIA}{fmt(2 * ta.HUB_R)} down to the case (z {fmt(ta.CORE_Z[1])}); 4X {DIA}2.5 M3 on {DIA}{fmt(PCD_D)} "
        f"PCD at 45°, centre {DIA}3.2. Bare spline (horn off) {DIA}{fmt(2 * ta.SPLINE[0])} up to z "
        f"{fmt(ta.SPLINE[1])}.",
        f"Passive horn {DIA}{fmt(2 * ta.HORN_R)}: disc z {fmt(-ta.PASSIVE_DISC)} to {fmt(-ta.PASSIVE_FACE)}, "
        f"turning on the idler boss {DIA}{fmt(2 * ta.BOSS[0])} (tip z {fmt(-ta.BOSS[1])}).",
        f"Case holes {DIA}1.6 (M2 self-tap): top face at ({fmt(HXF)}, ±{fmt(HY)}), bottom face at "
        f"({fmt(HXB)}, ±{fmt(HY)}). Each face has a second pair at x {fmt(HXS)}; the base uses the bottom one.",
    ])
    s1.notes(["Bought part: Feetech STS3215 (the 7.4 V and 12 V variants share this case). Sizes measured "
              "from Feetech's CAD inside SO-101's assembly STEP (cad/reference/so101/step/"
              "STS3215_from_SO101.step); each modelled size is the real one or up to 0.25 bigger. "
              "servo_reference.py checks every pocket against that model.",
              "Both horns come off (one centre screw each): every servo goes into its part with its horns "
              "off. Sheet 2 gives pocket P, the envelope every servo part cuts."])

    pocket = as_shape(ta.servo_pocket())
    s2 = Sheet("Servo pocket P (cut into parts)", "TAL-001", slug="servo-pocket-reference", qty="-",
               material="-", frame="servo frame (as sheet 1); origin P0", page=2, pages=2)
    s2.views(pocket)
    frame_mark(s2, "front", (0, 0, 0), "P0", 4, 3)
    frame_mark(s2, "top", (0, 0, 0), "P0", 4, 3)
    c, q = ta.FIT_C, ta.PASS_C
    px0, px1 = x0 - c, x1 + c
    top, bot = ta.TOP_FACE + c, -(ta.BOTTOM_FACE + c)
    r_top, b_bot, boss_z = rib[3] + q, -(bump[3] + q), -(ta.BOSS[1] + q)
    s2.ordinate("front", "h", [(px0, 0, bot), (bump[0] - q, 0, b_bot), (-17.6, 0, -28.5),
                               (-10.6, 0, -28.5), (0, 0, boss_z), (px1, 0, bot)], "below")
    s2.ordinate("front", "h", [(rib[0] - q, 0, r_top), (10.8, 0, 19.5)], "above")
    s2.ordinate("front", "v", [(px0, 0, bot), (px0, 0, top), (rib[0] - q, 0, r_top), (bump[0] - q, 0, b_bot),
                               (-17.6, 0, -28.5), (0, 0, 0)], "left")
    s2.ordinate("front", "v", [(10.8, 0, boss_z), (10.8, 0, ta.CORE_Z[0]), (10.8, 0, ta.CORE_Z[1]),
                               (10.8, 0, 19.5)], "right")
    s2.ordinate("top", "v", [(px0, -ta.CASE_HW - c, 0), (px0, ta.CASE_HW + c, 0), (-17.6, -10.6, 0),
                             (-17.6, 10.6, 0), (bump[0] - q, -bump[2] - q, 0), (bump[0] - q, bump[2] + q, 0)], "left")
    s2.ordinate("top", "v", [(-ta.SPLINE[0] - q, -rib[2] - q, 0), (-ta.SPLINE[0] - q, rib[2] + q, 0)], "right")
    s2.callout("top", s2.on_circle("top", (0, 0, 0), 10.8, 120), f"2X {DIA}21.6 HORN CLEARANCE\n(BOTH FACES)", -30, 34)
    s2.features(f"POCKET P (servo frame; {fmt(c)} / {fmt(q)} clearance included)", [
        f"Case box x {fmt(px0)} to {fmt(px1)}, y ±{fmt(ta.CASE_HW + c)}, z {fmt(bot)} to {fmt(top)}: the "
        f"case + {fmt(c)} all round (printing tolerance).",
        f"Rib channel x {fmt(rib[0] - q)} to {fmt(-ta.SPLINE[0] - q)} (into the spline channel), y ±{fmt(rib[2] + q)}, z {fmt(ta.TOP_FACE)} to "
        f"{fmt(r_top)}; bump channel x {fmt(bump[0] - q)} to {fmt(bump[1] + q)}, y ±{fmt(bump[2] + q)}, z "
        f"{fmt(b_bot)} to {fmt(-ta.BOTTOM_FACE)} (each + {fmt(q)}).",
        f"Plug bay box x -17.6 to -10.6, y ±10.6, z -28.5 to {fmt(ta.CORE_Z[0])} (mated plugs + cable bend).",
        f"Relief box x -10.6 to -9.0, y ±4.5, z {fmt(boss_z)} to {fmt(ta.CORE_Z[0])}: no knife edge where the "
        f"plug bay meets the idler {DIA}21.6.",
        f"Idler cylinder {DIA}21.6 on the shaft axis, z {fmt(boss_z)} to {fmt(ta.CORE_Z[0])} (passive horn "
        f"and boss); horn cylinder {DIA}21.6, z {fmt(ta.CORE_Z[1])} to 19.5 (output horn; bare spline "
        f"{DIA}{fmt(2 * (ta.SPLINE[0] + q))} to z {fmt(ta.SPLINE[1] + c)}).",
        "* Way in (each part sheet): the case box, rib, bump and plug bay boxes run on out along the "
        f"servo's entry direction, and the spline and boss {DIA}7 become slots of their width. The servo "
        "goes in with both horns off.",
    ])
    s2.notes(["Each part sheet locates P0 and gives the servo axes in the part frame; cut the union of "
              "the bodies.", GENERAL[0]])
    return [s1, s2]


def sheet_common():
    clv = as_shape(ta.clevis())
    s1 = Sheet("Common features: clevis", "TAL-002", slug="common-features", qty="-",
               frame="link: origin on the joint axis, Y = axis", page=1, pages=2)
    sc = s1.add_view("front", clv, "front", (14, 14, 143, 150), scale=2, label="FRONT (IDLER PLATE, -Y)",
                     margins=(18, 30, 18, 12))
    s1.scale_label = SCALE_NAME[sc]
    s1.add_view("rear", clv, "rear", (143, 14, 272, 150), scale=2, label="REAR (OUTPUT PLATE, +Y)",
                margins=(30, 30, 10, 12))
    s1.add_view("top", clv, "top", (14, 150, 143, 283), scale=2, margins=(40, 18, 10, 10))
    s1.iso(clv, (150, 160, 268, 280), max_scale=2)
    s1.center("front", (0, 0, 0), 10.5)
    s1.center("rear", (0, 0, 0), 16)
    s1.callout("front", s1.on_circle("front", (0, 0, 0), 10.5, 215), "R10.5", -10, -18)
    horn_callouts(s1, "front", (0, 0, 0), (1, 0, 0), (0, 0, 1), dx=(26, 12), dy=(-20, 40))
    for name, (x, z), off in (("N1", (0, 10.5), (-2, 2)), ("N2", (6, 8.5), (1, 2)), ("N3", (18, 8.5), (0, 2)),
                              ("N4", (30, 15), (1.5, 1.5))):
        s1.tag("front", (x, 0, z), name, *off)
    s1.table("IDLER NECK POINTS (X, Z), MIRRORED IN Z", ["PT", "X", "Z"],
             [["N1", "0", "±10.5"], ["N2", "6", "±8.5"], ["N3", "0.6R = 18", "±8.5"], ["N4", "R = 30", "±H = ±15"]],
             [14, 34, 40])
    s1.callout("rear", s1.on_circle("rear", (0, 0, 0), 16, 90), "R16", 6, 18)
    horn_callouts(s1, "rear", (0, 0, 0), (-1, 0, 0), (0, 0, 1), dx=(20, -14), dy=(-16, 34))
    s1.dim("rear", (0, 0, -15), (30, 0, -15), "h", "below", 1, text="R = 30")
    s1.dim("rear", (30, 0, -15), (30, 0, 15), "v", "left", 1, text="2H = 30")
    s1.ordinate("top", "v", [(-16, -22.9, 0), (-16, -18.9, 0), (-9.5, -ta.PASSIVE_FACE, 0), (-16, 18.9, 0),
                             (-16, 22.9, 0)], "left")
    s1.axis_line("top", (-20, 0, 0), (34, 0, 0))
    s1.features("CLEVIS (every pitch joint)", [
        "* Drawn for the arm links: reach R = 30, half-height H = 15. The wrist uses R = 24, H = 12.",
        "* Not printed on its own: the plates are modelled into the links. On TAL-103 and TAL-104 the "
        "idler plate is the separate part TAL-110; on the wrist (TAL-105) both plates are wrist body.",
        "Output plate (+Y), y 18.9 to 22.9: circle R16 at the origin + rectangle x 0 to R, z ±H.",
        f"Output plate pad {DIA}19 on the inner face, y 18.7 to 18.9: bears on the output horn face (the "
        "plate itself clears the parent servo's shell by 0.5).",
        f"Output plate holes, through: {DIA}7 centre (horn centre screw) + 4X {DIA}{fmt(M3_HOLE_D)} on "
        f"{DIA}{fmt(PCD_D)} PCD at 45°.",
        "Idler plate (-Y), y -22.9 to -18.9: circle R10.5 + neck polygon N1 to N4 (mirrored). The neck "
        "clears the parent servo's bus plugs.",
        f"Idler plate pad {DIA}19 on the inner face, y -18.9 to {fmt(-ta.PASSIVE_FACE)}, bearing on the passive "
        f"horn; centre {DIA}7 through (clears the idler boss); 4X {DIA}{fmt(M3_HOLE_D)} on {DIA}{fmt(PCD_D)} PCD "
        f"at 45°, no counterbore (4X M3 × {ta.PASSIVE_SCREW} into the passive horn, which turns on the boss; "
        f"heads on the outer face).",
        "Clevis gap in the link body: remove a cylinder R21 about Y between y ±18.9, and everything at "
        "x < 0 between those planes, except the two horn pads (dimensioned on each link's section).",
    ])
    s1.notes(GENERAL)

    s2 = Sheet("Common features: clip, screw hole, window", "TAL-002", slug="common-features", qty="-",
               frame="local frames given per feature", page=2, pages=2)
    clip = as_shape(ta.cable_clip((0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1))))
    sc = s2.add_view("clip_r", clip, "right", (14, 150, 120, 283), scale=5, label="CABLE CLIP: END VIEW")
    s2.scale_label = SCALE_NAME[sc]
    s2.add_view("clip_f", clip, "front", (120, 150, 272, 283), scale=5, label="CABLE CLIP: SIDE VIEW")
    s2.dim("clip_r", (0, -5, -1), (0, 5, -1), "h", "below", 1)
    s2.dim("clip_r", (0, -2.5, 0.8), (0, 2.5, 0.8), "h", "below", 2)
    s2.dim("clip_r", (0, -0.75, 5), (0, 0.75, 5), "h", "above", 1)
    s2.ordinate("clip_r", "v", [(0, -5, -1), (0, -5, 0), (0, -2.5, 0.8), (0, -2.5, 4.3), (0, -0.75, 4.2), (0, -4, 5)],
                "left")
    s2.callout("clip_r", (0, 4.5, 4.5), "2X C1", 16, 14)
    s2.dim("clip_f", (-3, 0, -1), (3, 0, -1), "h", "below", 1)
    s2.axis_line("clip_f", (-5, 0, 2.55), (5, 0, 2.55))
    s2.note("clip_f", (0, 0, 0), "MOUNTING FACE (w = 0)", 22, -10)
    F, S0 = ta.TOP_FACE, 17.5  # drawn for the top face and an example seat
    hole = as_shape(ta.box(-5, 5, F, 18.4, -5, 5).cut(ta.cyl_y(1.2, F - 0.1, S0)).cut(ta.cyl_y(2.1, S0, 18.5)))
    s2.section("H", hole, (0, 0, 0), (0, 0, -1), (1, 0, 0), (14, 14, 140, 150), scale=8,
               label="CASE SCREW HOLE (SECTION)")
    s2.dim("H", (-1.2, S0, 0), (1.2, S0, 0), "h", "below", 1, prefix=DIA, pos=s2.P("H", (0, F, 0))[1] - 8)
    s2.dim("H", (-2.1, 18.4, 0), (2.1, 18.4, 0), "h", "above", 1, prefix=DIA)
    s2.dim("H", (-5, F, 0), (-5, S0, 0), "v", "left", 1, text="S - F")
    s2.free_text(s2.P("H", (-5, F, 0))[0], s2.P("H", (-5, F, 0))[1] - 22,
                 f"SERVO FACE F: TOP FACE {fmt(ta.TOP_FACE)},\nBOTTOM FACE {fmt(ta.BOTTOM_FACE)}, FROM THE "
                 "SERVO MID-PLANE", ha="left", va="top")
    s2.note("H", (2.1, S0, 0), "SEAT S", 18, 8)
    s2.table("SEAT S (FROM THE SERVO MID-PLANE)", ["PART", "FACE", "S"],
             [["arm shells, gripper", "top", fmt(ta.FRONT_SEAT)], ["arm shells, gripper", "bottom",
                                                                   fmt(ta.SHELL_IN + 1.0)],
              ["shoulder", "both", "18.5"], ["base", "bottom", fmt(ta.BASE_SEAT)], ["wrist", "bottom", "18.0"]],
             [52, 18, 18])
    win = as_shape(ta.box(-6, 46, -1, 1, -14, 14).cut(ta.prism_xz(ta.hex_pts(0, 40, 0, 10), -2, 2)))
    s2.add_view("hex", win, "front", (140, 14, 272, 150), scale=2, hidden=False, label="HEX WINDOW H(xa, xb, zc, k)")
    s2.dim("hex", (0, 0, 0), (40, 0, 0), "h", "below", 1, text="xa to xb", pos=s2.P("hex", (0, 0, -14))[1] - 10)
    s2.dim("hex", (30, 0, 0), (40, 0, 0), "h", "above", 1, text="k")
    s2.dim("hex", (40, 0, 0), (30, 0, 10), "v", "right", 1, text="k")
    s2.axis_line("hex", (-4, 0, 0), (44, 0, 0))
    s2.note("hex", (0, 0, 0), "(xa, zc)", -6, 14)
    s2.features("COMMON FEATURES", [
        "* CABLE CLIP, one bus cable. Local u along the cable, v across, w out of the mounting face.",
        "Block u ±3, v ±5, w -1 to 5 (sunk 1 into the face).",
        "Tunnel through along u: v ±2.5, w 0.8 to 4.3. Slit v ±0.75 from w 4.2 out through the top.",
        "Chamfer C1 on the two outer long edges.",
        f"* CASE SCREW HOLE (M2 self-tap into the servo case): {DIA}2.4 from the servo face F to the seat S, "
        f"then {DIA}4.2 counterbore from S outward through the part. Screw length: S - F + 3.5, rounded down.",
        "* HEX WINDOW H(xa, xb, zc, k): hexagon (xa, zc) (xa+k, zc-k) (xb-k, zc-k) (xb, zc) (xb-k, zc+k) "
        "(xa+k, zc+k), cut through the part along Y. Drawn for H(0, 40, 0, 10).",
    ])
    s2.notes(GENERAL[:1])
    return [s1, s2]


def sheet_base():
    parts, _ = ta.base_parts()
    base = as_shape(parts["base"])
    top = ta.H_PAN - 2.8
    frame = "origin plinth bottom centre, Z up, +X = arm front"
    s1 = Sheet("Base", "TAL-101", slug="base", frame=frame, page=1, pages=2)
    sc = s1.add_view("top", base, "top", (14, 14, 200, 283), scale=2 / 3, margins=(42, 60, 20, 60))
    s1.scale_label = SCALE_NAME[sc]
    s1.mass = printed_mass(base)
    s1.iso(base, (205, 205, 272, 283), max_scale=0.4)
    s1.center("top", (0, 0, 0), 66)
    deck = [("D1", (-40, -32)), ("D2", (-88, -32)), ("D3", (-100, -20)), ("D4", (-100, 20)),
            ("D5", (-88, 32)), ("D6", (-40, 32))]
    offs = {"D1": (2, -2), "D2": (-2, -2), "D3": (-2, -2), "D4": (-2, 2), "D5": (-2, 2), "D6": (2, 2)}
    rows = []
    for name, (x, y) in deck:
        dx, dy = offs[name]
        s1.tag("top", (x, y, 10), name, dx, dy)
        rows.append([name, fmt(x), fmt(y)])
    s1.table("BOARD DECK POINTS (X, Y)", ["PT", "X", "Y"], rows, [14, 22, 22])
    seat_z = ta.H_PAN - ta.HORN_FACE - ta.BASE_SEAT  # case screw head seats, above the table
    s1.ordinate("top", "h", [(-85.5, 18.5, 15), (-57.5, 18.5, 15), (-36, 12.8, top), (HXB, HY, 27),
                             (0, 0, 0), (11, 12.8, top), (66, 0, 10)], "above")
    s1.ordinate("top", "v", [(-85.5, -18.5, 15), (-85.5, 18.5, 15), (-36, -12.8, top), (-36, 12.8, top),
                             (-36, 0, 0), (HXB, -HY, 27), (HXB, HY, 27)], "left")
    s1.circle_line("top", (0, 0, 0), 54)
    a45 = (54 * math.cos(math.radians(45)), 54 * math.sin(math.radians(45)), 10)
    s1.callout("top", (a45[0] + 3.1, a45[1] + 3.1, 10),
               f"4X {DIA}4.4 THRU\n{CBORE}{DIA}8.4 {DEPTH}3.5\nON R54 AT 45°", 34, 16)
    s1.angle("top", (0, 0, 0), (70, 0, 0), (a45[0] * 1.3, a45[1] * 1.3, 0), 30)
    s1.callout("top", (-57.5 + 2.3, 18.5 + 2.3, 15), f"4X {DIA}6.4 BOSS, TOP Z 15\nPILOT {DIA}2.1 {DEPTH}7 (M2.5)",
               -10, 62)
    s1.callout("top", (HXB + 0.85, -HY - 0.85, 27),
               f"4X {DIA}2.4 THRU FLOOR\nAT X {fmt(HXB)}, {fmt(HXS)}\n{CBORE}{DIA}4.2 FROM BOTTOM {DEPTH}{fmt(seat_z)}",
               -20, -64)
    f10 = (41.4 * math.cos(math.radians(10)), 41.4 * math.sin(math.radians(10)), 0)
    s1.callout("top", (f10[0] + 2.0, f10[1] - 1.6, 0), "18X FLUTE R2.6 ON R41.4\n20° PITCH, FIRST AT 10°", 20, -58)
    s1.angle("top", (0, 0, 0), (60, 0, 0), (f10[0] * 1.5, f10[1] * 1.5, 0), 34, text_r=40)
    s1.callout("top", (0, -63.75, 10), f"12-SIDED, {DIA}132 ACROSS\nCORNERS, CORNER ON +X", -20, -10)
    s1.cut_line("top", ((-104, 0, 0), (70, 0, 0)), np.array([0, 1, 0]), "A")
    s1.features("FEATURES (in order)", [
        f"Plinth: regular 12-gon, {DIA}132 across corners (a corner on +X), z 0 to 10.",
        "Board deck: polygon D1 to D6, z 0 to 10; union with the plinth.",
        "Chamfer C2.5 on all top outer edges (z 10).",
        f"Mounting holes: 4X {DIA}4.4 through on R54 at 45° + 90°n; counterbore {DIA}8.4 from z 6.5 up.",
        f"Collar {DIA}90, z 10 to 14, C2 on its top edge.",
        f"Column {DIA}80, z 10 to {fmt(top)}, C1.5 on its top edge.",
        f"Board standoffs: 4X {DIA}6.4, z 9 to 15, at x -57.5 / -85.5, y ±18.5; pilot {DIA}2.1 from z 8 to 16.",
        f"Flutes: 18X vertical {DIA}5.2 on R41.4 at 10° + 20°n, axis z 19 to 52 (z 28 to 52 for the two at "
        f"170° and 190°, over the cable tunnel), spherical ends; cut.",
        pocket_note((0, 0, ta.H_PAN - 18.7), "+Z", "+X", "+Y"),
        f"Top opening: cut box x -36 to 11, y ±12.8, from z {fmt(top - 1)} up (the way in: the servo drops "
        "in from the top, connector end first).",
        "Cable tunnel (plug bay to board deck): cut box x -60 to -13, y ±6, z 14.8 to 24.5. Pocket P's bump "
        "channel (z down to 24.3) opens into its roof.",
        f"Case screw holes from below (TAL-002, seat {fmt(ta.BASE_SEAT)}): 4X at x {fmt(HXB)} and x {fmt(HXS)}, "
        f"y ±{fmt(HY)}: {DIA}2.4 from z {fmt(seat_z)} up to the pocket floor (z "
        f"{fmt(ta.H_PAN - ta.HORN_FACE - ta.BOTTOM_FACE - ta.FIT_C)}), {DIA}4.2 counterbore from the bottom to z "
        f"{fmt(seat_z)}. The pair at x {fmt(HXS)} runs 0.2 from the plug bay: cut the wall between them away "
        f"(box x -10.6 to {fmt(HXS)}, |y| {fmt(HY - 2.1)} to 10.6, z 14.8 to {fmt(seat_z)}).",
    ])
    s1.notes(GENERAL + ["The pan servo drops in from the top (its horns may stay on); its plugs hang into the "
                        "bay under it. 4X M2 × 6 case screws from below, two at each end of the servo, so the "
                        "arm's overturning moment can't rock it in the pocket.",
                        print_note("TAL-101")])

    s2 = Sheet("Base", "TAL-101", slug="base", frame=frame, page=2, pages=2)
    sc = s2.add_view("front", base, "front", (14, 150, 272, 283), scale=1, margins=(40, 30, 30, 26))
    s2.scale_label = SCALE_NAME[sc]
    s2.ordinate("front", "v", [(-100, 0, 0), (-100, 0, 10), (-45, 0, 14), (-85.5 - 3.2, 0, 15),
                               (41.4, 0, 19), (41.4, 0, 52), (-40, 0, top)], "left")
    s2.dim("front", (-40, 0, top), (40, 0, top), "h", "above", 1, prefix=DIA)
    s2.dim("front", (-45, 0, 12), (45, 0, 12), "h", "above", 2, prefix=DIA)
    s2.callout("front", (40 - 0.75, 0, top - 0.75), "C1.5", 14, 6)
    s2.callout("front", (45 - 1, 0, 13), "C2", 22, 4)
    s2.callout("front", (66 - 1.25, 0, 8.75), "C2.5", 10, 12)
    s2.callout("front", (41.4 * math.cos(math.radians(-10)) - 2.6, 0, 35), "R2.6 FULL ROUND\nENDS", -58, -10)
    s2.section("A", base, (0, 0, 0), (0, 1, 0), (1, 0, 0), (14, 14, 272, 150), scale=1,
               margins=(40, 40, 30, 14))
    frame_mark(s2, "A", (0, 0, ta.H_PAN - 18.7), "P0", 4, 3)
    s2.ordinate("A", "h", [(-60, 0, 14.8), (-36, 0, top), (-13, 0, 14.8), (0, 0, 0), (11, 0, top)], "below")
    s2.ordinate("A", "v", [(-60, 0, 14.8), (-60, 0, 24.5), (-36, 0, ta.H_PAN - 18.7), (-36, 0, top - 1)], "left")
    s2.note("A", (-36, 0, 20), "CABLE TUNNEL, Y ±6", -24, 22)
    s2.note("A", (-5, 0, ta.H_PAN - 18.7 - 10), "POCKET P (TAL-001)", 40, -8)
    s2.features("SECTION A-A", [
        "* Section on the XZ plane (cutting line on sheet 1), looking toward +Y.",
        "* P0 = pocket P servo frame origin; servo axes along the part axes.",
    ])
    s2.notes(GENERAL[:1])
    return [s1, s2]


def sheet_shoulder():
    parts, _ = ta.shoulder_parts()
    sh = as_shape(parts["shoulder"])
    frame = "origin on pan axis at the turntable bottom (= pan horn face), Z up"
    s1 = Sheet("Shoulder (turntable + cradle)", "TAL-102", slug="shoulder", frame=frame, page=1, pages=2)
    s1.views(sh, iso=False, gap_ft=40)
    s1.mass = printed_mass(sh)
    point_table(s1, "front", [("T1", (-24, 6)), ("T2", (24, 6)), ("T3", (14, 21)), ("T4", (-14, 21))],
                heading="CHEEK PROFILE (X, Z)", tag_offsets={"T1": (-2, 2), "T4": (-2, 2)})
    s1.ordinate("front", "h", [(-37, 0, 0), (-17, 0, 0), (-12.9, 0, 50), (-HY, 0, ZB), (0, 0, 0),
                               (HY, 0, ZB), (12.9, 0, 50), (17, 0, 0), (37, 0, 0)], "below")
    s1.ordinate("front", "v", [(-37, 0, 0), (-37, 0, 6), (-17, 0, ZB), (-17, 0, 14.5), (-17, 0, 34),
                               (-17, 0, ta.H_SH), (-17, 0, ta.H_SH + 4)], "left")
    s1.callout("front", (2.8, -25, 16.8), f"{DIA}8 THRU (ALONG Y)", -4, 42)
    s1.callout("front", (HY + 1.5, -25, ZB + 1.5), f"2X {DIA}4.2 THRU CHEEK\n{DIA}2.4 TO POCKET (SEAT 18.5)",
               40, 46)
    s1.callout("front", (35.6, 0, 5.4), "C2", 10, 16)
    s1.callout("front", (36.6, 0, 0.4), "C0.8", 10, -10)
    s1.ordinate("top", "v", [(-24, -25, 21), (-17, -18.4, 21), (-12.8, -ta.SHELL_IN, 30),
                             (-12.8, ta.TOP_FACE + ta.FIT_C, 30), (-17, 18.4, 21), (-24, 25, 21)], "left")
    s1.ordinate("top", "h", [(-12.8, ta.TOP_FACE + ta.FIT_C, 30), (12.8, ta.TOP_FACE + ta.FIT_C, 30)], "above")
    s1.callout("top", (33, 0, 6), "4X ARC SLOT R27 TO R33\n36° WIDE, AT 90° PITCH, THRU", 44, 8)
    s1.angle("top", (0, 0, 0), (33 * math.cos(math.radians(-18)), 33 * math.sin(math.radians(-18)), 0),
             (33 * math.cos(math.radians(18)), 33 * math.sin(math.radians(18)), 0), 40)
    horn_callouts(s1, "top", (0, 0, 10), (1, 0, 0), (0, 1, 0), dx=(26, 30), dy=(-36, 48),
                  extra=f"\n{CBORE}{DIA}6.5 {DEPTH}6 FROM INSIDE")
    s1.cut_line("top", ((HY, -38, 0), (HY, 38, 0)), np.array([1, 0, 0]), "A")
    s1.features("FEATURES (in order)", [
        f"Turntable {DIA}74, z 0 to 6; C2 top edge, C0.8 bottom edge.",
        "Cradle: box x ±17, y ±18.4, z 5 to 34.",
        f"Posts: boxes x -17 to -12.9 and 12.9 to 17, y ±18.4, z 30 to {fmt(ta.H_SH + 4)}.",
        "Cheeks: profile T1 to T4, y 18.4 to 25 and -25 to -18.4 (fused to the cradle walls).",
        f"Doubler on the -Y cradle wall: box x ±10.5, y {fmt(-ta.SHELL_OUT - ta.DOUBLER)} to -18.4, z "
        f"{fmt(ta.H_SH - 31.0)} to 34 (leaves 0.8 over the bump channel).",
        "Cable clip (TAL-002) on the -X face: centre (-17, -9, 21), cable along Z.",
        pocket_note((0, 0, ta.H_SH), "+Y", "+Z (body hangs down)", "+X"),
        f"Servo opening: cut box x ±12.8, y {fmt(-ta.SHELL_IN)} to {fmt(ta.TOP_FACE + ta.FIT_C)}, from z "
        f"{fmt(ta.H_SH + ta.CASE_X[0] - ta.FIT_C)} up. Way in from the top (pocket P run on up, TAL-001; horns "
        "off): +Y wall: rib channel x ±7.5, y 15.5 to 17.1, from z 11.2 up; spline slot x ±3.5, y to 17.6, "
        "from z 46 up. -Y wall: bump channel x ±9.7, y -19 to -16.5, from z 15.8 up; plug slot through the "
        "wall x ±10.6, from z 28.4 up; boss slot x ±3.5, from z 46 up.",
        f"Cable hole {DIA}8 along Y, through all, at x 0, z 14.",
        f"Pan horn: {DIA}7 centre through. 4X {DIA}{fmt(M3_HOLE_D)} through on {DIA}{fmt(PCD_D)} PCD at 45°, counterbore "
        f"{DIA}6.5 from the floor (z 10) down to z 4 (M3 × 6 from inside the cradle); centre opened to "
        f"{DIA}8.6 over the same depth (no thin web between the counterbores and the centre hole).",
        "Turntable windows: 4X arc slot R27 to R33, ±18° about 0°, 90°, 180°, 270°, through.",
        f"Case screw holes (TAL-002, seat 18.5) through the cheeks: +Y cheek at x ±{fmt(HY)}, z "
        f"{fmt(ta.H_SH + HXF)}; -Y cheek at x ±{fmt(HY)}, z {fmt(ZB)}.",
    ])
    s1.notes(GENERAL + ["Fit the shoulder servo from the top, far end first with the shaft to +Y, with both "
                        "horns off; then the case screws through the cheeks (2X M2 × 6 on +Y, 2X M2 × 5 on -Y).",
                        print_note("TAL-102")])

    s2 = Sheet("Shoulder (turntable + cradle)", "TAL-102", slug="shoulder", frame=frame, page=2, pages=2)
    sc = s2.section("A", sh, (HY, 0, 0), (1, 0, 0), (0, -1, 0), (14, 140, 180, 283), scale=1.5,
                    label="SECTION A-A", margins=(36, 40, 10, 10))
    s2.scale_label = SCALE_NAME[sc]
    frame_mark(s2, "A", (0, 0, ta.H_SH), "P0", 4, 3)
    s2.ordinate("A", "v", [(0, 25, 0), (0, 25, 6), (0, 15.9, 10.5), (0, -25, ZB), (0, 25, 17), (0, 25, 21),
                           (0, 18.4, ta.H_SH)], "left")
    s2.ordinate("A", "h", [(0, 25, 6), (0, 18.5, 17), (0, 15.5, 17), (0, -16.5, ZB),
                           (0, -18.5, ZB), (0, -25, 6)], "below")
    s2.callout("A", (0, 18.5 + 1, 17 + 2.1), f"{CBORE}{DIA}4.2 TO SEAT", 26, 26)
    s2.callout("A", (0, -16.5 - 0.7, ZB - 1.2), f"{DIA}2.4", -24, -20)
    s2.note("A", (0, 0, 30), "POCKET P (TAL-001)", 34, 40)
    s2.section("B", sh, (HK, 0, 0), (-1, 0, 0), (0, 1, 0), (14, 14, 180, 140), scale=1.5,
               label="SECTION B-B", margins=(36, 30, 10, 10))
    s2.add_view("left", sh, "left", (180, 150, 272, 283), scale=1, label="LEFT", margins=(30, 30, 6, 6))
    s2.ordinate("left", "h", [(-23, -9, 18), (-23, 0, 0)], "below")
    s2.ordinate("left", "v", [(-23, -9, 21), (-23, 0, 0)], "right")
    s2.callout("left", (-23, -9, 26), "CABLE CLIP (TAL-002)\nCABLE ALONG Z", -4, 22)
    s2.iso(sh, (180, 20, 272, 140), max_scale=1)
    s2.ordinate("B", "v", [(HK, -37, 0), (HK, -6.5, 4), (HK, -ta.SHELL_IN, 10.5)], "left")
    s2.callout("B", (HK, HK + 3.25, 7.5), f"{CBORE}{DIA}6.5", 26, 20)
    s2.callout("B", (HK, -HK - M3_HOLE_D / 2, 2), f"{DIA}{fmt(M3_HOLE_D)}", -16, -24)
    s1.cut_line("front", ((HK, 0, -1), (HK, 0, 51)), np.array([-1, 0, 0]), "B")
    s2.features("SECTIONS", [
        f"* A-A: plane x {fmt(HY)} through one hole of each case screw row (cutting line on sheet 1, "
        f"top view), looking toward +X. The other holes mirror at x {fmt(-HY)}.",
        f"* B-B: plane x {fmt(HK)} through two pan horn screw holes (cutting line on sheet 1, front view).",
    ])
    s2.notes(GENERAL[:1])
    return [s1, s2]


def loft_profile(w, h, c):
    """Beam station profile (octagon + spine ridge) as a thin solid in the YZ plane, for a profile view."""
    octo = cq.Workplane("YZ").polyline(ta.octagon_flat(w, h, c)).close().extrude(0.2)
    ridge = cq.Workplane("YZ").polyline(ta.ridge_profile(h)).close().extrude(0.2)
    return as_shape(octo.union(ridge))


def profile_dims(s, name, w, h, c):
    a, b, zb = w / 2, h / 2, ta.Z_BED
    s.dim(name, (0, -a, zb), (0, a, zb), "h", "below", 1)
    s.dim(name, (0, -a, zb), (0, -a, b), "v", "left", 1)
    s.dim(name, (0, a - c, b), (0, a, b), "h", "above", 1)
    s.dim(name, (0, a, b - c), (0, a, b), "v", "right", 1)
    s.dim(name, (0, -4, b), (0, 4, b), "h", "above", 2)
    s.dim(name, (0, -4, b), (0, -2.4, b + 1.8), "v", "left", 2)
    s.callout(name, (0, -3.2, b + 1.0), "2X C1.6", -14, 6)


def sheet_arm(upper=True):
    L = ta.L1 if upper else ta.L2
    p = dict(h0=30.0, h1=30.8, w_mid=34.0, h_mid=25.0) if upper else dict(ta.FOREARM)
    body, _, _, _ = ta.arm_link(L, **p)
    body = as_shape(body)
    name, no, axis = ("Upper arm", "TAL-103", "shoulder") if upper else ("Forearm", "TAL-104", "elbow")
    distal = "elbow" if upper else "wrist flex"
    slug = name.lower().replace(" ", "-")
    frame = f"origin on the {axis} axis, X to the {distal} axis, Y = axis"
    x0, x1 = 20.0, L - 36.0
    xm, wins = arm_windows(L, **p)
    s1 = Sheet(name, no, slug=slug, frame=frame, page=1, pages=2)
    s1.views(body, right=False, iso=False, scale=1, gap_ft=46)
    s1.mass = printed_mass(body)
    s1.iso(body, (170, 190, 272, 283), max_scale=0.6)
    s1.center("front", (0, 0, 0), 16)
    s1.center("front", (L, 0, 0), 18)
    zb = ta.Z_BED
    s1.ordinate("front", "h", [(0, 0, zb), (x0, 0, zb), (26, 0, -7), (xm, 0, zb), (x1, 0, zb),
                               (L - 38.2, 0, zb), (L - 34.2, 0, 15.4), (L + HXB, 0, -HY), (L, 0, zb)],
                "below")
    s1.ordinate("front", "v", [(L + 18, 0, zb), (L + 18, 0, -HY), (L + 18, 0, 0),
                               (L + 18, 0, HY), (L + 18, 0, 11.4), (L + 18, 0, 15.4)], "right")
    s1.ordinate("front", "v", [(-16, 0, -7), (-16, 0, 7)], "left")
    s1.callout("front", s1.on_circle("front", (L, 0, 0), 18, -40), "R18", 4, -26)
    s1.callout("front", s1.on_circle("front", (0, 0, 0), 16, 125),
               "CLEVIS OUTPUT PLATE (TAL-002)\nR = 30, H = 15", 6, 30)
    s1.callout("front", (26 - 0.9, 0, -7 - 0.9),
               f"2X {DIA}2.5 {DEPTH}8 (M3 × 10)\nIN IDLER PLATE POCKET:\nTAL-110 OUTLINE + 0.3\nY -23.9 TO -18.9",
               -12, -38)
    s1.callout("front", (L + HXB - 1.5, 0, HY + 1.5),
               f"4X {CBORE}{DIA}4.2, {DIA}2.4 TO POCKET\n-Y FACE X {fmt(L + HXB)}, SEAT "
               f"{fmt(ta.SHELL_IN + 1.0)}\n+Y FACE X {fmt(L - 29)}, SEAT {fmt(ta.FRONT_SEAT)}", 12, 24)
    tags, rows = {}, []
    for k, win in enumerate(wins):
        for j, idx in enumerate((0, 1, 2, 3)):
            x, z = win[idx]
            tag = f"W{4 * k + j + 1}"
            s1.tag("front", (x, 0, z if idx in (0, 3) else -abs(z)), tag,
                   {0: 1.5, 1: -1.0, 2: 1.0, 3: -1.5}[idx], 1.5 if idx in (0, 3) else -1.5)
            rows.append([tag, fmt(x), "0" if idx in (0, 3) else f"±{fmt(abs(z))}"])
    s1.table("WINDOW POINTS (X, Z), THROUGH Y", ["PT", "X", "Z"], rows, [14, 24, 24])
    s1.ordinate("top", "v", [(0, -ta.CLV_OUT, 0), (0, ta.CLV_OUT, 0)], "left")
    s1.ordinate("top", "v", [(L, -ta.SHELL_OUT, 0), (x1 - 4, -4, 0), (x1 - 4, 4, 0), (L, ta.SHELL_OUT, 0)], "right")
    s1.ordinate("top", "h", [(x0 + 8, 4, 0), (x1 - 4, 4, 0)], "above")
    s1.callout("top", (xm + 3, -p["w_mid"] / 2 - 5.2, 0),
               f"CABLE CLIP (TAL-002) AT X {fmt(xm)}\n-Y FLANK, CABLE ALONG X", 20, -14)
    for letter, x in (("A", x0), ("B", xm), ("C", x1)):
        s1.cut_line("front", ((x, 0, -18), (x, 0, 18)), np.array([-1, 0, 0]), letter)
    s1.cut_line("top", ((-18, 0, 0), (L + 20, 0, 0)), np.array([0, 1, 0]), "D")
    s1.features("FEATURES (in order)", [
        "Clevis output plate only (TAL-002, +Y plate with its horn pad), R = 30, H = 15. The idler plate "
        "is the separate part TAL-110.",
        f"Beam: ruled loft of the profiles A, B, C (sheet 2) at x {fmt(x0)}, {fmt(xm)}, {fmt(x1)}. "
        f"Every profile sits on z {fmt(zb)}, so the underside is flat; the waist dips on top only.",
        f"Servo shell: XZ polygon ({fmt(L - 38.2)}, {fmt(zb)}) ({fmt(L)}, {fmt(zb)}) ({fmt(L)}, 15.4) "
        f"({fmt(L - 34.2)}, 15.4) ({fmt(L - 38.2)}, 11.4), extruded y ±18.4; plus a cylinder R18 about Y "
        f"at x {fmt(L)}.",
        f"Flat underside: cut everything below z {fmt(zb)} (trims the clevis hub and the R18 cylinder).",
        f"Nose floor: box x {fmt(L + 9)} to {fmt(L + 13.5)}, y ±18.4, z {fmt(zb)} to {fmt(zb + 4)}; ties the shell "
        f"walls together on the bed (the servo entry cut later trims its back to x {fmt(L + 10.5)}).",
        f"Doubler on the -Y wall: box x {fmt(L - 31.0)} to {fmt(L - 16.4)}, y {fmt(-ta.SHELL_OUT - ta.DOUBLER)} "
        f"to -18.4, z {fmt(zb)} to 10.5 (leaves 0.8 over the bump channel; out of the next link's reach).",
        f"Spine: ruled loft of the ridge profiles A, B, C, trimmed to x {fmt(x0 + 8)} to {fmt(x1 - 4)}.",
        "Windows: polygons W1 to W4 and W5 to W8 (mirrored in Z), cut through Y.",
        f"Cable clip (TAL-002) on the -Y flank at x {fmt(xm)}, cable along X.",
        "Clevis gap cut (TAL-002): R21 about Y between y ±18.9, plus x < 0 between those planes; the "
        "output pad stays.",
        pocket_note((L, 0, 0), "+Y", "+X", "-Z"),
        f"Servo entry from below (pocket P run on down, TAL-001; horns off): case box x {fmt(L - 35.5)} to "
        f"{fmt(L + 10.5)}, y -16.9 to 15.9, z below 12.8 (opens the floor under the whole servo). +Y wall: rib "
        f"channel x {fmt(L - 34.8)} to {fmt(L - 3.5)}, y 15.5 to 17.1, z below 7.5; spline slot x {fmt(L)} ±3.5, "
        f"y to 17.6, z below 0. -Y wall: bump channel x {fmt(L - 30.2)} to {fmt(L - 17.2)}, y -19 to -16.5, z "
        f"below 9.7; plug window x {fmt(L - 17.6)} to {fmt(L - 10.6)}, z below 10.6; notch through the wall x "
        f"{fmt(L - 17.6)} to {fmt(L + 3.5)}, z below 0 (joins the plug window and the idler hole).",
        f"Case screw holes (TAL-002): +Y face (seat {fmt(ta.FRONT_SEAT)}, over the rib channel; heads stand 0.7 "
        f"proud) at x {fmt(L + HXF)}, z ±{fmt(HY)}; -Y face (seat {fmt(ta.SHELL_IN + 1.0)}) at x {fmt(L + HXB)}, "
        f"z ±{fmt(HY)}.",
        "Idler plate pocket: the TAL-110 outline (circle R10.5 + neck N1 to N4) grown 0.3, cut from "
        "y -23.9 to -18.9.",
        f"Idler plate screw holes: 2X {DIA}2.5 at x 26, z ±7, from y -18.9 8 deep (M3 × 10).",
    ])
    s1.notes(GENERAL + [f"Fit the {distal} servo from below, shaft to +Y, with both horns off; then the case "
                        "screws (2X M2 × 6 on +Y, 2X M2 × 5 on -Y) and refit the horns.",
                        f"The {distal} servo's bus plugs come out through the -Y shell wall.",
                        f"Print: {ta.PRINT_SETUP[no][1]}. The flat underside is z {fmt(zb)}."])

    s2 = Sheet(name, no, slug=slug, frame=frame, page=2, pages=2)
    st = ((x0, 45.8, p["h0"], 6.0), (xm, p["w_mid"], p["h_mid"], 5.0), (x1, 36.8, p["h1"], 6.0))
    sc = None
    for k, ((x, w, h, c), letter) in enumerate(zip(st, "ABC")):
        box = (14 + 86 * k, 188, 100 + 86 * k, 280)
        sc = s2.add_view(f"prof{letter}", loft_profile(w, h, c), "right", box, scale=1, hidden=False,
                         label=f"PROFILE {letter}-{letter} (X {fmt(x)})", margins=(26, 22, 14, 30))
        profile_dims(s2, f"prof{letter}", w, h, c)
    s2.scale_label = SCALE_NAME[1]
    s2.section("D", body, (0, 0, 0), (0, 1, 0), (1, 0, 0), (14, 98, 272, 186), scale=1,
               label="SECTION D-D", margins=(36, 30, 36, 8))
    frame_mark(s2, "D", (L, 0, 0), "P0", 4, 3)
    s2.callout("D", s2.on_circle("D", (0, 0, 0), 21, 35), "R21 GAP (TAL-002)", 12, 20)
    s2.ordinate("D", "h", [(L - 35.5, 0, ta.Z_BED), (L + 10.5, 0, ta.Z_BED)], "below")
    s2.add_view("bottom", body, "bottom", (14, 14, 272, 100), scale=1, hidden=False, margins=(36, 28, 36, 6))
    s2.ordinate("bottom", "v", [(L - 35.5, -ta.SHELL_IN, 0), (L - 35.5, ta.TOP_FACE + ta.FIT_C, 0)], "left")
    s2.ordinate("bottom", "h", [(L - 35.5, 15.9, 0), (L + 10.5, 15.9, 0)], "below")
    s2.ordinate("bottom", "h", [(L - 30.2, -19, 0), (L - 17.6, -18.4, 0), (L + 3.5, -18.4, 0)], "above")
    s2.note("bottom", (L - 20, 8, -15), "SERVO ENTRY (FROM BELOW)", 30, 14)
    s2.features("PROFILES AND SECTION", [
        f"* Profiles A, B, C: beam section (top corners chamfered, underside flat and square-cornered) with "
        f"the spine ridge on top, in the YZ "
        f"plane at the loft stations; cutting lines on sheet 1. Each sits on z {fmt(ta.Z_BED)} (flat "
        f"underside) with its top at the height given.",
        f"* Ridge: 8 wide, 4.8 across the top, 1.8 above the beam, 45° bevels; its base runs 3 into the beam.",
        "* Section D-D: XZ plane, looking toward +Y. P0 = pocket P servo frame origin.",
    ])
    s2.notes(GENERAL[:1])
    return [s1, s2]


def sheet_wrist():
    parts, _ = ta.wrist_parts()
    wr = as_shape(parts["wrist"])
    xs = ta.X_ROLL - 18.7
    xe = ta.X_ROLL - 2.8
    frame = "origin on the wrist flex axis, X along the roll axis, Y = flex axis"
    s1 = Sheet("Wrist", "TAL-105", slug="wrist", frame=frame, page=1, pages=2)
    s1.views(wr, iso=False)
    s1.mass = printed_mass(wr)
    s1.center("front", (0, 0, 0), 11)
    s1.axis_line("front", (-18, 0, 0), (xe + 6, 0, 0))
    s1.ordinate("front", "h", [(0, 0, -16), (20.5, 0, -38.5), (xs - 28.5, 0, -17.6), (xs - 15.5, 0, -17.6),
                               (xs - 8, 0, -12), (57, 0, -12), (xe, 0, -38.5)], "below")
    s1.ordinate("front", "v", [(20.5, 0, -38.5), (20.5, 0, -17.6), (20.5, 0, -12.4),
                               (20.5, 0, -12), (20.5, 0, 13.6)], "left")
    s1.callout("front", (53, 0, -8), f"HEX WINDOW H({fmt(xs - 8)}, 57, -12, 8)\nTHRU (TAL-002)", 24, 56)
    s1.callout("top", (47, -19, 0), "CABLE CLIP (TAL-002)\nAT X 46, Z -30, ALONG X", 20, -8)
    s1.callout("front", (xs - 22, 0, -15), "PLUG BAY EXIT Y -30 TO 0\nSTEP Z -10.6, Y -18.6", -10, -56)
    s1.callout("front", s1.on_circle("front", (0, 0, 0), 16, 250), "CLEVIS (TAL-002)\nR = 24, H = 12", -2, -14)
    s1.ordinate("top", "v", [(-16, -ta.CLV_OUT, 0), (-16, ta.CLV_OUT, 0), (xe, -15, 0), (xe, 15, 0)], "left")
    s1.ordinate("top", "h", [(31, ta.CLV_OUT, 0)], "above")
    s1.callout("top", (xe - 1.5, 15 - 1.5, 13.6), "C4 (CRADLE, EDGES ∥ X)", 16, 16)
    s1.ordinate("right", "h", [(xe, -12.8, 0), (xe, 12.8, 0)], "below")
    s1.ordinate("right", "v", [(xe, 12.8, -35.5), (xe, 12.8, 10.5)], "right")
    s1.note("right", (xe, 0, -20), "INTERIOR OPEN TO +X", 26, -36)
    s1.cut_line("top", ((-20, 0, 0), (xe + 4, 0, 0)), np.array([0, 1, 0]), "A")
    s1.features("FEATURES (in order)", [
        "Clevis (TAL-002), R = 24, H = 12; the output plate's rectangle extended up to z 13.6, with the horn "
        "holes cut through it.",
        "Bridge lugs: box x 20.5 to 31, y ±22.9, z -12 to 13.6.",
        f"Cradle: box x 20.5 to {fmt(xe)}, y ±15, z -38.5 to 13.6; C4 on the edges parallel to X.",
        "Flat top: cut everything above z 13.6 (trims the clevis hubs), so the wrist prints upside down "
        "on one face.",
        "Cable clip (TAL-002) on the -Y face: centre (46, -15.2, -30), cable along X.",
        "Clevis gap cut (TAL-002); both horn pads stay.",
        pocket_note((xs, 0, 0), "+X", "+Z (body hangs down)", "-Y"),
        f"Cradle interior: cut box x {fmt(xs - 16)} to the +X end, y ±12.8, z -35.5 to 10.5 (the way in: the "
        "servo slides in along the roll axis, connector end first; its horns may stay on).",
        f"Plug bay exit to -Y: cut boxes x {fmt(xs - 28.5)} to {fmt(xs - 15.5)}: y -18.6 to 0, z -17.6 to -10.6; "
        "and y -30 to 0, z -17.6 to -12.4.",
        f"Side windows: hex window H({fmt(xs - 8)}, 57, -12, 8) through Y (TAL-002).",
        f"Case screw holes (TAL-002, seat 18.0), bottom face only, from the back wall: (y, z) = (±{fmt(HY)}, {fmt(HXB)}).",
    ])
    s1.notes(GENERAL + ["Fit the roll servo's 2X M2 × 5 case screws from the clevis side, before the wrist goes onto the forearm.",
                        print_note("TAL-105")])

    s2 = Sheet("Wrist", "TAL-105", slug="wrist", frame=frame, page=2, pages=2)
    sc = s2.section("A", wr, (0, 0, 0), (0, 1, 0), (1, 0, 0), (14, 120, 200, 283), scale=1.5,
                    margins=(36, 40, 20, 10))
    s2.scale_label = SCALE_NAME[sc]
    frame_mark(s2, "A", (xs, 0, 0), "P0", 4, 3)
    s2.ordinate("A", "h", [(xs - 16, 0, 10.5), (xs - 18, 0, HXB), (xs - 16.5, 0, HXB), (xs, 0, -35.5)], "below")
    s2.ordinate("A", "v", [(20.5, 0, -35.5), (20.5, 0, 10.5)], "left")
    s2.callout("A", s2.on_circle("A", (0, 0, 0), 21, 40), "R21 GAP (TAL-002)", 12, 18)
    s2.add_view("left", wr, "left", (14, 14, 160, 120), scale=1, margins=(36, 30, 30, 6))
    s2.ordinate("left", "h", [(-16, -HY, HXB), (-16, HY, HXB)], "below")
    s2.ordinate("left", "v", [(-16, HY, HXB), (-16, 0, 0)], "right")
    s2.callout("left", (20.5, HY + 2.1, HXB - 1), f"2X {CBORE}{DIA}4.2 TO X {fmt(xs - 18)}\n{DIA}2.4 TO POCKET",
               36, -24)
    s2.iso(wr, (200, 120, 272, 283), max_scale=1)
    s2.features("SECTION A-A AND LEFT VIEW", [
        "* A-A: XZ plane (cutting line on sheet 1), looking toward +Y. P0 = pocket P servo frame origin.",
        "* Left view: the cradle back wall, seen from -X between the clevis plates.",
    ])
    s2.notes(GENERAL[:1])
    return [s1, s2]


def sheet_gripper():
    parts, _ = ta.gripper_parts()
    g = as_shape(parts["gripper"])
    XJ, ZJ = ta.XJ, ta.ZJ
    frame = "origin on the roll axis at the roll horn face, X forward, Y = jaw pivot axis"
    s1 = Sheet("Gripper body (with fixed jaw)", "TAL-106", slug="gripper-body", frame=frame, page=1, pages=2)
    s1.views(g, iso=False, gap_ft=42)
    s1.mass = printed_mass(g)
    s1.center("front", (XJ, 0, ZJ), 18)
    s1.axis_line("front", (-6, 0, 0), (20, 0, 0))
    keel = [("K1", (24, -9.4)), ("K2", (56, -9.4)), ("K3", (56, -17)), ("K4", (30, -17)), ("K5", (24, -13))]
    finger = [("F1", (48, -9.4)), ("F2", (54, -9.4)), ("F3", (66, -0.5)), ("F4", (108, -0.5)),
              ("F5", (108, -6)), ("F6", (64, -17)), ("F7", (48, -17))]
    offs = {"K1": (-2, 2), "K3": (2, -2), "K4": (0, -2), "K5": (-2, -2), "F1": (-2, 2), "F2": (1, 2),
            "F3": (-2, 2), "F4": (1, 2), "F5": (1, -2), "F6": (1, -2), "F7": (-2, -2)}
    point_table(s1, "front", keel + finger, heading="KEEL K AND FINGER F PROFILES (X, Z), Y ±10",
                tag_offsets=offs)
    s1.ordinate("front", "h", [(0, 0, -9.4), (4, 0, -9.4), (9, 0, ZJ), (33, 0, ZJ), (XJ, 0, ZJ - 18)], "below")
    s1.ordinate("front", "v", [(0, 0, ZJ - 15.4), (0, 0, ZJ - HY), (0, 0, 0), (0, 0, ZJ), (0, 0, ZJ + HY),
                               (0, 0, ZJ + 15.4)], "left")
    s1.callout("front", s1.on_circle("front", (XJ, 0, ZJ), 18, 50), "R18", 10, 12)
    s1.callout("front", (30, 0, ZJ + 3), f"HEX WINDOW H(9, 33, {fmt(ZJ)}, 7)\nTHRU (TAL-002)", 50, 46)
    s1.callout("front", (73.5, 0, -1.1), "6X V-NOTCH, PITCH 6, x 72 TO 102\n(x, 0) (x+1.5, -1.6) (x+3, 0)",
               12, 34)
    s1.callout("front", (XJ + HXB + 1.5, 0, ZJ - HY - 1.5),
               f"4X {CBORE}{DIA}4.2 TO SEAT, {DIA}2.4 TO POCKET\n-Y FACE X {fmt(XJ + HXB)} (SEAT "
               f"{fmt(ta.SHELL_IN + 1.0)}), +Y FACE X {fmt(XJ - 29)} (SEAT {fmt(ta.FRONT_SEAT)})", 70, -30)
    s1.callout("front", (0.9, 0, ZJ + 15.4 - 0.9), "C3 (∥ Y)", -6, 24)
    s1.ordinate("top", "v", [(XJ, -ta.SHELL_OUT, 0), (108, -10, 0), (108, 10, 0), (XJ, ta.SHELL_OUT, 0)], "right")
    s1.ordinate("top", "v", [(4, -ta.SHELL_IN, 0), (4, ta.TOP_FACE + ta.FIT_C, 0)], "left")
    s1.ordinate("top", "h", [(XJ - 35.5, 15.9, 0), (XJ + 10.5, 15.9, 0)], "above")
    s1.ordinate("top", "h", [(XJ - 30.2, -19, 0), (XJ - 17.6, -18.4, 0), (XJ + 3.5, -18.4, 0)], "below")
    s1.note("top", (20, 8, 30), "SERVO ENTRY FROM THE TOP", 40, 30)
    s1.cut_line("top", ((-2, 4.95, 0), (110, 4.95, 0)), np.array([0, 1, 0]), "A")
    s1.features("FEATURES (in order)", [
        f"Disc {DIA}31 along X, x 0 to 4; C0.6 on the front edge.",
        f"Housing: box x 0 to {fmt(XJ)}, y ±18.4, z {fmt(ZJ - 15.4)} to {fmt(ZJ + 15.4)}; C3 on the edges "
        f"parallel to Y; plus a cylinder R18 about Y at (x, z) = ({fmt(XJ)}, {fmt(ZJ)}).",
        "Keel K1 to K5 and finger F1 to F7, each extruded y ±10.",
        f"Doubler on the -Y wall: box x {fmt(XJ - 31.0)} to {fmt(XJ - 16.4)}, y {fmt(-ta.SHELL_OUT - ta.DOUBLER)} "
        f"to -18.4, z {fmt(ZJ - 10.5)} to {fmt(ZJ + 15.4)} (leaves 0.8 over the bump channel).",
        pocket_note((XJ, 0, ZJ), "+Y", "+X", "-Z"),
        f"Servo entry from the top (pocket P run on up, TAL-001; horns off): case box x {fmt(XJ - 35.5)} to "
        f"{fmt(XJ + 10.5)}, y -16.9 to 15.9, z {fmt(ZJ - 12.8)} up (opens the top over the whole servo). +Y "
        f"wall: rib channel x {fmt(XJ - 34.8)} to {fmt(XJ - 3.5)}, y 15.5 to 17.1, z {fmt(ZJ - 7.5)} up; spline "
        f"slot x {fmt(XJ)} ±3.5, y to 17.6, z {fmt(ZJ)} up. -Y wall: bump channel x {fmt(XJ - 30.2)} to "
        f"{fmt(XJ - 17.2)}, y -19 to -16.5, z {fmt(ZJ - 9.7)} up; plug window x {fmt(XJ - 17.6)} to "
        f"{fmt(XJ - 10.6)}, z {fmt(ZJ - 10.6)} up; notch through the wall x {fmt(XJ - 17.6)} to {fmt(XJ + 3.5)}, "
        f"z {fmt(ZJ)} up (joins the plug window and the idler hole).",
        f"Side windows: hex window H(9, 33, {fmt(ZJ)}, 7) through Y (TAL-002).",
        "Grip notches: 6X triangle (x, 0) (x+1.5, -1.6) (x+3, 0), x = 72, 78 ... 102, through Y.",
        f"Roll horn (sheet 2): {DIA}7 centre, x -1 to 7. 4X {DIA}{fmt(M3_HOLE_D)} on {DIA}{fmt(PCD_D)} PCD at 45°, "
        f"x -1 to 3.5, "
        f"counterbore {DIA}6.5 from x 3.5 to 7 (M3 × 6 from inside the housing, with the short leg of a 2 mm "
        f"hex key); the two counterbores at z {fmt(-HK)} run on to x {fmt(ta.ROLL_CBORE_RUN)}, as grooves in the "
        f"pocket floor, so those screws can be laid in; centre opened to {DIA}8.6 "
        f"from x 3.5 to 7 (no thin web between the counterbores and the centre hole).",
        f"Case screw holes (TAL-002): +Y face (seat {fmt(ta.FRONT_SEAT)}) at x {fmt(XJ - 29)}, z "
        f"{fmt(ZJ - HY)} and {fmt(ZJ + HY)}; -Y face (seat {fmt(ta.SHELL_IN + 1.0)}) at x {fmt(XJ + HXB)}, z "
        f"{fmt(ZJ - HY)} and {fmt(ZJ + HY)}.",
    ])
    s1.notes(GENERAL + ["Fit the jaw servo from the top, shaft to +Y, with both horns off; then the case screws "
                        "(2X M2 × 6 on +Y, 2X M2 × 5 on -Y) and refit the horns.",
                        "The jaw servo's bus plugs come out through the -Y housing wall.",
                        print_note("TAL-106")])

    s2 = Sheet("Gripper body (with fixed jaw)", "TAL-106", slug="gripper-body", frame=frame, page=2, pages=2)
    sc = s2.add_view("left", g, "left", (14, 120, 150, 283), scale=2, margins=(36, 30, 20, 10))
    s2.scale_label = SCALE_NAME[sc]
    s2.center("left", (0, 0, 0), 15.5)
    s2.callout("left", s2.on_circle("left", (0, 0, 0), 15.5, 300), f"{DIA}31 DISC\nC0.6 AT X 4", 14, -16)
    horn_callouts(s2, "left", (0, 0, 0), (0, -1, 0), (0, 0, 1), dx=(-24, 26), dy=(-34, 40),
                  extra=f"\n{CBORE}{DIA}6.5 FROM INSIDE")
    s2.section("A", g, (0, HK, 0), (0, 1, 0), (1, 0, 0), (14, 14, 272, 122), scale=1.5,
               margins=(36, 36, 10, 6))
    s2.ordinate("A", "h", [(0, 0, -9.4), (3.5, 0, -HK - 3.25), (4, 0, -9.4), (7, 0, -HK - 3.25)], "below")
    s2.ordinate("A", "v", [(0, 0, -HK), (0, 0, HK)], "left")
    s2.callout("A", (5.25, HK, HK + 3.25), f"{CBORE}{DIA}6.5", 24, 20)
    s2.iso(g, (200, 130, 272, 283), max_scale=1.2)
    s2.features("LEFT VIEW AND SECTION A-A", [
        "* Left view: the disc face (roll horn side), seen from -X.",
        f"* A-A: plane y {fmt(round(ta.HORN_PCD_R / math.sqrt(2), 2))} through two roll horn screws (cutting "
        "line on sheet 1, top view), "
        "looking toward +Y.",
    ])
    s2.notes(GENERAL[:1])
    return [s1, s2]


SHEETS = [("TAL-000", sheet_assembly), ("TAL-001", sheet_servo), ("TAL-002", sheet_common),
          ("TAL-101", sheet_base), ("TAL-102", sheet_shoulder),
          ("TAL-103", lambda: sheet_arm(True)), ("TAL-104", lambda: sheet_arm(False)),
          ("TAL-105", sheet_wrist), ("TAL-106", sheet_gripper), ("TAL-107", sheet_jaw),
          ("TAL-108", lambda: sheet_cap(True)), ("TAL-110", sheet_idler), ("TAL-111", sheet_gauge)]


def build(only=None):
    """Build every sheet (or the parts named in `only`), numbered through the full set."""
    sheets = []
    for part_no, make in SHEETS:
        if only and part_no not in only:
            continue
        made = make()
        sheets.extend(made if isinstance(made, list) else [made])
    for i, s in enumerate(sheets, start=1):
        s.number, s.total = i, len(sheets)
    for s in sheets:
        s.title_block()
    return sheets


def stem(s):
    return f"{s.part_no}_{s.slug}_rev-{REV}"


def main():
    """Write the full set and the two work sets as PDFs, one PDF per part, and a PNG per sheet.

    Sheet numbers ("sheet n of N") follow the full set in every file, so a sheet is
    identical wherever it appears.
    """
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", help="part numbers to draw (previews only, for checking)")
    args = ap.parse_args()
    sheets_dir, preview_dir, sets_dir = OUT / "sheets", OUT / "previews", OUT / "work-sets"
    for d in (OUT, sheets_dir, preview_dir, sets_dir):
        d.mkdir(exist_ok=True)
    sheets = build(args.only)
    if args.only:
        for s in sheets:
            name = stem(s) + (f"_sh{s.page}" if s.pages > 1 else "")
            s.fig.savefig(preview_dir / f"{name}.png", dpi=110)
            print("preview", name)
        return
    for d in (sheets_dir, preview_dir):
        for f in list(d.glob("*.pdf")) + list(d.glob("*.png")):
            f.unlink()
    full_path = OUT / f"TAL-000_drawing-set_rev-{REV}.pdf"
    set_paths = {k: sets_dir / f"TAL-SET-{k}_{slug}_rev-{REV}.pdf" for k, (slug, _, _) in WORK_SETS.items()}
    pdfs = {k: PdfPages(p) for k, p in set_paths.items()}
    parts = {}
    work = {k: [0, 0, 0] for k in WORK_SETS}            # parts, sheets, feature steps
    try:
        with PdfPages(full_path) as full:
            for s in sheets:
                ws = work_set(s.part_no)
                full.savefig(s.fig)
                for k in ([ws] if ws else WORK_SETS):
                    pdfs[k].savefig(s.fig)
                if ws:
                    work[ws][0] += s.page == 1
                    work[ws][1] += 1
                    work[ws][2] += s.n_features
                if s.part_no not in parts:
                    parts[s.part_no] = PdfPages(sheets_dir / f"{stem(s)}.pdf")
                parts[s.part_no].savefig(s.fig)
                name = stem(s) + (f"_sh{s.page}" if s.pages > 1 else "")
                s.fig.savefig(preview_dir / f"{name}.png", dpi=110)
                plt.close(s.fig)
                print(f"sheet {s.number}/{s.total}: {name}  [{'set ' + ws if ws else 'both sets'}]")
    finally:
        for pdf in list(pdfs.values()) + list(parts.values()):
            pdf.close()
    print(f"wrote {full_path}")
    for k, p in set_paths.items():
        print(f"wrote {p}  ({work[k][0]} parts, {work[k][1]} sheets, {work[k][2]} modelling steps)")


if __name__ == "__main__":
    main()
