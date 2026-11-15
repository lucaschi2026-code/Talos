"""Print-ready files: every printed part turned onto its print face, one 3MF per part.

Orientation and support notes come from PRINT_SETUP in talos_arm.py, so the files,
the drawings and the README agree. 3MF carries millimetres, so parts import at the
right size. The mesh is welded into one closed surface (no open edges, which some
slicers flag), with a chord tolerance of 0.01 mm so the M3 and M2 holes stay round.

    python print_files.py      # checks each part (part_checks.py), then writes print/<part no>_<name>_rev-A.3mf
"""

import zipfile
from pathlib import Path

import numpy as np

import talos_arm as ta

REV = "A"
OUT = Path(__file__).with_name("print")
BED = (220.0, 220.0, 240.0)  # Creality Ender 3 V3 KE build volume, mm
MARGIN = 5.0                 # kept clear of the bed edge on every side (skirt or brim)

CONTENT_TYPES = """<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>"""
RELS = """<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>"""


def welded_mesh(solid, tol=0.01, ang=0.2):
    """Triangles of the solid with shared vertices, outward-facing."""
    verts, tris = solid.tessellate(tol, ang)
    V = np.array([(v.x, v.y, v.z) for v in verts])
    T = np.array(tris)
    key = np.round(V / 1e-5).astype(np.int64)
    _, first, inverse = np.unique(key, axis=0, return_index=True, return_inverse=True)
    V, T = V[first], inverse.reshape(-1)[T]
    T = T[(T[:, 0] != T[:, 1]) & (T[:, 1] != T[:, 2]) & (T[:, 0] != T[:, 2])]
    return V, T


def check(V, T):
    """(volume, open edges, non-manifold edges)."""
    a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    vol = np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6
    e = np.sort(np.vstack([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]]), axis=1)
    _, cnt = np.unique(e, axis=0, return_counts=True)
    return vol, int((cnt == 1).sum()), int((cnt > 2).sum())


def write_3mf(path, name, V, T):
    verts = "\n".join(f'<vertex x="{x:.4f}" y="{y:.4f}" z="{z:.4f}"/>' for x, y, z in V)
    tris = "\n".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in T)
    model = (f'<?xml version="1.0" encoding="UTF-8"?>\n'
             f'<model unit="millimeter" xml:lang="en-US" '
             f'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">\n'
             f'<metadata name="Title">{name}</metadata>\n'
             f'<resources><object id="1" type="model" name="{name}"><mesh>\n'
             f'<vertices>\n{verts}\n</vertices>\n<triangles>\n{tris}\n</triangles>\n'
             f'</mesh></object></resources>\n<build><item objectid="1"/></build>\n</model>\n')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", CONTENT_TYPES)
        z.writestr("_rels/.rels", RELS)
        z.writestr("3D/3dmodel.model", model)


def read_3mf(path):
    """(vertices, triangles) back out of a written file."""
    import re
    with zipfile.ZipFile(path) as z:
        m = z.read("3D/3dmodel.model").decode()
    V = np.array(re.findall(r'<vertex x="([^"]+)" y="([^"]+)" z="([^"]+)"/>', m), float)
    T = np.array(re.findall(r'<triangle v1="(\d+)" v2="(\d+)" v3="(\d+)"/>', m), int)
    return V, T


def first_layer(s, h=0.2):
    """(area mm2, islands) of the part's slice 0 to h above the bed."""
    import cadquery as cq
    bb = s.BoundingBox()
    sl = s.intersect(cq.Solid.makeBox(bb.xlen + 20, bb.ylen + 20, h, cq.Vector(bb.xmin - 10, bb.ymin - 10, 0)))
    return sl.Volume() / h, sorted((p.Volume() / h for p in sl.Solids()), reverse=True)


def main():
    """Every part is checked before its file is written; nothing is written for a part that fails."""
    import part_checks as pc
    OUT.mkdir(exist_ok=True)
    for f in OUT.glob("*.3mf"):
        f.unlink()
    failed = []
    for part_no, (slug, qty, solid) in ta.printed_parts().items():
        down, note = ta.PRINT_SETUP[part_no]
        s = ta.on_bed(solid, down)
        V, T = welded_mesh(s)
        vol, open_edges, nonmanifold = check(V, T)
        bb = s.BoundingBox()
        n_ok, fails = pc.check_part(part_no, solid, V, T)
        area, islands = first_layer(s)
        gate = {"closed mesh": open_edges == 0 and nonmanifold == 0,
                "mesh volume = solid": abs(vol - s.Volume()) < 5e-3 * s.Volume(),
                "on the bed": abs(V[:, 2].min()) < 1e-3,
                f"fits the {BED[0]:.0f} x {BED[1]:.0f} x {BED[2]:.0f} bed": (
                    max(bb.xlen, bb.ylen) <= max(BED[:2]) - 2 * MARGIN
                    and min(bb.xlen, bb.ylen) <= min(BED[:2]) - 2 * MARGIN and bb.zlen <= BED[2]),
                "first layer: no specks or slivers (every island 5 mm2 or more)": min(islands) >= 5.0,
                f"{n_ok}/{len(pc.PROBES[part_no])} dimensions": not fails}
        bad = [k for k, ok in gate.items() if not ok]
        path = OUT / f"{part_no}_{slug}_rev-{REV}.3mf"
        if bad:
            failed.append(part_no)
            print(f"{path.name}: NOT WRITTEN, failed {', '.join(bad)}")
            continue
        write_3mf(path, f"{part_no} {slug}", V, T)
        V2, T2 = read_3mf(path)
        assert len(V2) == len(V) and len(T2) == len(T) and np.abs(V2 - V).max() < 1e-4, path
        pieces = "one piece" if len(islands) == 1 else " + ".join(f"{a:.0f}" for a in islands) + " mm2"
        print(f"{path.name}: x{qty}, {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm, {len(T)} triangles, "
              f"mesh {vol:.0f} / solid {s.Volume():.0f} mm3, closed, lowest point z {V[:, 2].min():+.4f}; "
              f"first layer {area:.0f} mm2 in {pieces}; {n_ok}/{len(pc.PROBES[part_no])} dimensions OK; {note}")
    for name, ok, txt in pc.fit_checks():
        print(f"fit {'OK  ' if ok else 'FAIL'} {name}: {txt}")
        if not ok:
            failed.append(name)
    if failed:
        raise SystemExit(f"failed: {', '.join(failed)}")


if __name__ == "__main__":
    main()
