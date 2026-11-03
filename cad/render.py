"""Render the Talos assembly to PNG and report part-to-part interference.

    python render.py                # ready pose, renders into renders/
    python render.py --pose zero --no-check
"""

import argparse
import itertools
from pathlib import Path

import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
import numpy as np

import talos_arm

VIEWS = {
    "iso": ((1.0, -1.25, 0.75), 30),
    "side": ((0.0, -1.0, 0.0), 30),
    "front": ((1.0, 0.0, 0.15), 30),
    "rear_iso": ((-1.0, 1.1, 0.6), 30),
}


def to_polydata(shape, tol=0.15):
    verts, tris = shape.tessellate(tol, 0.3)
    pts = np.array([(v.x, v.y, v.z) for v in verts], dtype=float)
    cells = np.hstack([np.full((len(tris), 1), 3), np.array(tris)]).astype(np.int64).ravel()
    poly = vtk.vtkPolyData()
    p = vtk.vtkPoints()
    p.SetData(numpy_to_vtk(pts, deep=True))
    poly.SetPoints(p)
    ca = vtk.vtkCellArray()
    ca.SetCells(len(tris), numpy_to_vtkIdTypeArray(cells, deep=True))
    poly.SetPolys(ca)
    normals = vtk.vtkPolyDataNormals()
    normals.SetInputData(poly)
    normals.SetFeatureAngle(35)
    normals.Update()
    return normals.GetOutput()


def render(parts, out_dir, size=(1600, 1200)):
    out_dir.mkdir(parents=True, exist_ok=True)
    ren = vtk.vtkRenderer()
    ren.SetBackground(0.80, 0.81, 0.83)
    ren.SetBackground2(0.58, 0.60, 0.63)
    ren.GradientBackgroundOn()
    for name, shape, color in parts:
        m = vtk.vtkPolyDataMapper()
        m.SetInputData(to_polydata(shape))
        a = vtk.vtkActor()
        a.SetMapper(m)
        r, g, b, _ = color.toTuple() if color else (0.7, 0.7, 0.7, 1)
        prop = a.GetProperty()
        prop.SetColor(r, g, b)
        prop.SetSpecular(0.35)
        prop.SetSpecularPower(25)
        prop.SetDiffuse(0.85)
        prop.SetAmbient(0.18)
        ren.AddActor(a)
    win = vtk.vtkRenderWindow()
    win.SetOffScreenRendering(1)
    win.SetSize(*size)
    win.AddRenderer(ren)
    ren.UseFXAAOn()
    light = vtk.vtkLight()
    light.SetLightTypeToCameraLight()
    light.SetPosition(0.4, 0.6, 1)
    ren.AddLight(light)
    paths = []
    for vname, (direction, va) in VIEWS.items():
        cam = ren.GetActiveCamera()
        cam.SetFocalPoint(0, 0, 0)
        cam.SetPosition(*direction)
        cam.SetViewUp(0, 0, 1)
        cam.SetViewAngle(va)
        ren.ResetCamera()
        cam.Zoom(0.95)
        ren.ResetCameraClippingRange()
        win.Render()
        w2i = vtk.vtkWindowToImageFilter()
        w2i.SetInput(win)
        w2i.Update()
        wr = vtk.vtkPNGWriter()
        path = out_dir / f"{vname}.png"
        wr.SetFileName(str(path))
        wr.SetInputConnection(w2i.GetOutputPort())
        wr.Write()
        paths.append(path)
    return paths


def interference(parts, min_vol=1.0):
    hits = []
    boxes = [(n, s, s.BoundingBox()) for n, s, _ in parts]
    for (na, sa, ba), (nb, sb, bb) in itertools.combinations(boxes, 2):
        if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin
                or bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
            continue
        v = sa.intersect(sb).Volume()
        if v > min_vol:
            hits.append((na, nb, v))
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="ready", choices=sorted(talos_arm.POSES))
    ap.add_argument("--no-check", action="store_true")
    args = ap.parse_args()
    parts = list(talos_arm.flatten(talos_arm.build(talos_arm.POSES[args.pose])))
    out = Path(__file__).with_name("renders") / args.pose
    for p in render(parts, out):
        print("render", p)
    if not args.no_check:
        hits = interference(parts)
        print("interference:", "none" if not hits else "")
        for a, b, v in hits:
            print(f"  {a} x {b}: {v:.1f} mm^3")


if __name__ == "__main__":
    main()
