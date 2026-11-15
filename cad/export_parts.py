"""One STEP per printed part, in the part's own frame: the frame its drawing sheet uses.

The coordinates in each file are the ones on the part's sheets, so a point read off the
drawing is the same point in the STEP. Use these to check a part modelled by hand (for
example in SolidWorks) against the design, or to import a single part. The assembly STEPs
(talos_arm.py) stay the reference for how the parts go together.

    python export_parts.py              # writes parts/<part no>_<name>_rev-A.step
    python export_parts.py --out DIR    # somewhere else
"""

import argparse
from pathlib import Path

import cadquery as cq

import talos_arm as ta

REV = "A"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(Path(__file__).with_name("parts")))
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(exist_ok=True)
    for part_no, (slug, qty, solid) in ta.printed_parts().items():
        shape = solid.val() if isinstance(solid, cq.Workplane) else solid
        path = out / f"{part_no}_{slug}_rev-{REV}.step"
        cq.exporters.export(cq.Workplane().add(shape), str(path))
        bb = shape.BoundingBox()
        print(f"wrote {path.name}: x{qty}, {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, "
              f"{shape.Volume() / 1000:.1f} cm3")


if __name__ == "__main__":
    main()
