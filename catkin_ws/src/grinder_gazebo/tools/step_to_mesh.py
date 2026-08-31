#!/usr/bin/env python3
"""Inspect a STEP assembly and optionally export a Gazebo-friendly STL mesh.

Run this script with FreeCAD's Python interpreter, for example:

    freecadcmd tools/step_to_mesh.py input.step output.stl --report output.json

The report records the native CAD bounding box so the URDF can center and place
the visual mesh deterministically. STEP coordinates are preserved in the STL;
the URDF applies the millimetre-to-metre scale and origin transform.
"""

import argparse
import json
import os
import sys
import time

import FreeCAD as App
import Mesh
import Part


def _vector_payload(vector):
    return [float(vector.x), float(vector.y), float(vector.z)]


def inspect_shape(shape, source_path):
    bounds = shape.BoundBox
    solids = list(shape.Solids)
    solid_volume = sum(float(item.Volume) for item in solids)
    if solid_volume > 0.0:
        center_of_mass = App.Vector(
            sum(float(item.CenterOfMass.x) * float(item.Volume) for item in solids) / solid_volume,
            sum(float(item.CenterOfMass.y) * float(item.Volume) for item in solids) / solid_volume,
            sum(float(item.CenterOfMass.z) * float(item.Volume) for item in solids) / solid_volume,
        )
    else:
        center_of_mass = bounds.Center
    return {
        "source": os.path.abspath(source_path),
        "source_size_bytes": int(os.path.getsize(source_path)),
        "native_unit": "millimeter",
        "bounding_box_mm": {
            "min": [float(bounds.XMin), float(bounds.YMin), float(bounds.ZMin)],
            "max": [float(bounds.XMax), float(bounds.YMax), float(bounds.ZMax)],
            "size": [float(bounds.XLength), float(bounds.YLength), float(bounds.ZLength)],
            "center": _vector_payload(bounds.Center),
        },
        "topology": {
            "solids": len(solids),
            "shells": len(shape.Shells),
            "faces": len(shape.Faces),
            "edges": len(shape.Edges),
            "vertices": len(shape.Vertexes),
        },
        "volume_mm3": float(solid_volume),
        "center_of_mass_mm": _vector_payload(center_of_mass),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("step", help="Input STEP assembly")
    parser.add_argument("stl", nargs="?", help="Optional output STL mesh")
    parser.add_argument("--report", help="Optional JSON inspection report")
    parser.add_argument(
        "--linear-deflection-mm",
        type=float,
        default=2.0,
        help="Absolute mesh chord tolerance in millimetres (default: 2.0)",
    )
    parser.add_argument(
        "--angular-deflection-rad",
        type=float,
        default=0.35,
        help="Mesh angular tolerance in radians (default: 0.35)",
    )
    env_step = os.environ.get("GRINDER_STEP_INPUT", "").strip()
    if env_step:
        env_args = [env_step]
        env_stl = os.environ.get("GRINDER_STEP_OUTPUT_STL", "").strip()
        env_report = os.environ.get("GRINDER_STEP_REPORT", "").strip()
        if env_stl:
            env_args.append(env_stl)
        if env_report:
            env_args.extend(["--report", env_report])
        env_linear = os.environ.get("GRINDER_STEP_LINEAR_DEFLECTION_MM", "").strip()
        env_angular = os.environ.get("GRINDER_STEP_ANGULAR_DEFLECTION_RAD", "").strip()
        if env_linear:
            env_args.extend(["--linear-deflection-mm", env_linear])
        if env_angular:
            env_args.extend(["--angular-deflection-rad", env_angular])
        args = parser.parse_args(env_args)
    else:
        args = parser.parse_args()

    step_path = os.path.abspath(args.step)
    if not os.path.isfile(step_path):
        parser.error("STEP file does not exist: {}".format(step_path))

    started = time.time()
    print("Loading STEP assembly: {}".format(step_path), flush=True)
    shape = Part.read(step_path)
    if shape.isNull():
        raise RuntimeError("FreeCAD returned a null shape for {}".format(step_path))

    report = inspect_shape(shape, step_path)
    report["load_seconds"] = round(time.time() - started, 3)
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True), flush=True)

    if args.stl:
        stl_path = os.path.abspath(args.stl)
        parent = os.path.dirname(stl_path)
        if parent and not os.path.isdir(parent):
            os.makedirs(parent)
        mesh_started = time.time()
        print("Tessellating STL: {}".format(stl_path), flush=True)
        vertices, facets = shape.tessellate(max(0.05, float(args.linear_deflection_mm)))
        mesh = Mesh.Mesh((vertices, facets))
        mesh.write(stl_path)
        report["mesh"] = {
            "path": stl_path,
            "facets": int(mesh.CountFacets),
            "points": int(mesh.CountPoints),
            "size_bytes": int(os.path.getsize(stl_path)),
            "linear_deflection_mm": float(args.linear_deflection_mm),
            "angular_deflection_rad": float(args.angular_deflection_rad),
            "note": "FreeCAD 0.18 Part.tessellate uses linear deflection; angular value is recorded for reproducibility but is not consumed by this fallback exporter.",
            "export_seconds": round(time.time() - mesh_started, 3),
        }

    if args.report:
        report_path = os.path.abspath(args.report)
        parent = os.path.dirname(report_path)
        if parent and not os.path.isdir(parent):
            os.makedirs(parent)
        with open(report_path, "w", encoding="utf-8") as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
            handle.write("\n")
        print("Wrote report: {}".format(report_path), flush=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
