"""Turn the original arm / hand STLs into clean, seamless, textured meshes
without redesigning them.

For every arm link: the STL is scaled to metres exactly as the URDF used it,
voxel-remeshed at sub-millimetre resolution (welds the separate shells, gaps
and non-manifold edges into one watertight surface), lightly relaxed,
decimated to a sensible polycount, then UV-unwrapped and baked with the same
material set as the legs.  Output per link:
    meshes/arms/<link>.obj/.mtl/.png  (URDF visuals)
    meshes/arms/glb/<link>.glb        (self-contained PBR glTF)
The original STLs stay in meshes/left_arm, meshes/right_arm (used for collision).

    blender -b --factory-startup -P blender/build_arms.py -- <robot.urdf> [link,link,...]
"""
import math, os, sys
import xml.etree.ElementTree as ET
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree
import build_parts as bp

argv = sys.argv[sys.argv.index("--") + 1:]
URDF = argv[0]
ONLY = set(argv[1].split(",")) if len(argv) > 1 else None
OUT = os.path.join(bp.PKG, "meshes", "arms")
os.makedirs(os.path.join(OUT, "glb"), exist_ok=True)


def role(link):
    """Colour language shared with the legs."""
    n = link.lower()
    if "upper" in n and ("finger" in n or "thomb" in n):
        return "accent"            # fingertips
    if "wrist" in n:
        return "accent"
    if "elbow_inword" in n or "elbow_outword" in n or "elbow_midle" in n or "internal_support" in n:
        return "gunmetal"
    return "white"


def arm_links():
    root = ET.parse(URDF).getroot()
    out = []
    for l in root.findall("link"):
        v = l.find("visual")
        if v is None or v.find("geometry/mesh") is None:
            continue
        m = v.find("geometry/mesh")
        fn = m.get("filename")
        if "/left_arm/" not in fn and "/right_arm/" not in fn:
            continue
        scale = [float(x) for x in m.get("scale", "1 1 1").split()]
        path = os.path.join(bp.PKG, fn.split("my_robot_description/")[1])
        out.append((l.get("name"), path, scale))
    return out


def clean_remesh(obj, n_orig):
    me = obj.data
    diag = (Vector(obj.bound_box[6]) - Vector(obj.bound_box[0])).length
    voxel = min(0.0011, max(0.00035, diag / 260.0))
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode="EDIT"); bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.remove_doubles(threshold=1e-6)
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode="OBJECT")
    rm = obj.modifiers.new("remesh", "REMESH"); rm.mode = "VOXEL"; rm.voxel_size = voxel; rm.adaptivity = 0.0
    sm = obj.modifiers.new("relax", "CORRECTIVE_SMOOTH"); sm.factor = 0.5; sm.iterations = 4
    sm.smooth_type = "SIMPLE"; sm.use_only_smooth = True
    bpy.ops.object.modifier_apply(modifier="remesh")
    bpy.ops.object.modifier_apply(modifier="relax")
    target = min(16000 if diag > 0.09 else 9000 if diag > 0.04 else 4000, max(1500, n_orig))   # triangles
    tris = sum(len(p.vertices) - 2 for p in obj.data.polygons)
    if tris > target:
        dc = obj.modifiers.new("dec", "DECIMATE"); dc.ratio = target / tris; dc.use_collapse_triangulate = True
        bpy.ops.object.modifier_apply(modifier="dec")
    bpy.ops.object.shade_smooth_by_angle(angle=math.radians(38))
    return voxel


def deviation(orig_pts, obj):
    bvh = BVHTree.FromObject(obj, bpy.context.evaluated_depsgraph_get())
    d = [bvh.find_nearest(p)[3] for p in orig_pts]
    d.sort()
    return sum(d) / len(d), d[int(len(d) * 0.99)]


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    report = []
    for link, path, scale in arm_links():
        if ONLY and link not in ONLY:
            continue
        bpy.ops.object.select_all(action="DESELECT")
        bpy.ops.wm.stl_import(filepath=path)
        obj = bpy.context.selected_objects[0]
        obj.name = link
        obj.scale = scale
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        step = max(1, len(obj.data.vertices) // 4000)
        orig = [v.co.copy() for i, v in enumerate(obj.data.vertices) if i % step == 0]
        nf0 = len(obj.data.polygons)
        voxel = clean_remesh(obj, nf0)
        mean, p99 = deviation(orig, obj)
        obj.data.materials.clear()
        obj.data.materials.append(bp.make_material(link, role(link)))
        diag = (Vector(obj.bound_box[6]) - Vector(obj.bound_box[0])).length
        bp.bake(obj, link, OUT, 1024 if diag > 0.09 else 512)
        bp.export_part(obj, OUT, link)
        report.append((link, nf0, len(obj.data.polygons), voxel, mean, p99))
        bpy.data.objects.remove(obj, do_unlink=True)
    print("\nARM REPORT  link  faces_in -> faces_out  voxel_mm  mean_dev_mm  p99_dev_mm")
    for r in report:
        print("ARM %-30s %7d -> %6d  %.2f  %.3f  %.3f" % (r[0], r[1], r[2], r[3] * 1e3, r[4] * 1e3, r[5] * 1e3))


main()
