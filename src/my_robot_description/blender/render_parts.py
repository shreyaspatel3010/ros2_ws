"""Render textured turntables (PNG frames) or preview stills of exported parts
(.obj with its baked maps, or a self-contained .glb).

    blender -b --factory-startup -P blender/render_parts.py -- <out_dir> <frames|0 for stills> <obj> [<obj> ...]

Each OBJ is re-imported from disk, so the render shows exactly the textured
asset that RViz / Gazebo will load.
"""
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bpy
from mathutils import Vector
import scene_util as su
import urdf_blender as ub

argv = sys.argv[sys.argv.index("--") + 1:]
out, frames, objs = argv[0], int(argv[1]), argv[2:]
RES = int(os.environ.get("RES", "480"))

for path in objs:
    name = os.path.splitext(os.path.basename(path))[0]
    su.reset()
    if path.endswith(".glb"):
        bpy.ops.import_scene.gltf(filepath=path)        # self-contained: textures come from the file
    else:
        bpy.ops.wm.obj_import(filepath=path, forward_axis="Y", up_axis="Z")
        ub.upgrade_baked_materials()
    parts = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    for o in parts:
        print("MATERIAL", o.name, [(m.name, [n.image.name for n in m.node_tree.nodes if n.type == "TEX_IMAGE"]) for m in o.data.materials])
    mn = Vector((1e9,) * 3); mx = -mn
    for o in parts:
        for c in o.bound_box:
            w = o.matrix_world @ Vector(c); mn = Vector(map(min, mn, w)); mx = Vector(map(max, mx, w))
    ctr, size = (mn + mx) / 2, (mx - mn)
    # put the part on the floor, centred, so it orbits nicely
    for o in parts:
        o.location -= Vector((ctr.x, ctr.y, mn.z))
    ext = max(size.x, size.y, size.z)
    su.studio(res=(RES, RES), samples=48, floor=True, floor_z=0.0)
    for l in [o for o in bpy.context.scene.objects if o.type == "LIGHT"]:
        l.location = Vector(l.location) * ext * 1.6 + Vector((0, 0, ext * 0.4))
        l.data.energy *= (ext * 1.6) ** 2 / 9.0
        l.data.size *= ext
        tgt = Vector((0, 0, size.z / 2))
        l.rotation_euler = (tgt - l.location).to_track_quat("-Z", "Y").to_euler()
    tgt = (0, 0, size.z * 0.5)
    lens = 85
    dist = ext * 2.9 * (50 / lens) * 1.55
    if frames > 0:
        cam, piv = su.orbit_rig(tgt, dist * 0.94, dist * 0.34, lens, frames)
        su.render_frames(os.path.join(out, name), "f")
    else:
        for tag, ang in (("a", -40), ("b", 50), ("c", 160)):
            a = math.radians(ang)
            su.camera((dist * math.cos(a) * 0.94, dist * math.sin(a) * 0.94, size.z * 0.5 + dist * 0.3), tgt, lens)
            su.render_still(os.path.join(out, "%s_%s.png" % (name, tag)))
    print("RENDERED", name)
