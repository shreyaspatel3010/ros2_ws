"""Light collision meshes for the arm / hand links (physics + MoveIt).

Decimates the cleaned visual meshes (meshes/arms/<link>.obj) to ~1.5k triangles
and writes meshes/arms/collision/<link>.stl.  The originals had up to 440k
triangles per link, which made Gazebo contacts and MoveIt collision checks slow.

    blender -b --factory-startup -P blender/build_arm_collision.py
"""
import glob, os
import bpy

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(PKG, "meshes", "arms", "collision")
os.makedirs(OUT, exist_ok=True)
bpy.ops.wm.read_factory_settings(use_empty=True)
for f in sorted(glob.glob(os.path.join(PKG, "meshes", "arms", "*.obj"))):
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=f, forward_axis="Y", up_axis="Z")
    o = [x for x in bpy.data.objects if x not in before][0]
    bpy.ops.object.select_all(action="DESELECT"); o.select_set(True); bpy.context.view_layer.objects.active = o
    tris = sum(len(p.vertices) - 2 for p in o.data.polygons)
    if tris > 1500:
        d = o.modifiers.new("d", "DECIMATE"); d.ratio = 1500 / tris
        bpy.ops.object.modifier_apply(modifier="d")
    name = os.path.splitext(os.path.basename(f))[0]
    bpy.ops.wm.stl_export(filepath=os.path.join(OUT, name + ".stl"), export_selected_objects=True,
                          forward_axis="Y", up_axis="Z", ascii_format=False)
    bpy.data.objects.remove(o, do_unlink=True)
print("COLLISION", len(glob.glob(os.path.join(OUT, "*.stl"))))
