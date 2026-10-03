"""Assemble the whole robot from its URDF and export it as glTF binaries:

    meshes/robot/hruh_robot.glb       standing pose
    meshes/robot/hruh_robot_walk.glb  same model with a baked walking animation
                                      (driven by scripts/hruh_walker.py, the same
                                      gait generator the ROS node uses)

    blender -b --factory-startup -P blender/export_robot_glb.py -- <robot.urdf> [preview_dir]
"""
import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(PKG, "scripts"))
import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector
import scene_util as su
import urdf_blender as ub
import hruh_walker as hw

argv = sys.argv[sys.argv.index("--") + 1:]
urdf_path = argv[0]
preview = argv[1] if len(argv) > 1 else None
OUT = os.path.join(PKG, "meshes", "robot")
os.makedirs(OUT, exist_ok=True)
FPS = 25
MAX_FACES = 6000       # decimate the very dense original arm / hand STLs for the GLB


def to_matrix(Rb, pb):
    M = Matrix([list(Rb[0]) + [pb[0]], list(Rb[1]) + [pb[1]], list(Rb[2]) + [pb[2]], [0, 0, 0, 1]])
    return M


su.reset()
robot = ub.Robot(urdf_path, PKG)
robot.build()
ub.upgrade_baked_materials()
for o in robot.col.objects:
    if o.type == "MESH" and len(o.data.polygons) > MAX_FACES:
        md = o.modifiers.new("dec", "DECIMATE"); md.ratio = MAX_FACES / len(o.data.polygons)
        bpy.context.view_layer.objects.active = o
        bpy.ops.object.modifier_apply(modifier=md.name)
    if o.type == "MESH":
        for p in o.data.polygons:
            p.use_smooth = True
        bpy.context.view_layer.objects.active = o
        o.select_set(True)
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))
        o.select_set(False)

urdf_xml = open(urdf_path).read()
gen = hw.WalkingPatternGenerator(urdf_xml)
sc = bpy.context.scene
sc.render.fps = FPS


def apply(q, Rb, pb):
    robot.set_joints(q, to_matrix(Rb, pb))


# --- static, standing pose
q, Rb, pb = gen.update()
apply(q, Rb, pb)


def export(path, animated):
    bpy.ops.object.select_all(action="DESELECT")
    for o in robot.col.objects:
        o.select_set(True)
    bpy.ops.export_scene.gltf(filepath=path, export_format="GLB", use_selection=True, export_yup=True,
                              export_apply=True, export_animations=animated, export_frame_range=animated,
                              export_force_sampling=True, export_optimize_animation_size=True,
                              export_image_format="JPEG", export_image_quality=88)
    print("EXPORT %s %.1f MB" % (os.path.basename(path), os.path.getsize(path) / 1e6))


export(os.path.join(OUT, "hruh_robot.glb"), False)

# --- walking animation: stand 1 s, walk 0.25 m/s, turn a little, stop
script = [(1.0, (0, 0, 0)), (5.0, (0.25, 0, 0)), (2.4, (0.15, 0, 0.35)), (2.6, (0, 0, 0))]
frame, sub = 1, int(round(1.0 / (FPS * gen.p.dt)))
for dur, cmd in script:
    for _ in range(int(dur * FPS)):
        gen.set_command(*cmd)
        for _ in range(sub):
            q, Rb, pb = gen.update()
        apply(q, Rb, pb)
        robot.keyframe(frame)
        frame += 1
sc.frame_start, sc.frame_end = 1, frame - 1
export(os.path.join(OUT, "hruh_robot_walk.glb"), True)

if preview:
    # a few frames as stills to eyeball the motion
    su.studio(res=(640, 800), samples=32)
    for fr in (40, 70, 90, 110, 130, 150):
        sc.frame_set(fr)
        base = robot.empties[robot.root_link()].matrix_world.translation
        su.camera((base.x + 1.2, base.y - 3.2, 1.0), (base.x, base.y, 0.78), 60)
        su.render_still(os.path.join(preview, "walk_%03d.png" % fr))
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(preview, "walk.blend"))
