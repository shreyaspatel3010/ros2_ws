"""Shared studio scene setup for preview stills and turntable GIFs."""
import math, os
import bpy
from mathutils import Vector


def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def studio(engine="BLENDER_EEVEE", res=(640, 640), samples=32, floor=True, floor_z=0.0, bg=(0.06, 0.07, 0.09)):
    sc = bpy.context.scene
    try:
        sc.render.engine = engine
    except TypeError:
        sc.render.engine = "BLENDER_EEVEE_NEXT"
    if sc.render.engine.startswith("BLENDER_EEVEE"):
        sc.eevee.taa_render_samples = samples
        try:
            sc.eevee.use_shadows = True
            sc.eevee.use_raytracing = False
        except AttributeError:
            pass
    else:
        sc.cycles.samples = samples
        sc.cycles.device = "CPU"
        sc.cycles.use_denoising = True
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.film_transparent = False
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.look = "AgX - Medium High Contrast"
    w = bpy.data.worlds.new("w"); sc.world = w; w.use_nodes = True
    bgn = w.node_tree.nodes["Background"]
    bgn.inputs[0].default_value = (*bg, 1); bgn.inputs[1].default_value = 1.0

    def light(name, kind, loc, energy, size, rot=None, color=(1, 1, 1)):
        d = bpy.data.lights.new(name, kind); d.energy = energy; d.color = color
        if kind == "AREA":
            d.size = size
        o = bpy.data.objects.new(name, d); sc.collection.objects.link(o); o.location = loc
        tgt = Vector((0, 0, 0.8)); o.rotation_euler = (tgt - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
        return o
    light("key", "AREA", (2.5, -2.5, 3.0), 520, 2.5, color=(1.0, 0.96, 0.9))
    light("fill", "AREA", (-3.0, -1.5, 1.6), 160, 3.0, color=(0.85, 0.9, 1.0))
    light("rim", "AREA", (-0.5, 3.0, 2.8), 520, 2.0, color=(0.8, 0.9, 1.0))
    if floor:
        bpy.ops.mesh.primitive_plane_add(size=30, location=(0, 0, floor_z))
        f = bpy.context.object; f.name = "floor"
        m = bpy.data.materials.new("floor"); m.use_nodes = True
        nt = m.node_tree; b = nt.nodes["Principled BSDF"]
        b.inputs["Base Color"].default_value = (0.05, 0.055, 0.065, 1); b.inputs["Roughness"].default_value = 0.6
        # subtle grid so motion over the floor is readable
        tc = nt.nodes.new("ShaderNodeTexCoord"); ch = nt.nodes.new("ShaderNodeTexChecker")
        ch.inputs["Scale"].default_value = 24
        ch.inputs["Color1"].default_value = (0.045, 0.05, 0.06, 1); ch.inputs["Color2"].default_value = (0.06, 0.066, 0.078, 1)
        nt.links.new(tc.outputs["Object"], ch.inputs["Vector"]); nt.links.new(ch.outputs["Color"], b.inputs["Base Color"])
        f.data.materials.append(m)
    return sc


def camera(loc, target, lens=50, ortho=None):
    sc = bpy.context.scene
    cd = bpy.data.cameras.new("cam"); cd.lens = lens
    if ortho:
        cd.type = "ORTHO"; cd.ortho_scale = ortho
    cam = bpy.data.objects.new("cam", cd); sc.collection.objects.link(cam)
    cam.location = loc
    cam.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    sc.camera = cam
    return cam


def orbit_rig(target, radius, height, lens, frames):
    """Camera parented to a rotating empty -> turntable."""
    sc = bpy.context.scene
    piv = bpy.data.objects.new("pivot", None); sc.collection.objects.link(piv); piv.location = target
    cam = camera((target[0] + radius, target[1] - radius * 0.0, target[2] + height), target, lens)
    cam.location = (radius, 0, height)
    cam.rotation_euler = (Vector((0, 0, 0)) - Vector((radius, 0, height))).to_track_quat("-Z", "Y").to_euler()
    cam.parent = piv
    sc.frame_start, sc.frame_end = 1, frames
    piv.rotation_euler = (0, 0, 0); piv.keyframe_insert("rotation_euler", frame=1)
    piv.rotation_euler = (0, 0, 2 * math.pi); piv.keyframe_insert("rotation_euler", frame=frames + 1)
    for fc in _fcurves(piv):
        for k in fc.keyframe_points:
            k.interpolation = "LINEAR"
    return cam, piv


def _fcurves(obj):
    ad = obj.animation_data
    if ad is None or ad.action is None:
        return []
    act = ad.action
    if hasattr(act, "fcurves"):
        return list(act.fcurves)
    # Blender 4.4+/5.x layered actions
    out = []
    for layer in act.layers:
        for strip in layer.strips:
            for cb in strip.channelbags:
                out.extend(cb.fcurves)
    return out


def render_frames(outdir, prefix="f"):
    sc = bpy.context.scene
    os.makedirs(outdir, exist_ok=True)
    sc.render.image_settings.file_format = "PNG"
    for fr in range(sc.frame_start, sc.frame_end + 1):
        sc.frame_set(fr)
        sc.render.filepath = os.path.join(outdir, "%s%04d.png" % (prefix, fr))
        bpy.ops.render.render(write_still=True)


def render_still(path):
    sc = bpy.context.scene
    sc.render.image_settings.file_format = "PNG"
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)
