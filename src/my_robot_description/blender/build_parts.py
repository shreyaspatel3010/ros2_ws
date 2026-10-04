"""Procedurally model, texture-bake and export the HRUH humanoid leg / pelvis /
waist / chest meshes.

Run (from the package root):
    blender -b --factory-startup -P blender/build_parts.py -- meshes

Every part is modelled in its URDF link frame (x forward, y left, z up, metres),
textured with procedural materials, then baked (Cycles) into albedo (with
ambient occlusion), roughness/metallic and emission maps, and exported twice:
  * <part>.obj + .mtl + .png  -> used by the URDF (RViz / Gazebo)
  * glb/<part>.glb            -> self-contained PBR glTF (Blender, web viewers, Unity, Isaac Sim ...)  Right-side parts are mirrored copies of the left ones and
reuse the left textures.  Joint locations come from KIN and must match
urdf/legs.xacro and urdf/torso.xacro.
"""
import math, os, sys
import bpy, bmesh
import numpy as np
from mathutils import Vector, Matrix

ARGV = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.abspath(ARGV[0]) if ARGV else os.path.join(PKG, "meshes")
ONLY = set(ARGV[1].split(",")) if len(ARGV) > 1 else None
TEX = 1024

# Kinematic layout (keep in sync with urdf/legs.xacro + urdf/torso.xacro)
KIN = dict(
    hip_y=0.105,       # lateral offset of hip yaw axis from pelvis centre (thigh clearance)
    hip_yaw_z=-0.03,   # hip yaw joint below pelvis frame
    hip_roll_z=-0.06,  # hip centre (roll/pitch axes) below hip yaw joint
    thigh=0.34,        # hip centre -> knee
    shin=0.34,         # knee -> ankle centre
    ankle_h=0.065,     # ankle centre -> sole
    toe_x=0.125,       # toe hinge in front of ankle
    toe_z=-0.045,      # toe hinge below ankle
    neck_pitch_z=0.07, # head nod pivot above the chest_to_neck joint
    eye_y=0.032,       # half of the stereo baseline (human IPD ~64 mm)
    eye_z=0.155,       # eye height above the chest_to_neck joint
    waist_z=0.09,      # rigid lumbar block (torso_link) above pelvis frame
    chest_z=0.04,      # chest_hruh origin above torso_link
)
# Hip actuator packaging. The three hip axes still meet at the hip centre (the walker's
# closed-form leg IK relies on it), but each motor body sits *along its own axis* where it
# physically fits, as in real humanoid hips:
#   yaw   motor: inside the pelvis, above the hip (axis z)
#   roll  motor: behind the hip centre (axis x) - the front stays open for hip flexion
#   pitch motor: outside the thigh (axis y)
HIP = dict(
    yaw_r=0.050, yaw_h=0.060,              # 150 N m class pancake actuator
    yaw_z0=0.012,                          # its underside, high in the pelvis: the pitch
                                           # actuator swings up under it in abduction
    roll_x=-0.130, roll_r=0.050, roll_h=0.055,    # 200 N m class, centre behind the hip
    pitch_y=0.080, pitch_r=0.056, pitch_h=0.050,  # 250 N m class, centre outside the thigh
)

# ----------------------------------------------------------------------------
# Materials: procedural, evaluated in object (= link) space, then baked
# ----------------------------------------------------------------------------
PALETTE = {
    "white":    dict(col=(0.78, 0.79, 0.81), rough=0.38, metal=0.0),
    "dark":     dict(col=(0.030, 0.033, 0.037), rough=0.42, metal=0.2),
    "gunmetal": dict(col=(0.13, 0.14, 0.16), rough=0.30, metal=0.85),
    "metal":    dict(col=(0.52, 0.54, 0.57), rough=0.22, metal=1.0),
    "rubber":   dict(col=(0.020, 0.020, 0.023), rough=0.92, metal=0.0),
    "accent":   dict(col=(0.04, 0.26, 0.85), rough=0.30, metal=0.1),
    "glow":     dict(col=(0.20, 0.80, 1.00), rough=0.20, metal=0.0, glow=1.0),
    "groove":   dict(col=(0.10, 0.105, 0.115), rough=0.6, metal=0.0),
    "lens":     dict(col=(0.006, 0.012, 0.03), rough=0.05, metal=0.0),
}
SOCKETS = {}   # material name -> {"col","rm","glow"} output sockets


def _mix(nt, fac, a, b):
    m = nt.nodes.new("ShaderNodeMix"); m.data_type = "RGBA"; m.blend_type = "MIX"; m.clamp_factor = True
    ins = {s.identifier: s for s in m.inputs}
    nt.links.new(fac, ins["Factor_Float"])
    for sock, key in ((a, "A_Color"), (b, "B_Color")):
        if isinstance(sock, tuple):
            ins[key].default_value = (*sock, 1.0)
        else:
            nt.links.new(sock, ins[key])
    return [s for s in m.outputs if s.identifier == "Result_Color"][0]


def _math(nt, op, *args, clamp=False):
    n = nt.nodes.new("ShaderNodeMath"); n.operation = op; n.use_clamp = clamp
    for i, v in enumerate(args):
        if v is None:
            continue
        if isinstance(v, (int, float)):
            n.inputs[i].default_value = v
        else:
            nt.links.new(v, n.inputs[i])
    return n.outputs[0]


def _rgb(nt, c):
    n = nt.nodes.new("ShaderNodeRGB"); n.outputs[0].default_value = (*c, 1.0)
    return n.outputs[0]


def make_material(part, kind, bands=()):
    """bands: (axis, centre, half_width, colour_kind, [(axis, lo, hi), ...])"""
    name = "%s_%s" % (part, kind)
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    P = PALETTE[kind]
    m = bpy.data.materials.new(name); m.use_nodes = True
    nt = m.node_tree; N = nt.nodes; L = nt.links
    N.clear()
    out = N.new("ShaderNodeOutputMaterial")
    emis = N.new("ShaderNodeEmission"); L.new(emis.outputs[0], out.inputs["Surface"])
    tc = N.new("ShaderNodeTexCoord")
    sep = N.new("ShaderNodeSeparateXYZ"); L.new(tc.outputs["Object"], sep.inputs[0])
    axes = {"x": sep.outputs[0], "y": sep.outputs[1], "z": sep.outputs[2]}

    # --- base colour with surface variation
    noise = N.new("ShaderNodeTexNoise"); noise.inputs["Scale"].default_value = 35.0
    noise.inputs["Detail"].default_value = 6.0
    L.new(tc.outputs["Object"], noise.inputs["Vector"])
    var = _math(nt, "MULTIPLY_ADD", noise.outputs["Fac"], 0.12, 0.94)       # 0.94..1.06
    if kind in ("dark",):                                                     # carbon weave
        w1 = N.new("ShaderNodeTexWave"); w1.wave_type = "BANDS"; w1.bands_direction = "X"
        w2 = N.new("ShaderNodeTexWave"); w2.wave_type = "BANDS"; w2.bands_direction = "Z"
        ck = N.new("ShaderNodeTexChecker")
        for n_, s in ((w1, 260.0), (w2, 260.0), (ck, 130.0)):
            n_.inputs["Scale"].default_value = s
            L.new(tc.outputs["Object"], n_.inputs["Vector"])
        weave = _mix(nt, ck.outputs["Fac"], w1.outputs["Color"], w2.outputs["Color"])
        bw = N.new("ShaderNodeRGBToBW"); L.new(weave, bw.inputs[0])
        var = _math(nt, "MULTIPLY", var, _math(nt, "MULTIPLY_ADD", bw.outputs[0], 0.9, 0.6))
    elif kind in ("metal", "gunmetal"):                                       # brushed
        mp = N.new("ShaderNodeMapping"); mp.inputs["Scale"].default_value = (2.0, 2.0, 400.0)
        L.new(tc.outputs["Object"], mp.inputs["Vector"])
        bn = N.new("ShaderNodeTexNoise"); bn.inputs["Scale"].default_value = 3.0; bn.inputs["Detail"].default_value = 10
        L.new(mp.outputs[0], bn.inputs["Vector"])
        var = _math(nt, "MULTIPLY", var, _math(nt, "MULTIPLY_ADD", bn.outputs["Fac"], 0.35, 0.82))
    elif kind == "rubber":                                                    # sole tread
        tw = N.new("ShaderNodeTexWave"); tw.wave_type = "BANDS"; tw.bands_direction = "X"
        tw.inputs["Scale"].default_value = 70.0; L.new(tc.outputs["Object"], tw.inputs["Vector"])
        var = _math(nt, "MULTIPLY", var, _math(nt, "MULTIPLY_ADD", tw.outputs["Fac"], 1.2, 0.5))
    vm = N.new("ShaderNodeVectorMath"); vm.operation = "SCALE"
    L.new(_rgb(nt, P["col"]), vm.inputs[0]); L.new(var, vm.inputs["Scale"])
    col = vm.outputs[0]
    rm = _rgb(nt, (1.0, P["rough"], P["metal"]))   # glTF layout: G = roughness, B = metallic
    glow = _rgb(nt, tuple(c * P.get("glow", 0.0) for c in P["col"]))

    # --- painted bands / panel grooves (object-space)
    for (ax, c, hw, bkind, limits) in bands:
        if isinstance(ax, tuple):           # plane with normal ax: dot(p, n) = c
            dn = N.new("ShaderNodeVectorMath"); dn.operation = "DOT_PRODUCT"
            L.new(tc.outputs["Object"], dn.inputs[0]); dn.inputs[1].default_value = Vector(ax).normalized()
            coord = dn.outputs["Value"]
        else:
            coord = axes[ax]
        d = _math(nt, "ABSOLUTE", _math(nt, "SUBTRACT", coord, c))
        mr = N.new("ShaderNodeMapRange"); mr.interpolation_type = "SMOOTHSTEP"
        L.new(d, mr.inputs["Value"])
        mr.inputs["From Min"].default_value = hw; mr.inputs["From Max"].default_value = hw * 0.55
        mask = mr.outputs["Result"]
        for (lax, lo, hi) in limits:
            inside = _math(nt, "MULTIPLY", _math(nt, "GREATER_THAN", axes[lax], lo), _math(nt, "LESS_THAN", axes[lax], hi))
            mask = _math(nt, "MULTIPLY", mask, inside)
        B = PALETTE[bkind]
        bcol = B["col"]
        if bkind == "dark":     # keep the band textured, just darker
            dv = N.new("ShaderNodeVectorMath"); dv.operation = "SCALE"
            L.new(_rgb(nt, bcol), dv.inputs[0]); L.new(var, dv.inputs["Scale"]); bcol = dv.outputs[0]
        col = _mix(nt, mask, col, bcol)
        rm = _mix(nt, mask, rm, (1.0, B["rough"], B["metal"]))
        glow = _mix(nt, mask, glow, tuple(cc * B.get("glow", 0.0) for cc in B["col"]))
    L.new(col, emis.inputs["Color"])
    SOCKETS[name] = dict(col=col, rm=rm, glow=glow, emis=emis)
    return m


# ----------------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------------
def _link(obj):
    bpy.context.scene.collection.objects.link(obj)
    return obj


def _ring(sec, n):
    """sec: (w, cu, cv, ru+, ru-, rv+, rv-, p) -> list of (u, v, w)"""
    w, cu, cv, rup, run, rvp, rvn, p = sec
    e = 2.0 / p
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        c, s = math.cos(t), math.sin(t)
        ru = rup if c >= 0 else run
        rv = rvp if s >= 0 else rvn
        pts.append((cu + ru * math.copysign(abs(c) ** e, c), cv + rv * math.copysign(abs(s) ** e, s), w))
    return pts


def loft(name, axis, secs, mat, n=48, subsurf=2):
    """Loft superellipse sections along an axis.
    axis 'z': u=x, v=y ; axis 'x': u=y, v=z ; axis 'y': u=x, v=z."""
    mp = {"z": lambda u, v, w: (u, v, w), "x": lambda u, v, w: (w, u, v), "y": lambda u, v, w: (u, w, v)}[axis]
    bm = bmesh.new()
    rings = [[bm.verts.new(mp(*p)) for p in _ring(s, n)] for s in secs]
    for a, b in zip(rings, rings[1:]):
        for i in range(n):
            bm.faces.new((a[i], a[(i + 1) % n], b[(i + 1) % n], b[i]))
    bm.faces.new(list(reversed(rings[0]))); bm.faces.new(rings[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
    o = _link(bpy.data.objects.new(name, me))
    for p in me.polygons:
        p.use_smooth = True
    if subsurf:
        md = o.modifiers.new("ss", "SUBSURF"); md.levels = md.render_levels = subsurf
    o.data.materials.append(mat)
    return o


def _orient(o, center, axis):
    axis = Vector(axis).normalized()
    o.rotation_mode = "QUATERNION"
    o.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(axis)
    o.location = center


def cyl(name, center, axis, r, length, mat, bevel=0.002, verts=48):
    bpy.ops.mesh.primitive_cylinder_add(radius=r, depth=length, vertices=verts)
    o = bpy.context.object; o.name = name
    _orient(o, center, axis)
    if bevel:
        b = o.modifiers.new("bv", "BEVEL"); b.width = bevel; b.segments = 3; b.limit_method = "ANGLE"
    o.data.materials.append(mat)
    return o


def seg(name, p0, p1, r, mat, bevel=0.0015):
    p0, p1 = Vector(p0), Vector(p1)
    return cyl(name, (p0 + p1) / 2, p1 - p0, r, (p1 - p0).length, mat, bevel, verts=32)


def rbox(name, center, size, mat, bevel=0.004, rot=(0, 0, 0)):
    bpy.ops.mesh.primitive_cube_add(size=1)
    o = bpy.context.object; o.name = name
    o.scale = size; o.location = center; o.rotation_euler = rot
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel:
        b = o.modifiers.new("bv", "BEVEL"); b.width = bevel; b.segments = 4; b.limit_method = "ANGLE"
    o.data.materials.append(mat)
    return o


def ellipsoid(name, center, radii, mat, rot=(0, 0, 0)):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1, segments=48, ring_count=24)
    o = bpy.context.object; o.name = name
    o.scale = radii; o.location = center; o.rotation_euler = rot
    bpy.ops.object.shade_smooth()
    o.data.materials.append(mat)
    return o


def torus(name, center, axis, R, r, mat):
    bpy.ops.mesh.primitive_torus_add(major_radius=R, minor_radius=r, major_segments=64, minor_segments=16)
    o = bpy.context.object; o.name = name
    _orient(o, center, axis)
    bpy.ops.object.shade_smooth()
    o.data.materials.append(mat)
    return o


def text_plate(name, text, center, right, up, size, mat):
    bpy.ops.object.text_add()
    o = bpy.context.object; o.name = name
    o.data.body = text; o.data.size = size; o.data.extrude = 0.0008
    o.data.align_x = "CENTER"; o.data.align_y = "CENTER"
    right, up = Vector(right).normalized(), Vector(up).normalized()
    normal = right.cross(up)
    R = Matrix((right, up, normal)).transposed().to_4x4()
    o.matrix_world = Matrix.Translation(center) @ R
    bpy.ops.object.convert(target="MESH")
    o.data.materials.append(mat)
    return o


def surface_point(obj, origin, direction):
    """Ray-cast against an (evaluated) object; returns hit location or None."""
    dg = bpy.context.evaluated_depsgraph_get()
    ok, loc, nor, idx = obj.evaluated_get(dg).ray_cast(Vector(origin), Vector(direction))
    return (loc, nor) if ok else (None, None)


def finish(name, objs):
    """Apply modifiers, join into one object with origin = link frame."""
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.object.convert(target="MESH")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.object.join()
    o = bpy.context.object; o.name = name; o.data.name = name
    # boolean cuts can leave an empty material slot: give those faces the main material
    empty = {i for i, m in enumerate(o.data.materials) if m is None}
    for p in o.data.polygons:
        if p.material_index in empty:
            p.material_index = 0
    bpy.ops.object.material_slot_remove_unused()
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bpy.ops.object.shade_smooth_by_angle(angle=math.radians(40))
    return o


# ----------------------------------------------------------------------------
# Baking + export
# ----------------------------------------------------------------------------
def _np(img):
    a = np.empty(img.size[0] * img.size[1] * 4, dtype=np.float32)
    img.pixels.foreach_get(a)
    return a.reshape(img.size[1], img.size[0], 4)


def _srgb(x):
    x = np.clip(x, 0, 1)
    return np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1 / 2.4) - 0.055)


def _save_png(arr, path, srgb):
    h, w = arr.shape[:2]
    img = bpy.data.images.new(os.path.basename(path), w, h, alpha=False)
    img.colorspace_settings.name = "sRGB" if srgb else "Non-Color"
    out = arr.copy()
    out[..., :3] = _srgb(arr[..., :3]) if srgb else np.clip(arr[..., :3], 0, 1)
    out[..., 3] = 1.0
    img.pixels.foreach_set(out.ravel())
    img.filepath_raw = path; img.file_format = "PNG"
    img.save()
    return img


def bake(obj, tex_name, outdir, res=TEX):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"; sc.cycles.device = "CPU"
    for o in sc.objects:
        o.hide_render = o is not obj
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True); bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode="EDIT"); bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.smart_project(angle_limit=math.radians(55), island_margin=0.004)
    bpy.ops.object.mode_set(mode="OBJECT")

    imgs = {}
    for k in ("col", "ao", "glow", "rm"):
        im = bpy.data.images.new("%s_%s" % (tex_name, k), res, res, alpha=False, float_buffer=True)
        im.colorspace_settings.name = "Linear Rec.709" if k in ("col", "glow") else "Non-Color"
        imgs[k] = im
    tex_nodes = []
    for mat in obj.data.materials:
        n = mat.node_tree.nodes.new("ShaderNodeTexImage")
        mat.node_tree.nodes.active = n
        tex_nodes.append((mat, n))
    sc.render.bake.margin = 6
    for k in ("col", "glow", "rm", "ao"):
        for mat, n in tex_nodes:
            n.image = imgs[k]
            if k != "ao":
                s = SOCKETS[mat.name]
                mat.node_tree.links.new(s[k], s["emis"].inputs["Color"])
        sc.cycles.samples = 64 if k == "ao" else 4
        bpy.ops.object.bake(type="AO" if k == "ao" else "EMIT", margin=6, use_clear=True)
    col, ao, glow, rm = (_np(imgs[k]) for k in ("col", "ao", "glow", "rm"))
    aof = 0.38 + 0.62 * ao[..., :1]
    gmask = np.clip(glow[..., :3].max(axis=2, keepdims=True) * 1.5, 0, 1)
    alb = col.copy()
    alb[..., :3] = col[..., :3] * aof * (1 - gmask) + glow[..., :3] * gmask
    _save_png(alb, os.path.join(outdir, tex_name + "_albedo.png"), True)
    _save_png(glow, os.path.join(outdir, tex_name + "_glow.png"), True)
    _save_png(rm, os.path.join(outdir, tex_name + "_rm.png"), False)

    # Final single PBR material referencing the baked maps (OBJ keeps the albedo,
    # GLB embeds albedo + roughness/metallic + emission)
    fm = bpy.data.materials.new(tex_name + "_mat"); fm.use_nodes = True
    nt = fm.node_tree; bsdf = nt.nodes["Principled BSDF"]

    def tex(suffix, non_color=False):
        n = nt.nodes.new("ShaderNodeTexImage")
        n.image = bpy.data.images.load(os.path.join(outdir, tex_name + suffix), check_existing=True)
        if non_color:
            n.image.colorspace_settings.name = "Non-Color"
        return n
    nt.links.new(tex("_albedo.png").outputs["Color"], bsdf.inputs["Base Color"])
    sep = nt.nodes.new("ShaderNodeSeparateColor")
    nt.links.new(tex("_rm.png", True).outputs["Color"], sep.inputs["Color"])
    nt.links.new(sep.outputs["Green"], bsdf.inputs["Roughness"])
    nt.links.new(sep.outputs["Blue"], bsdf.inputs["Metallic"])
    nt.links.new(tex("_glow.png").outputs["Color"], bsdf.inputs["Emission Color"])
    bsdf.inputs["Emission Strength"].default_value = 1.0
    obj.data.materials.clear(); obj.data.materials.append(fm)
    for p in obj.data.polygons:
        p.material_index = 0
    for o in sc.objects:
        o.hide_render = False
    return obj


def export_obj(obj, path):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True); bpy.context.view_layer.objects.active = obj
    bpy.ops.wm.obj_export(filepath=path, export_selected_objects=True, forward_axis="Y", up_axis="Z",
                          export_materials=True, path_mode="STRIP", export_uv=True, export_normals=True,
                          apply_modifiers=True, export_triangulated_mesh=True, export_pbr_extensions=False)
    # tone down the default (very glossy) specular for RViz / Gazebo's Phong-ish shading
    mtl = os.path.splitext(path)[0] + ".mtl"
    lines = []
    for ln in open(mtl).read().splitlines():
        if ln.startswith("Ns "):
            ln = "Ns 40.000000"
        elif ln.startswith("Ks "):
            ln = "Ks 0.150000 0.150000 0.150000"
        elif ln.startswith("Ka "):
            ln = "Ka 0.800000 0.800000 0.800000"
        lines.append(ln)
    open(mtl, "w").write("\n".join(lines) + "\n")
    ntri = sum(len(p.vertices) - 2 for p in obj.data.polygons)
    print("EXPORT %-28s %6d tris" % (os.path.basename(path), ntri))


def export_glb(obj, path):
    """Textured, self-contained glTF binary (Y-up per the glTF spec)."""
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True); bpy.context.view_layer.objects.active = obj
    bpy.ops.export_scene.gltf(filepath=path, export_format="GLB", use_selection=True, export_yup=True,
                              export_apply=True, export_materials="EXPORT", export_image_format="AUTO")
    print("EXPORT %-28s %6.2f MB" % (os.path.basename(path), os.path.getsize(path) / 1e6))


def export_part(obj, outdir, name):
    export_obj(obj, os.path.join(outdir, name + ".obj"))
    export_glb(obj, os.path.join(outdir, "glb", name + ".glb"))


def mirror_y(obj, name):
    o = obj.copy(); o.data = obj.data.copy(); o.name = name; o.data.name = name
    bpy.context.scene.collection.objects.link(o)
    bpy.ops.object.select_all(action="DESELECT")
    o.select_set(True); bpy.context.view_layer.objects.active = o
    o.scale = (1, -1, 1)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return o


# ----------------------------------------------------------------------------
# Parts (left side, link frames)
# ----------------------------------------------------------------------------
def part_pelvis():
    P = "pelvis"
    W = make_material(P, "white", [
        ("z", 0.040, 0.011, "dark", []),                       # belt
        ("z", 0.040, 0.0016, "glow", [("x", 0.02, 1)]),        # light strip on belt (front)
        ("z", 0.002, 0.0018, "groove", []),
        ("y", 0.0, 0.0018, "groove", [("z", -1, 0.03)]),
    ])
    D, M, G = make_material(P, "dark"), make_material(P, "metal"), make_material(P, "glow")
    GM = make_material(P, "gunmetal")
    hy, zy = KIN["hip_y"], KIN["hip_yaw_z"]
    # shell: high floor (z >= 0.012) and narrow sides, so the thighs clear it in flexion and
    # the pitch actuators can swing up beside / under it in abduction
    z0 = HIP["yaw_z0"]
    o = [loft("pelvis_shell", "z", [
        (z0, 0.0, 0, 0.066, 0.074, 0.145, 0.145, 2.6),
        (0.030, 0.0, 0, 0.082, 0.092, 0.156, 0.156, 3.0),
        (0.048, 0.0, 0, 0.088, 0.098, 0.158, 0.158, 3.2),
        (0.064, 0.0, 0, 0.078, 0.088, 0.138, 0.138, 3.0),
        (0.078, 0.0, 0, 0.062, 0.070, 0.100, 0.100, 2.6),
    ], W)]
    # narrow crotch spar between the legs
    o.append(rbox("crotch", (0.0, 0, -0.012), (0.07, 0.044, 0.05), D, 0.012))
    for s in (1, -1):
        # hip yaw actuator high in the pelvis; a hub on the hip-yaw link reaches up to it
        o.append(cyl("yaw_motor", (0, s * hy, z0 + 0.5 * HIP["yaw_h"]), (0, 0, 1), HIP["yaw_r"], HIP["yaw_h"], GM))
        o.append(cyl("yaw_ring", (0, s * hy, z0 + 0.003), (0, 0, 1), HIP["yaw_r"] + 0.003, 0.006, D))
    o.append(cyl("waist_socket", (0, 0, 0.078), (0, 0, 1), 0.07, 0.024, GM))
    o.append(rbox("battery", (-0.093, 0, 0.025), (0.045, 0.17, 0.07), D, 0.014))
    for i in range(5):
        o.append(rbox("vent", (-0.1155, 0, 0.000 + i * 0.012), (0.004, 0.13, 0.0045), M, 0.0015))
    o.append(cyl("status_ring", (0.098, 0, 0.02), (1, 0, 0), 0.024, 0.012, D))
    o.append(cyl("status_led", (0.102, 0, 0.02), (1, 0, 0), 0.016, 0.012, G))
    return finish(P, o)


def part_waist():
    """Rigid lumbar block (torso_link frame) joining pelvis and chest: no joints."""
    P = "waist"
    W = make_material(P, "white", [("z", 0.0, 0.0018, "groove", [])])
    D, M, GM = make_material(P, "dark"), make_material(P, "metal"), make_material(P, "gunmetal")
    o = [loft("lumbar", "z", [
        (-0.030, 0.0, 0, 0.072, 0.078, 0.098, 0.098, 2.6),
        (0.000, 0.0, 0, 0.068, 0.074, 0.094, 0.094, 2.6),
        (0.040, 0.0, 0, 0.072, 0.076, 0.096, 0.096, 2.6),
    ], W)]
    for s in (1, -1):
        o.append(rbox("side_panel", (0, s * 0.093, 0.004), (0.08, 0.008, 0.05), D, 0.004))
    o.append(rbox("rear_spine", (-0.074, 0, 0.004), (0.012, 0.05, 0.062), GM, 0.004))
    o.append(cyl("front_light_bezel", (0.07, 0, 0.004), (1, 0, 0), 0.016, 0.01, M))
    return finish(P, o)


def part_hip_yaw():
    """Yaw output (top) + rear arm carrying the hip roll actuator behind the hip centre."""
    P = "hip_yaw"
    W = make_material(P, "white", [("x", HIP["roll_x"], 0.0015, "accent", [])])
    D, M, GM = make_material(P, "dark"), make_material(P, "metal"), make_material(P, "gunmetal")
    zr, rx = KIN["hip_roll_z"], HIP["roll_x"]
    hub_top = HIP["yaw_z0"] - KIN["hip_yaw_z"]          # yaw actuator underside, in this frame
    arm_lo, arm_hi = 0.004, 0.020                       # rear arm clears the thigh in flexion
    drop_lo = zr + HIP["roll_r"] - 0.010
    o = [cyl("yaw_hub", (0, 0, 0.5 * hub_top), (0, 0, 1), 0.032, hub_top, M),
         rbox("rear_arm", (rx * 0.5 - 0.01, 0, 0.5 * (arm_lo + arm_hi)), (abs(rx) + 0.02, 0.05, arm_hi - arm_lo), D, 0.005),
         rbox("drop_plate", (rx, 0, 0.5 * (arm_hi + drop_lo)), (0.04, 0.05, arm_hi - drop_lo), D, 0.004),
         cyl("roll_motor", (rx, 0, zr), (1, 0, 0), HIP["roll_r"], HIP["roll_h"], GM),
         cyl("roll_shroud", (rx - 0.004, 0, zr), (1, 0, 0), HIP["roll_r"] + 0.003, HIP["roll_h"] * 0.6, W)]
    return finish(P, o)


def part_hip_roll():
    """Roll output (behind) + bracket round the back to the pitch actuator outside the thigh."""
    P = "hip_roll"
    W = make_material(P, "white", [("y", HIP["pitch_y"], 0.0016, "glow", [])])
    D, M, GM = make_material(P, "dark"), make_material(P, "metal"), make_material(P, "gunmetal")
    rx, py = HIP["roll_x"], HIP["pitch_y"]
    front = rx + 0.5 * HIP["roll_h"]          # roll actuator output face
    by = py + 0.5 * HIP["pitch_h"]            # outer face of the pitch actuator
    o = [cyl("roll_output", (front + 0.005, 0, 0), (1, 0, 0), 0.044, 0.010, M),
         rbox("rear_bracket", (front + 0.012, 0.5 * by, 0), (0.016, by + 0.02, 0.07), D, 0.006),
         rbox("side_bracket", (0.5 * (front + 0.012), by + 0.004, 0), (abs(front) + 0.02, 0.012, 0.07), D, 0.006),
         cyl("pitch_motor", (0, py, 0), (0, 1, 0), HIP["pitch_r"], HIP["pitch_h"], W),
         cyl("pitch_cap", (0, by + 0.012, 0), (0, 1, 0), 0.040, 0.006, M),
         cyl("pitch_hub", (0, by + 0.016, 0), (0, 1, 0), 0.016, 0.006, GM)]
    return finish(P, o)


def part_thigh():
    P = "thigh"
    L_ = KIN["thigh"]
    W = make_material(P, "white", [
        ("z", -0.035, 0.0018, "groove", []),
        ("z", -0.255, 0.0018, "groove", []),
        ("y", -0.105, 0.062, "dark", [("z", -0.27, -0.03), ("x", -0.04, 0.035)]),  # inner-thigh carbon panel
        ("x", -0.004, 0.0016, "glow", [("y", 0.03, 1), ("z", -0.23, -0.06)]),  # outer light line
        ("x", 0.0, 0.0018, "groove", [("y", 0.03, 1), ("z", -0.255, -0.035)]),
    ])
    D, M, GM = make_material(P, "dark"), make_material(P, "metal"), make_material(P, "gunmetal")
    # slimmer than before at the hip: the pitch actuator sits outside (y > 0.055)
    o = [loft("thigh_shell", "z", [
        (0.022, 0.000, 0, 0.036, 0.040, 0.036, 0.036, 2.3),
        (0.000, 0.002, 0, 0.048, 0.050, 0.044, 0.044, 2.4),
        (-0.060, 0.008, 0, 0.062, 0.060, 0.052, 0.052, 2.4),
        (-0.140, 0.010, 0, 0.062, 0.058, 0.052, 0.052, 2.4),
        (-0.220, 0.005, 0, 0.054, 0.050, 0.048, 0.048, 2.4),
        (-0.280, 0.000, 0, 0.046, 0.044, 0.045, 0.045, 2.4),
        (-0.310, 0.000, 0, 0.036, 0.040, 0.040, 0.040, 2.4),
    ], W)]
    o.append(cyl("knee_motor", (0, 0, -L_), (0, 1, 0), 0.043, 0.088, GM))
    for s in (1, -1):
        o.append(cyl("knee_cap", (0, s * 0.046, -L_), (0, 1, 0), 0.044, 0.008, M))
    o.append(cyl("hip_output_plate", (0, HIP["pitch_y"] - 0.5 * HIP["pitch_h"] - 0.007, 0), (0, 1, 0), 0.036, 0.010, D))
    return finish(P, o)


def part_shin():
    P = "shin"
    L_ = KIN["shin"]
    W = make_material(P, "white", [
        ("z", -0.205, 0.0018, "groove", [("x", 0.0, 1)]),
        ("y", 0.0, 0.0035, "glow", [("x", 0.02, 1), ("z", -0.19, -0.07)]),  # tibia light
        ("y", -0.04, 0.01, "dark", [("z", -0.25, -0.05)]),
        ("y", 0.04, 0.01, "dark", [("z", -0.25, -0.05), ("x", -1, -0.02)]),
    ])
    D, M, GM = make_material(P, "dark"), make_material(P, "metal"), make_material(P, "gunmetal")
    o = [loft("shin_shell", "z", [
        (-0.040, -0.004, 0, 0.036, 0.040, 0.046, 0.046, 2.4),
        (-0.065, -0.006, 0, 0.040, 0.056, 0.050, 0.050, 2.4),
        (-0.115, -0.008, 0, 0.041, 0.064, 0.051, 0.051, 2.4),
        (-0.175, -0.006, 0, 0.037, 0.054, 0.045, 0.045, 2.4),
        (-0.245, -0.002, 0, 0.031, 0.038, 0.035, 0.035, 2.4),
        (-0.290, 0.000, 0, 0.027, 0.029, 0.029, 0.029, 2.4),
        (-0.310, 0.000, 0, 0.021, 0.023, 0.024, 0.024, 2.4),
    ], W)]
    o.append(rbox("knee_guard", (0.04, 0, -0.008), (0.014, 0.05, 0.052), GM, 0.006, rot=(0, -0.2, 0)))
    o.append(rbox("knee_guard_light", (0.0475, 0, -0.008), (0.002, 0.03, 0.004), make_material(P, "glow"), 0.0008, rot=(0, -0.2, 0)))
    for s in (1, -1):
        o.append(cyl("knee_fork_boss", (0, s * 0.056, 0), (0, 1, 0), 0.034, 0.01, D))
        o.append(rbox("knee_fork", (-0.004, s * 0.056, -0.036), (0.058, 0.01, 0.07), D, 0.004))
        o.append(rbox("ankle_fork", (0, s * 0.033, -L_ + 0.026), (0.036, 0.008, 0.05), D, 0.003))
        o.append(cyl("ankle_fork_boss", (0, s * 0.033, -L_), (0, 1, 0), 0.02, 0.008, D))
        o.append(cyl("ankle_pin", (0, s * 0.039, -L_), (0, 1, 0), 0.01, 0.005, M))
    o.append(seg("achilles_rod", (-0.052, 0, -0.16), (-0.03, 0, -0.30), 0.0065, M))
    o.append(seg("achilles_rod2", (-0.05, 0, -0.165), (-0.054, 0, -0.12), 0.011, GM))
    return finish(P, o)


def part_ankle():
    P = "ankle"
    D, M, GM = make_material(P, "dark"), make_material(P, "metal"), make_material(P, "gunmetal")
    o = [rbox("cross", (0, 0, 0), (0.032, 0.052, 0.032), GM, 0.007),
         cyl("roll_pin", (0, 0, 0), (1, 0, 0), 0.011, 0.07, M),
         cyl("pitch_pin", (0, 0, 0), (0, 1, 0), 0.011, 0.074, M)]
    return finish(P, o)


FOOT_SECS = [  # (x, y-centre, z-centre, ry, ry, rz_top, rz_bot, p) for loft axis 'x'
    (-0.072, 0.000, -0.046, 0.024, 0.024, 0.014, 0.019, 2.4),
    (-0.058, 0.000, -0.038, 0.035, 0.035, 0.028, 0.027, 2.8),
    (-0.020, 0.000, -0.028, 0.041, 0.041, 0.042, 0.037, 3.0),
    (0.030, -0.003, -0.036, 0.046, 0.046, 0.030, 0.029, 3.2),
    (0.080, -0.004, -0.044, 0.049, 0.049, 0.019, 0.021, 3.4),
    (0.116, -0.004, -0.047, 0.049, 0.049, 0.012, 0.018, 3.4),
    (0.123, -0.004, -0.047, 0.045, 0.045, 0.008, 0.014, 3.0),
]


def part_foot():
    P = "foot"
    W = make_material(P, "white", [
        ("z", -0.046, 0.0016, "accent", [("x", -0.06, 0.11)]),
        ("z", -0.058, 0.007, "dark", []),
    ])
    D, M, R = make_material(P, "dark"), make_material(P, "metal"), make_material(P, "rubber")
    o = [loft("foot_body", "x", [(x, cy, cz, ry, ry2, rzt, rzb, p) for (x, cy, cz, ry, ry2, rzt, rzb, p) in FOOT_SECS], W)]
    sole = []
    for (x, cy, cz, ry, _, rzt, rzb, p) in FOOT_SECS:
        sole.append((x - (0.004 if x < 0 else -0.003 if x > 0.1 else 0), cy, -KIN["ankle_h"] + 0.005, ry + 0.003, ry + 0.003, 0.005, 0.005, 4.0))
    o.append(loft("sole", "x", sole, R, subsurf=1))
    for s in (1, -1):
        o.append(rbox("ankle_bracket", (s * 0.04, 0, -0.024), (0.008, 0.036, 0.046), D, 0.003))
        o.append(cyl("ankle_boss", (s * 0.04, 0, 0), (1, 0, 0), 0.019, 0.008, D))
    o.append(cyl("toe_hinge", (KIN["toe_x"], 0, KIN["toe_z"]), (0, 1, 0), 0.009, 0.085, M))
    return finish(P, o)


def part_toe():
    P = "toe"
    zs = -KIN["ankle_h"] - KIN["toe_z"]          # sole height in toe frame (-0.02)
    W = make_material(P, "white", [("z", zs + 0.007, 0.007, "dark", [])])
    R, GM = make_material(P, "rubber"), make_material(P, "gunmetal")
    secs = [(0.002, -0.004, -0.004, 0.045, 0.045, 0.011, 0.013, 3.2),
            (0.025, -0.006, -0.006, 0.045, 0.045, 0.009, 0.012, 3.2),
            (0.045, -0.010, -0.008, 0.035, 0.035, 0.007, 0.010, 2.6),
            (0.055, -0.012, -0.009, 0.020, 0.020, 0.005, 0.008, 2.2)]
    o = [loft("toe_body", "x", secs, W)]
    o.append(loft("toe_sole", "x", [(x, cy, zs + 0.005, ry + 0.002, ry + 0.002, 0.005, 0.005, 4.0) for (x, cy, cz, ry, _, a, b, p) in secs], R, subsurf=1))
    o.append(cyl("toe_knuckle", (0, 0, 0), (0, 1, 0), 0.011, 0.06, GM))
    return finish(P, o)


def part_chest():
    """Modelled facing +x, rotated 180 deg about z on export (chest_hruh faces -x)."""
    P = "chest"
    W = make_material(P, "white", [
        ("z", 0.062, 0.062, "dark", []),                                  # abdomen under-suit
        ("z", 0.128, 0.0018, "groove", []),
        ("z", 0.300, 0.0018, "groove", []),
        ("y", 0.0, 0.0018, "groove", [("x", 0.0, 1), ("z", 0.13, 0.30)]),  # sternum
        ("y", 0.0, 0.0022, "groove", [("x", -1, 0.0), ("z", 0.13, 0.33)]),
        ("y", 0.112, 0.016, "dark", [("z", 0.10, 0.30)]),                  # lat side frames
        ("y", -0.112, 0.016, "dark", [("z", 0.10, 0.30)]),
        ("z", 0.136, 0.0016, "glow", [("x", 0.02, 1)]),                    # lower-rib light line
        # pectoral armour plates: lower edges rise from the sternum to the armpit
        ((0, -0.45, 0.89), 0.89 * 0.172, 0.0018, "groove", [("x", 0.0, 1), ("y", 0.0, 0.105)]),
        ((0, 0.45, 0.89), 0.89 * 0.172, 0.0018, "groove", [("x", 0.0, 1), ("y", -0.105, 0.0)]),
        ((0, -0.45, 0.89), 0.89 * 0.166, 0.0012, "accent", [("x", 0.0, 1), ("y", 0.035, 0.09)]),
        ((0, 0.45, 0.89), 0.89 * 0.166, 0.0012, "accent", [("x", 0.0, 1), ("y", -0.09, -0.035)]),
    ])
    D, M, G = make_material(P, "dark"), make_material(P, "metal"), make_material(P, "glow")
    GM, A = make_material(P, "gunmetal"), make_material(P, "accent")
    WP = make_material(P + "_plates", "white")
    shell = loft("rib_shell", "z", [
        (0.000, 0.000, 0, 0.074, 0.068, 0.092, 0.092, 2.6),
        (0.060, 0.002, 0, 0.084, 0.074, 0.103, 0.103, 2.8),
        (0.140, 0.006, 0, 0.098, 0.084, 0.120, 0.120, 3.0),
        (0.215, 0.012, 0, 0.108, 0.088, 0.126, 0.126, 3.2),
        (0.270, 0.006, 0, 0.094, 0.086, 0.116, 0.116, 3.0),
        (0.320, 0.000, 0, 0.070, 0.074, 0.094, 0.094, 2.8),
        (0.352, 0.000, 0, 0.046, 0.050, 0.062, 0.062, 2.4),
        (0.368, 0.000, 0, 0.036, 0.036, 0.040, 0.040, 2.2),
    ], W)
    bpy.context.view_layer.update()
    o = [shell]
    # abdominal plates (3 rows x 2)
    for row, z in enumerate((0.022, 0.058, 0.094)):
        for s in (1, -1):
            hit, nor = surface_point(shell, (0.5, s * 0.026, z), (-1, 0, 0))
            o.append(rbox("abs", hit + Vector((-0.002, 0, 0)), (0.014, 0.040 - row * 0.002, 0.029), WP, 0.0065, rot=(0, -0.08, 0)))
    # chest core (light) between the pectorals
    hit, nor = surface_point(shell, (0.5, 0, 0.198), (-1, 0, 0))
    o.append(cyl("core_bezel", hit + Vector((0.004, 0, 0)), (1, 0, 0), 0.03, 0.018, GM))
    o.append(torus("core_ring", hit + Vector((0.013, 0, 0)), (1, 0, 0), 0.026, 0.0035, M))
    o.append(cyl("core_light", hit + Vector((0.009, 0, 0)), (1, 0, 0), 0.019, 0.016, G, bevel=0.003))
    # clavicles, shoulder flanges, collar
    for s in (1, -1):
        o.append(seg("clavicle", (0.014, s * 0.03, 0.348), (0.0, s * 0.098, 0.318), 0.015, D))
        o.append(cyl("shoulder_flange", (0, s * 0.103, 0.315), (0, 1, 0), 0.037, 0.016, M))
        o.append(cyl("shoulder_ring", (0, s * 0.095, 0.315), (0, 1, 0), 0.04, 0.006, A))
    o.append(torus("collar", (0, 0, 0.362), (0, 0, 1), 0.038, 0.008, D))
    o.append(cyl("neck_mount", (0, 0, 0.364), (0, 0, 1), 0.034, 0.018, M))
    # back: spine, vents, name plate
    for i in range(7):
        z = 0.13 + i * 0.03
        hit, nor = surface_point(shell, (-0.5, 0, z), (1, 0, 0))
        o.append(rbox("vertebra", hit + Vector((-0.002, 0, 0)), (0.016, 0.03, 0.022), D, 0.006))
    for s in (1, -1):
        hit, nor = surface_point(shell, (-0.5, s * 0.058, 0.215), (1, 0, 0))
        base = hit + Vector((0.004, 0, 0))
        o.append(rbox("vent_panel", base, (0.02, 0.05, 0.1), D, 0.006))
        for k in range(6):
            o.append(rbox("vent_slat", base + Vector((-0.0105, 0, -0.04 + k * 0.016)), (0.004, 0.042, 0.005), M, 0.0015))
    hit, nor = surface_point(shell, (-0.5, 0, 0.075), (1, 0, 0))
    plate = rbox("name_plate", hit + Vector((0.0, 0, 0)), (0.012, 0.11, 0.03), GM, 0.004)
    o.append(plate)
    o.append(text_plate("name", "HRUH", hit + Vector((-0.0068, 0, 0)), (0, -1, 0), (0, 0, 1), 0.02, G))
    obj = finish(P, o)
    return obj


def boolean_diff(target, cutters):
    for c in cutters:
        md = target.modifiers.new("cut", "BOOLEAN"); md.operation = "DIFFERENCE"; md.solver = "EXACT"; md.object = c
        md.material_mode = "INDEX"
        bpy.context.view_layer.objects.active = target
        bpy.ops.object.modifier_apply(modifier=md.name)
        bpy.data.objects.remove(c, do_unlink=True)


def part_head():
    """Human-proportioned head.  Modelled facing +x relative to the
    chest_to_neck joint; on export it is rotated 180 deg and shifted down to the
    nod pivot (head_base frame).  Eyes = stereo cameras, crown = LiDAR."""
    P = "head"
    W = make_material(P, "white", [
        ("x", 0.012, 0.0016, "groove", [("z", 0.07, 0.235)]),                    # face-mask seam
        ("z", 0.214, 0.0015, "groove", [("x", 0.012, 1)]),                       # hairline seam
        ("z", 0.089, 0.0016, "groove", [("x", 0.05, 1), ("y", -0.019, 0.019)]),  # lips
        ("y", 0.0, 0.0016, "groove", [("x", -1, -0.02), ("z", 0.10, 0.236)]),    # occipital seam
        ("z", 0.072, 0.010, "dark", [("x", -1, 0.0)]),                           # nape
    ])
    D, M, G = make_material(P, "dark"), make_material(P, "metal"), make_material(P, "glow")
    GM, LN = make_material(P, "gunmetal"), make_material(P, "lens")
    WD = make_material(P + "_details", "white")
    #      z      cx     front  back   half-width     p
    skull = loft("skull", "z", [
        (0.052, 0.040, 0, 0.026, 0.020, 0.024, 0.024, 2.2),   # chin tip
        (0.064, 0.030, 0, 0.050, 0.036, 0.040, 0.040, 2.2),   # chin / jaw line
        (0.082, 0.016, 0, 0.068, 0.062, 0.054, 0.054, 2.2),   # jaw
        (0.102, 0.008, 0, 0.078, 0.080, 0.062, 0.062, 2.3),   # mouth
        (0.124, 0.004, 0, 0.083, 0.084, 0.068, 0.068, 2.4),   # cheekbones
        (0.150, 0.000, 0, 0.082, 0.090, 0.071, 0.071, 2.4),   # eyes (slightly recessed)
        (0.172, 0.000, 0, 0.088, 0.093, 0.073, 0.073, 2.4),   # brow ridge
        (0.198, -0.004, 0, 0.082, 0.092, 0.073, 0.073, 2.3),  # forehead
        (0.222, -0.008, 0, 0.068, 0.082, 0.066, 0.066, 2.3),
        (0.240, -0.010, 0, 0.050, 0.066, 0.052, 0.052, 2.2),  # crown (LiDAR seat)
    ], W)
    bpy.ops.object.select_all(action="DESELECT"); skull.select_set(True)
    bpy.context.view_layer.objects.active = skull
    bpy.ops.object.convert(target="MESH")
    ey, ez = KIN["eye_y"], KIN["eye_z"]
    hits = [surface_point(skull, (0.5, s * ey, ez), (-1, 0, 0))[0] for s in (1, -1)]
    # nose: lofted along z, following the face surface
    nose = []
    for z, d, w in ((0.158, 0.001, 0.005), (0.145, 0.006, 0.006), (0.130, 0.011, 0.008),
                    (0.120, 0.014, 0.009), (0.114, 0.008, 0.011), (0.111, 0.001, 0.008)):
        h = surface_point(skull, (0.5, 0, z), (-1, 0, 0))[0]
        nose.append((z, h.x - 0.004, 0, d + 0.004, 0.004, w, w, 2.2))
    nose_o = loft("nose", "z", nose[::-1], WD, n=32)
    # almond eye sockets
    cutters = []
    for h in hits:
        bpy.ops.mesh.primitive_uv_sphere_add(radius=1, segments=48, ring_count=24, location=h + Vector((0.003, 0, 0)))
        c = bpy.context.object; c.scale = (0.016, 0.017, 0.0115)
        cutters.append(c)
    boolean_diff(skull, cutters)
    o = [skull, nose_o]
    ex = min(h.x for h in hits)
    for h in hits:
        c = Vector((ex, h.y, h.z))
        o.append(ellipsoid("eyeball", c + Vector((-0.011, 0, 0)), (0.012, 0.0145, 0.0105), GM))
        o.append(cyl("eye_lens", c + Vector((-0.0012, 0, 0)), (1, 0, 0), 0.0075, 0.003, LN, bevel=0.0008))
        o.append(torus("eye_iris", c + Vector((-0.0008, 0, 0)), (1, 0, 0), 0.0086, 0.0009, G))
    # ears with microphones
    for s in (1, -1):
        hit, nor = surface_point(skull, (-0.008, s * 0.5, 0.138), (0, -s, 0))
        o.append(ellipsoid("ear", hit + Vector((0, s * 0.002, 0)), (0.014, 0.006, 0.025), D))
        o.append(cyl("ear_mic", hit + Vector((0, s * 0.007, 0.002)), (0, 1, 0), 0.006, 0.003, M))
    # LiDAR crown, sunk into the skull
    lx, lz = -0.010, 0.247
    o.append(torus("lidar_collar", (lx, 0, 0.238), (0, 0, 1), 0.047, 0.0035, M))
    o.append(cyl("lidar_body", (lx, 0, lz), (0, 0, 1), 0.046, 0.022, D))
    o.append(cyl("lidar_window", (lx, 0, lz + 0.001), (0, 0, 1), 0.0466, 0.008, LN, bevel=0.0006))
    o.append(cyl("lidar_light", (lx, 0, lz + 0.0062), (0, 0, 1), 0.0469, 0.0014, G, bevel=0.0))
    o.append(cyl("lidar_cap", (lx, 0, lz + 0.0135), (0, 0, 1), 0.044, 0.006, WD, bevel=0.0025))
    o.append(cyl("lidar_hub", (lx, 0, lz + 0.017), (0, 0, 1), 0.012, 0.002, M))
    print("HEAD eye_front_x %.4f lidar x %.4f z %.4f" % (ex - 0.0005, lx, lz + 0.001))
    return finish(P, o)

def part_neck():
    """Ribbed neck (chest_to_neck frame) up to the head nod pivot."""
    P = "neck"
    D, M, GM = make_material(P, "dark"), make_material(P, "metal"), make_material(P, "gunmetal")
    secs = []
    z = 0.0
    for i in range(9):
        r = 0.031 if i % 2 == 0 else 0.027
        secs.append((z, 0.0, 0, r, r, r * 1.08, r * 1.08, 2.2))
        z += 0.0082
    o = [loft("neck_bellows", "z", secs, D, n=40, subsurf=1)]
    o.append(cyl("neck_core", (0, 0, 0.04), (0, 0, 1), 0.02, 0.08, GM))
    o.append(cyl("nod_axle", (0, 0, KIN["neck_pitch_z"]), (0, 1, 0), 0.016, 0.07, M))
    return finish(P, o)


PARTS = [  # (builder, texture/part name, export dir, mirrored?)
    (part_pelvis, "pelvis", "legs", False),
    (part_waist, "waist", "legs", False),
    (part_hip_yaw, "hip_yaw", "legs", True),
    (part_hip_roll, "hip_roll", "legs", True),
    (part_thigh, "thigh", "legs", True),
    (part_shin, "shin", "legs", True),
    (part_ankle, "ankle", "legs", True),
    (part_foot, "foot", "legs", True),
    (part_toe, "toe", "legs", True),
    (part_chest, "chest", "chest", False),
    (part_head, "head", "head", False),
    (part_neck, "neck_v2", "neck", False),
]


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    built = []
    for fn, name, sub, mirrored in PARTS:
        if ONLY and name not in ONLY:
            continue
        outdir = os.path.join(OUT, sub); os.makedirs(os.path.join(outdir, "glb"), exist_ok=True)
        obj = fn()
        tex = name + "_v2" if name in ("chest", "head") else name
        bake(obj, tex, outdir, 2048 if name in ("chest", "head") else TEX)
        if name in ("chest", "head"):
            # chest_hruh / head_base frames face -x; the head frame sits at the nod pivot
            obj.rotation_euler = (0, 0, math.pi)
            if name == "head":
                obj.location = (0, 0, -KIN["neck_pitch_z"])
            bpy.ops.object.select_all(action="DESELECT"); obj.select_set(True)
            bpy.context.view_layer.objects.active = obj
            bpy.ops.object.transform_apply(location=True, rotation=True, scale=False)
            obj.name = tex
            export_part(obj, outdir, tex)
            built.append(obj)
        elif mirrored:
            obj.name = "left_" + name
            export_part(obj, outdir, "left_" + name)
            r = mirror_y(obj, "right_" + name)
            export_part(r, outdir, "right_" + name)
            built += [obj, r]
        else:
            export_part(obj, outdir, name)
            built.append(obj)
    # lay parts out in a row so the saved .blend is easy to browse
    for i, o in enumerate(built):
        o.location = (0, i * 0.35, 0)
    blend = os.path.join(PKG, "blender", "humanoid_parts.blend")
    if not ONLY:
        bpy.ops.wm.save_as_mainfile(filepath=blend)
        print("SAVED", blend)


if __name__ == "__main__":
    main()
