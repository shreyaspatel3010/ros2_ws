"""Export every HRUH body part as a print-ready 3MF (one object per part).

    blender -b --factory-startup -P blender/export_3mf.py -- [out.3mf] [scale]

  * Source: the textured OBJ meshes the URDF uses (legs, pelvis, waist, chest,
    neck, head and the cleaned arm / hand parts).
  * Each part is fused into one closed, manifold solid (voxel remesh) so slicers
    get a valid volume, then decimated to a printer-friendly triangle count.
  * Per-triangle colours are read back from the baked material maps and stored
    as 3MF base materials (white shell, black frame, grey metal, blue accent,
    cyan light) for multi-material printers; single-colour printers ignore them.
  * Units are millimetres, real size by default (pass a scale such as 0.2 for a
    1:5 model).  Parts are laid out flat on the plate; re-arrange / orient in
    your slicer.
"""
import math, os, sys, zipfile, glob
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
import bpy, bmesh
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT = os.path.abspath(argv[0]) if argv else os.path.join(PKG, "print", "hruh_robot_parts.3mf")
SCALE = float(argv[1]) if len(argv) > 1 else 1.0
os.makedirs(os.path.dirname(OUT), exist_ok=True)

# print colours: name, sRGB hex, and the (roughness, metallic) signature baked into *_rm.png
PRINT_COLOURS = [
    ("White", "#E4E6EAFF"), ("Black", "#1C1E22FF"), ("Silver", "#9EA3A9FF"),
    ("Blue", "#1F5BE0FF"), ("Cyan", "#4FD4FFFF"),
]
SIGNATURES = [  # (roughness, metallic) -> colour index   (see PALETTE in build_parts.py)
    ((0.38, 0.0), 0),   # white shell
    ((0.60, 0.0), 0),   # panel grooves print as part of the shell
    ((0.42, 0.2), 1),   # carbon / dark frame
    ((0.92, 0.0), 1),   # rubber sole
    ((0.05, 0.0), 1),   # lens glass
    ((0.30, 0.85), 2),  # gunmetal actuators
    ((0.22, 1.0), 2),   # bright metal
    ((0.30, 0.1), 3),   # blue accent
    ((0.20, 0.0), 4),   # light strips
]
SIG = np.array([s for s, _ in SIGNATURES]); SIG_COL = np.array([c for _, c in SIGNATURES])


def part_files():
    files = []
    for sub in ("legs", "chest", "neck", "head", "arms"):
        for f in sorted(glob.glob(os.path.join(PKG, "meshes", sub, "*.obj"))):
            files.append(f)
    return files


def load_map(path):
    img = bpy.data.images.load(path, check_existing=True)
    img.colorspace_settings.name = "Non-Color"
    a = np.empty(img.size[0] * img.size[1] * 4, np.float32)
    img.pixels.foreach_get(a)
    return a.reshape(img.size[1], img.size[0], 4)


def classify_faces(src, solid):
    """Colour index per triangle of `solid`, sampled from src's rm/glow maps."""
    mat = src.data.materials[0]
    tex = next(n for n in mat.node_tree.nodes if n.type == "TEX_IMAGE" and n.image)
    base = bpy.path.abspath(tex.image.filepath)[: -len("_albedo.png")]
    rm, glow = load_map(base + "_rm.png"), load_map(base + "_glow.png")
    h, w = rm.shape[:2]
    me = src.data
    uv = me.uv_layers.active.data
    bvh = BVHTree.FromObject(src, bpy.context.evaluated_depsgraph_get())
    tri_uv = {}
    out = np.zeros(len(solid.data.polygons), np.int32)
    for i, p in enumerate(solid.data.polygons):
        loc, nor, fi, dist = bvh.find_nearest(p.center)
        if fi is None:
            continue
        sp = me.polygons[fi]
        vs = [me.vertices[v].co for v in sp.vertices[:3]]
        uvs = [uv[li].uv for li in list(sp.loop_indices)[:3]]
        # barycentric coordinates of loc in the source triangle
        v0, v1, v2 = vs[1] - vs[0], vs[2] - vs[0], loc - vs[0]
        d00, d01, d11, d20, d21 = v0.dot(v0), v0.dot(v1), v1.dot(v1), v2.dot(v0), v2.dot(v1)
        den = d00 * d11 - d01 * d01 or 1e-12
        b1 = (d11 * d20 - d01 * d21) / den; b2 = (d00 * d21 - d01 * d20) / den; b0 = 1 - b1 - b2
        u = b0 * uvs[0].x + b1 * uvs[1].x + b2 * uvs[2].x
        v = b0 * uvs[0].y + b1 * uvs[1].y + b2 * uvs[2].y
        px = min(w - 1, max(0, int(u * w))); py = min(h - 1, max(0, int(v * h)))
        if glow[py, px, :3].max() > 0.25:
            out[i] = 4
            continue
        r, m = rm[py, px, 1], rm[py, px, 2]
        out[i] = SIG_COL[np.argmin((SIG[:, 0] - r) ** 2 + (SIG[:, 1] - m) ** 2)]
    return out


def make_solid(src):
    solid = src.copy(); solid.data = src.data.copy(); solid.name = src.name + "_print"
    bpy.context.scene.collection.objects.link(solid)
    diag = (Vector(solid.bound_box[6]) - Vector(solid.bound_box[0])).length
    voxel = min(0.0012, max(0.0004, diag / 320.0))
    bpy.context.view_layer.objects.active = solid
    rm = solid.modifiers.new("remesh", "REMESH"); rm.mode = "VOXEL"; rm.voxel_size = voxel; rm.adaptivity = 0.0
    bpy.ops.object.modifier_apply(modifier="remesh")
    target = 60000 if diag > 0.2 else 30000 if diag > 0.08 else 12000
    tris = sum(len(p.vertices) - 2 for p in solid.data.polygons)
    if tris > target:
        dc = solid.modifiers.new("dec", "DECIMATE"); dc.ratio = target / tris; dc.use_collapse_triangulate = True
        bpy.ops.object.modifier_apply(modifier="dec")
    bm = bmesh.new(); bm.from_mesh(solid.data)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    manifold = all(e.is_manifold for e in bm.edges)
    volume = bm.calc_volume(signed=True)
    bm.to_mesh(solid.data); bm.free()
    return solid, manifold, volume, voxel


def model_xml(objects):
    """objects: list of (name, verts (n,3) mm, tris (m,3), colours (m,), (tx,ty))"""
    out = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">',
           '<metadata name="Title">HRUH humanoid - printable body parts</metadata>',
           '<metadata name="Designer">HRUH / my_robot_description</metadata>',
           '<metadata name="Application">blender/export_3mf.py</metadata>',
           '<resources>', '<basematerials id="1">']
    out += ['<base name="%s" displaycolor="%s"/>' % c for c in PRINT_COLOURS]
    out.append('</basematerials>')
    for oid, (name, V, T, C, _) in enumerate(objects, start=2):
        dom = int(np.bincount(C, minlength=len(PRINT_COLOURS)).argmax())
        out.append('<object id="%d" type="model" name="%s" pid="1" pindex="%d"><mesh><vertices>' % (oid, name, dom))
        out.append("".join('<vertex x="%.4f" y="%.4f" z="%.4f"/>' % tuple(v) for v in V))
        out.append('</vertices><triangles>')
        out.append("".join('<triangle v1="%d" v2="%d" v3="%d" pid="1" p1="%d"/>' % (t[0], t[1], t[2], c) for t, c in zip(T, C)))
        out.append('</triangles></mesh></object>')
    out.append('</resources><build>')
    for oid, (_, _, _, _, (tx, ty)) in enumerate(objects, start=2):
        out.append('<item objectid="%d" transform="1 0 0 0 1 0 0 0 1 %.3f %.3f 0"/>' % (oid, tx, ty))
    out.append('</build></model>')
    return "\n".join(out)


def write_3mf(path, xml):
    ct = ('<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
          '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
          '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
            'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        z.writestr("[Content_Types].xml", ct)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", xml)


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    parts, report = [], []
    for f in part_files():
        name = os.path.splitext(os.path.basename(f))[0]
        before = set(bpy.data.objects)
        bpy.ops.wm.obj_import(filepath=f, forward_axis="Y", up_axis="Z")
        src = [o for o in bpy.data.objects if o not in before][0]
        solid, manifold, vol, voxel = make_solid(src)
        cols = classify_faces(src, solid)
        me = solid.data
        V = np.array([v.co[:] for v in me.vertices]) * 1000.0 * SCALE
        T = np.array([p.vertices[:] for p in me.polygons])
        V -= [V[:, 0].min(), V[:, 1].min(), V[:, 2].min()]          # sit on the plate
        parts.append([name, V, T, cols, None])
        report.append((name, len(T), manifold, vol * 1e6 * SCALE ** 3, V.max(axis=0)))
        bpy.data.objects.remove(solid, do_unlink=True); bpy.data.objects.remove(src, do_unlink=True)
    # shelf layout on the plate, 12 mm gaps
    parts.sort(key=lambda p: -p[1][:, 1].max())
    x = y = row_h = 0.0
    for p in parts:
        sx, sy = p[1][:, 0].max(), p[1][:, 1].max()
        if x + sx > 1200 * max(SCALE, 0.25):
            x, y, row_h = 0.0, y + row_h + 12, 0.0
        p[4] = (x, y); x += sx + 12; row_h = max(row_h, sy)
    write_3mf(OUT, model_xml([tuple(p) for p in parts]))
    print("\n3MF %s  (%.1f MB, %d parts, scale %.3g)" % (OUT, os.path.getsize(OUT) / 1e6, len(parts), SCALE))
    for name, nt, man, vol, size in report:
        print("PART %-30s %6d tris  manifold=%s  volume=%8.1f cm3  size=%5.0f x %4.0f x %4.0f mm"
              % (name, nt, man, vol, size[0], size[1], size[2]))


main()
