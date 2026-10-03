"""Minimal URDF -> Blender loader (forward kinematics, meshes + primitives).

Usage inside Blender:
    import urdf_blender as ub
    robot = ub.Robot(urdf_path, pkg_root)
    robot.build()              # creates objects, one empty per link
    robot.set_joints({...})    # pose the robot (FK applied via parented empties)
"""
import math, os, xml.etree.ElementTree as ET
import bpy
from mathutils import Matrix, Vector, Euler


def rpy_xyz(el):
    if el is None:
        return Matrix.Identity(4)
    xyz = [float(v) for v in el.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in el.get("rpy", "0 0 0").split()]
    rot = Euler(rpy, "XYZ").to_matrix().to_4x4()  # URDF fixed-axis RPY == Blender XYZ euler
    return Matrix.Translation(xyz) @ rot


class Robot:
    def __init__(self, urdf_path, pkg_root, pkg_name="my_robot_description"):
        self.tree = ET.parse(urdf_path).getroot()
        self.pkg_root, self.pkg_name = pkg_root, pkg_name
        self.links = {l.get("name"): l for l in self.tree.findall("link")}
        self.joints = {j.get("name"): j for j in self.tree.findall("joint")}
        self.mats = {}
        for m in self.tree.findall("material"):
            c = m.find("color")
            if c is not None:
                self.mats[m.get("name")] = [float(v) for v in c.get("rgba").split()]
        self.empties = {}
        self.joint_empties = {}

    def resolve(self, fn):
        pre = "package://%s/" % self.pkg_name
        if fn.startswith(pre):
            return os.path.join(self.pkg_root, fn[len(pre):])
        return fn

    def root_link(self):
        children = {j.find("child").get("link") for j in self.joints.values()}
        return [n for n in self.links if n not in children][0]

    def _material(self, name, rgba):
        key = name or "c_%.3f_%.3f_%.3f" % tuple(rgba[:3])
        mat = bpy.data.materials.get("urdf_" + key)
        if mat is None:
            mat = bpy.data.materials.new("urdf_" + key)
            mat.use_nodes = True
            bsdf = mat.node_tree.nodes.get("Principled BSDF")
            bsdf.inputs["Base Color"].default_value = rgba
            bsdf.inputs["Roughness"].default_value = 0.45
            bsdf.inputs["Metallic"].default_value = 0.2
        return mat

    def _import_mesh(self, path):
        before = set(bpy.data.objects)
        ext = os.path.splitext(path)[1].lower()
        if ext == ".stl":
            bpy.ops.wm.stl_import(filepath=path)
        elif ext == ".obj":
            bpy.ops.wm.obj_import(filepath=path, forward_axis="Y", up_axis="Z")
        elif ext in (".glb", ".gltf"):
            bpy.ops.import_scene.gltf(filepath=path)
        else:
            raise RuntimeError("unsupported mesh " + path)
        new = [o for o in bpy.data.objects if o not in before]
        return new

    def build(self, collection_name="robot", textured=True):
        col = bpy.data.collections.new(collection_name)
        bpy.context.scene.collection.children.link(col)
        self.col = col
        for name, link in self.links.items():
            e = bpy.data.objects.new("L_" + name, None)
            e.empty_display_size = 0.02
            col.objects.link(e)
            self.empties[name] = e
        # joint frames: parent_link_empty -> joint_origin_empty -> child_link_empty
        for jname, j in self.joints.items():
            p = j.find("parent").get("link")
            c = j.find("child").get("link")
            je = bpy.data.objects.new("J_" + jname, None)
            je.empty_display_size = 0.01
            col.objects.link(je)
            je.parent = self.empties[p]
            je.matrix_parent_inverse = Matrix.Identity(4)
            je.matrix_basis = rpy_xyz(j.find("origin"))
            ce = self.empties[c]
            ce.parent = je
            ce.matrix_parent_inverse = Matrix.Identity(4)
            ce.rotation_mode = "QUATERNION"     # no euler flips when animating
            ce.matrix_basis = Matrix.Identity(4)
            ax = j.find("axis")
            axis = Vector([float(v) for v in ax.get("xyz").split()]) if ax is not None else Vector((1, 0, 0))
            self.joint_empties[jname] = (ce, j.get("type"), axis)
        # visuals
        for name, link in self.links.items():
            for vis in link.findall("visual"):
                g = vis.find("geometry")
                m = rpy_xyz(vis.find("origin"))
                objs = []
                if g.find("mesh") is not None:
                    me = g.find("mesh")
                    s = [float(v) for v in me.get("scale", "1 1 1").split()]
                    objs = self._import_mesh(self.resolve(me.get("filename")))
                    for o in objs:
                        o.matrix_world = m @ Matrix.Diagonal((s[0], s[1], s[2], 1.0))
                elif g.find("box") is not None:
                    sz = [float(v) for v in g.find("box").get("size").split()]
                    bpy.ops.mesh.primitive_cube_add(size=1)
                    o = bpy.context.object
                    o.matrix_world = m @ Matrix.Diagonal((sz[0], sz[1], sz[2], 1))
                    objs = [o]
                elif g.find("cylinder") is not None:
                    cy = g.find("cylinder")
                    r, h = float(cy.get("radius")), float(cy.get("length"))
                    bpy.ops.mesh.primitive_cylinder_add(radius=r, depth=h, vertices=48)
                    o = bpy.context.object
                    o.matrix_world = m
                    objs = [o]
                elif g.find("sphere") is not None:
                    r = float(g.find("sphere").get("radius"))
                    bpy.ops.mesh.primitive_uv_sphere_add(radius=r)
                    o = bpy.context.object
                    o.matrix_world = m
                    objs = [o]
                mat_el = vis.find("material")
                rgba = None
                if mat_el is not None:
                    c = mat_el.find("color")
                    rgba = [float(v) for v in c.get("rgba").split()] if c is not None else self.mats.get(mat_el.get("name"))
                for o in objs:
                    for cc in o.users_collection:
                        cc.objects.unlink(o)
                    col.objects.link(o)
                    has_tex = textured and len(o.data.materials) > 0 and o.data.materials[0] is not None
                    if not has_tex and rgba is not None:
                        o.data.materials.clear()
                        o.data.materials.append(self._material(mat_el.get("name"), rgba))
                    mw = o.matrix_world.copy()
                    o.parent = self.empties[name]
                    o.matrix_parent_inverse = Matrix.Identity(4)
                    o.matrix_basis = mw
                    o.name = "V_" + name
        root = self.empties[self.root_link()]
        root.rotation_mode = "QUATERNION"
        root.matrix_basis = Matrix.Identity(4)
        return col

    def set_joints(self, q, root_matrix=None):
        for jname, (ce, jt, axis) in self.joint_empties.items():
            v = q.get(jname, 0.0)
            if jt in ("revolute", "continuous"):
                ce.matrix_basis = Matrix.Rotation(v, 4, axis)
            elif jt == "prismatic":
                ce.matrix_basis = Matrix.Translation(axis * v)
        if root_matrix is not None:
            self.empties[self.root_link()].matrix_basis = root_matrix

    def keyframe(self, frame):
        for jname, (ce, jt, axis) in self.joint_empties.items():
            if jt in ("revolute", "continuous"):
                ce.keyframe_insert("rotation_quaternion", frame=frame)
            elif jt == "prismatic":
                ce.keyframe_insert("location", frame=frame)
        r = self.empties[self.root_link()]
        r.rotation_mode = "QUATERNION"
        r.keyframe_insert("location", frame=frame)
        r.keyframe_insert("rotation_quaternion", frame=frame)

    def world_bounds(self):
        bpy.context.view_layer.update()
        mn = Vector((1e9, 1e9, 1e9)); mx = -mn
        for o in self.col.objects:
            if o.type != "MESH":
                continue
            for c in o.bound_box:
                w = o.matrix_world @ Vector(c)
                mn = Vector(map(min, mn, w)); mx = Vector(map(max, mx, w))
        return mn, mx


def upgrade_baked_materials():
    """OBJ import only wires the albedo map.  If the matching *_rm.png
    (roughness, metallic) and *_glow.png maps exist next to it, hook them up so
    Blender renders show metal, rubber and the light strips properly."""
    for mat in bpy.data.materials:
        if not mat.use_nodes:
            continue
        nt = mat.node_tree
        bsdf = next((n for n in nt.nodes if n.type == "BSDF_PRINCIPLED"), None)
        img = next((n for n in nt.nodes if n.type == "TEX_IMAGE" and n.image), None)
        if bsdf is None or img is None or not img.image.filepath.endswith("_albedo.png"):
            continue
        base = bpy.path.abspath(img.image.filepath)[: -len("_albedo.png")]
        if os.path.exists(base + "_rm.png"):
            rm = nt.nodes.new("ShaderNodeTexImage")
            rm.image = bpy.data.images.load(base + "_rm.png", check_existing=True)
            rm.image.colorspace_settings.name = "Non-Color"
            sp = nt.nodes.new("ShaderNodeSeparateColor")
            nt.links.new(rm.outputs["Color"], sp.inputs[0])
            nt.links.new(sp.outputs[1], bsdf.inputs["Roughness"])   # glTF layout: G roughness, B metallic
            nt.links.new(sp.outputs[2], bsdf.inputs["Metallic"])
        if os.path.exists(base + "_glow.png"):
            gl = nt.nodes.new("ShaderNodeTexImage")
            gl.image = bpy.data.images.load(base + "_glow.png", check_existing=True)
            nt.links.new(gl.outputs["Color"], bsdf.inputs["Emission Color"])
            bsdf.inputs["Emission Strength"].default_value = 4.0
        bsdf.inputs["Specular IOR Level"].default_value = 0.5
