#!/usr/bin/env python3
"""Write an Isaac Sim / Isaac Lab ready URDF of HRUH.

* expands the xacro with hardware:=none (no ros2_control / gz plugins)
* replaces package://<pkg>/ mesh URIs with absolute paths (Isaac's URDF
  importer does not resolve ROS packages)
* drops <gazebo> blocks

    ros2 run hruh_isaac export_isaac_urdf.py [-o ~/.cache/hruh/hruh_isaac.urdf]
"""
import argparse
import os
import re
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

DEFAULT_OUT = os.path.expanduser("~/.cache/hruh/hruh_isaac.urdf")


def build(out_path=DEFAULT_OUT):
    xacro = os.path.join(get_package_share_directory("my_robot_description"), "urdf", "my_robot.urdf.xacro")
    xml = subprocess.run(["xacro", xacro, "hardware:=none", "enable_lidar:=false",
                          "enable_stereo_cameras:=false", "enable_rgbd_camera:=false"],
                         check=True, capture_output=True, text=True).stdout

    def resolve(m):
        pkg, rel = m.group(1), m.group(2)
        return os.path.join(get_package_share_directory(pkg), rel)
    xml = re.sub(r"package://([^/]+)/([^\"']+)", resolve, xml)
    root = ET.fromstring(xml)
    for g in root.findall("gazebo"):
        root.remove(g)
    missing = [m.get("filename") for m in root.iter("mesh") if not os.path.exists(m.get("filename"))]
    if missing:
        raise SystemExit("missing meshes: %s" % missing[:5])
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    ET.ElementTree(root).write(out_path, encoding="utf-8", xml_declaration=True)
    return out_path, root


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--output", default=DEFAULT_OUT)
    path, root = build(os.path.expanduser(ap.parse_args().output))
    n_j = sum(1 for j in root.findall("joint") if j.get("type") != "fixed")
    print("wrote %s  (%d links, %d movable joints)" % (path, len(root.findall("link")), n_j))


if __name__ == "__main__":
    main()
