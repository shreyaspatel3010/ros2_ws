"""Joint groups and the standing pose shared by every HRUH task.

Pure Python (no Isaac imports) so the ROS 2 policy runner and the motion-clip
tools can use the same definitions.
"""
SIDES = ("left", "right")
LEG_JOINTS = [f"{s}_{j}_joint" for s in SIDES
              for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll", "toe")]
WAIST_JOINTS = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
HEAD_JOINTS = ["chest_to_neck", "neck_to_head"]
ARM_JOINTS = {s: [f"chest_to_{s}_shoulder", f"{s}_shoulder_to_bisecp", f"{s}_bisecp_to_elbow_inword",
                  f"{s}_elbow_inword_to_midle", f"{s}_forarm_to_wrist"] for s in SIDES}
HAND_JOINT_REGEX = [".*_thomb.*", ".*finger.*"]

# joints a whole-body policy drives (hands are left to grasp controllers)
BODY_JOINTS = LEG_JOINTS + WAIST_JOINTS + ARM_JOINTS["left"] + ARM_JOINTS["right"] + HEAD_JOINTS

# standing pose (same as hruh_walker: soft knees, arms slightly away from the body)
STAND_POSE = {
    ".*_hip_pitch_joint": -0.21,
    ".*_knee_joint": 0.36,
    ".*_ankle_pitch_joint": -0.15,
    "left_shoulder_to_bisecp": -0.10,
    "right_shoulder_to_bisecp": 0.10,
    ".*_elbow_inword_to_midle": 0.30,
}
STAND_HEIGHT = 0.823          # pelvis (base_link) height in that pose
KEY_BODIES = ["left_foot_link", "right_foot_link", "left_wrist", "right_wrist"]   # palms are merged into the wrists


def stand_pose_value(joint_name):
    import re
    for pat, v in STAND_POSE.items():
        if re.fullmatch(pat, joint_name):
            return v
    return 0.0
