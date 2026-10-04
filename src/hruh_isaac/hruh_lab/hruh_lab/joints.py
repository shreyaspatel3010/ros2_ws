"""Joint groups and the standing pose shared by every HRUH task.

Pure Python (no Isaac imports) so the ROS 2 policy runner and the motion-clip
tools can use the same definitions.
"""
SIDES = ("left", "right")
LEG_JOINTS = [f"{s}_{j}_joint" for s in SIDES
              for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll", "toe")]
HEAD_JOINTS = ["chest_to_neck", "neck_to_head"]
ARM_JOINTS = {s: [f"chest_to_{s}_shoulder", f"{s}_shoulder_to_bisecp", f"{s}_bisecp_to_elbow_inword",
                  f"{s}_elbow_inword_to_midle", f"{s}_forarm_to_wrist"] for s in SIDES}
HAND_JOINT_REGEX = [".*_thomb.*", ".*finger.*"]

# joints a whole-body policy drives (hands are left to grasp controllers)
BODY_JOINTS = LEG_JOINTS + ARM_JOINTS["left"] + ARM_JOINTS["right"] + HEAD_JOINTS   # (rigid torso: no waist)

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


# ---------------------------------------------------------------- arm motion while walking
# Natural counter-swing (same convention as hruh_walker): shoulder flexion follows the
# *opposite* leg. phase = +1 when the left leg is fully forward (its hip more flexed),
# then the left arm swings back (+) and the right arm forward (-).
ARM_SWING_HIP_SPAN = 0.6      # rad of right-minus-left hip pitch that counts as a full stride
ARM_SWING_GAIN = 0.30         # rad shoulder amplitude used at deployment (training samples 0.15-0.45)
ARM_SWING_ELBOW = 0.4         # extra elbow flexion per rad of gain when the arm swings forward


def arm_swing_targets(hip_pitch_left, hip_pitch_right, gain=ARM_SWING_GAIN, stand=None):
    """Arm joint targets {name: rad} for the walking counter-swing (pure Python, deployment)."""
    stand = stand or {}
    phase = max(-1.0, min(1.0, (hip_pitch_right - hip_pitch_left) / ARM_SWING_HIP_SPAN))
    targets = {n: stand.get(n, stand_pose_value(n)) for s in SIDES for n in ARM_JOINTS[s]}
    targets["chest_to_left_shoulder"] += gain * phase
    targets["chest_to_right_shoulder"] -= gain * phase
    targets["left_elbow_inword_to_midle"] += ARM_SWING_ELBOW * gain * max(0.0, -phase)
    targets["right_elbow_inword_to_midle"] += ARM_SWING_ELBOW * gain * max(0.0, phase)
    return targets


# Arm poses an operator / MoveIt may hold while the robot walks (training samples inside these).
ARM_POSE_RANGES = {
    "chest_to_{s}_shoulder": (-1.5, 0.4),          # negative = arm forward / up
    "left_shoulder_to_bisecp": (-1.0, 0.0),        # abduction (sign mirrors per side)
    "right_shoulder_to_bisecp": (0.0, 1.0),
    "{s}_bisecp_to_elbow_inword": (-0.8, 0.8),
    "{s}_elbow_inword_to_midle": (0.0, 1.6),
    "{s}_forarm_to_wrist": (-1.0, 1.0),
}


def arm_pose_range(name):
    for pattern, value in ARM_POSE_RANGES.items():
        if any(pattern.format(s=s) == name for s in SIDES):
            return value
    raise KeyError(name)
