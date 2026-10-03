"""HRUH articulation for Isaac Lab, spawned straight from the URDF.

The URDF comes from `ros2 run hruh_isaac export_isaac_urdf.py` (absolute mesh
paths, no Gazebo / ros2_control tags); override with the HRUH_URDF variable.
Isaac Lab converts it to USD once and caches it.
"""
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from .joints import STAND_HEIGHT, STAND_POSE

HRUH_URDF = os.path.expanduser(os.environ.get("HRUH_URDF", "~/.cache/hruh/hruh_isaac.urdf"))
if not os.path.exists(HRUH_URDF):
    raise FileNotFoundError(f"{HRUH_URDF} not found: run `ros2 run hruh_isaac export_isaac_urdf.py` first")


def _spawn(fix_base: bool) -> sim_utils.UrdfFileCfg:
    return sim_utils.UrdfFileCfg(
        asset_path=HRUH_URDF,
        fix_base=fix_base,
        merge_fixed_joints=True,                       # massless frames (cameras, palm, pelvis) fold into parents
        convert_mimic_joints_to_normal_joints=True,    # finger segments become independent joints
        self_collision=False,
        collider_type="convex_hull",
        activate_contact_sensors=True,
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False, retain_accelerations=False, linear_damping=0.0, angular_damping=0.0,
            max_linear_velocity=1000.0, max_angular_velocity=1000.0, max_depenetration_velocity=1.0),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=1),
    )


ACTUATORS = {
    # PD gains in N*m/rad, N*m*s/rad; effort / speed limits follow the URDF
    "legs": ImplicitActuatorCfg(
        joint_names_expr=[".*_hip_yaw_joint", ".*_hip_roll_joint", ".*_hip_pitch_joint", ".*_knee_joint"],
        effort_limit_sim=250.0, velocity_limit_sim=10.0,
        stiffness={".*_hip_yaw_joint": 150.0, ".*_hip_roll_joint": 200.0, ".*_hip_pitch_joint": 200.0,
                   ".*_knee_joint": 250.0},
        damping={".*_hip_yaw_joint": 5.0, ".*_hip_roll_joint": 5.0, ".*_hip_pitch_joint": 5.0, ".*_knee_joint": 6.0},
        armature=0.01,
    ),
    "feet": ImplicitActuatorCfg(
        joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint", ".*_toe_joint"],
        effort_limit_sim=120.0, velocity_limit_sim=10.0,
        stiffness={".*_ankle_.*": 60.0, ".*_toe_joint": 15.0},
        damping={".*_ankle_.*": 3.0, ".*_toe_joint": 0.5},
        armature=0.01,
    ),
    "waist": ImplicitActuatorCfg(
        joint_names_expr=["waist_.*"], effort_limit_sim=150.0, velocity_limit_sim=4.0,
        stiffness=200.0, damping=6.0, armature=0.01,
    ),
    "arms": ImplicitActuatorCfg(
        joint_names_expr=["chest_to_.*_shoulder", ".*_shoulder_to_bisecp", ".*_bisecp_to_elbow_inword",
                          ".*_elbow_inword_to_midle", ".*_forarm_to_wrist"],
        effort_limit_sim=60.0, velocity_limit_sim=4.0, stiffness=60.0, damping=3.0, armature=0.01,
    ),
    "head": ImplicitActuatorCfg(
        joint_names_expr=["chest_to_neck", "neck_to_head"], effort_limit_sim=20.0, velocity_limit_sim=4.0,
        stiffness=30.0, damping=2.0,
    ),
    "hands": ImplicitActuatorCfg(
        joint_names_expr=[".*_thomb.*", ".*finger.*"], effort_limit_sim=10.0, velocity_limit_sim=6.0,
        stiffness=5.0, damping=0.2,
    ),
}

HRUH_CFG = ArticulationCfg(
    spawn=_spawn(fix_base=False),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, STAND_HEIGHT + 0.01), joint_pos=STAND_POSE, joint_vel={".*": 0.0}),
    soft_joint_pos_limit_factor=0.9,
    actuators=ACTUATORS,
)
"""Free-floating HRUH (locomotion, motion imitation)."""

HRUH_FIXED_CFG = HRUH_CFG.replace(
    spawn=_spawn(fix_base=True),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 1.0), joint_pos=STAND_POSE, joint_vel={".*": 0.0}),
)
"""Pelvis fixed in the air (arm skills without balancing)."""
