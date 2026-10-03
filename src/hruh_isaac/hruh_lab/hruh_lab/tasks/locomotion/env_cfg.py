"""HRUH velocity-tracking locomotion (adapted from Isaac Lab's H1 recipe).

Policy: legs + waist (17 joints, position targets around the standing pose).
Observations use only what the real robot / ROS runner has: pelvis IMU
(angular velocity, gravity direction), /cmd_vel, joint encoders, last action -
no ground-truth base velocity, no height scan on flat ground.
"""
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
    RewardsCfg,
)

from ... import mdp as hruh_mdp
from ...joints import ARM_JOINTS, HEAD_JOINTS, HAND_JOINT_REGEX, LEG_JOINTS, WAIST_JOINTS
from ...robots import HRUH_CFG

POLICY_JOINTS = LEG_JOINTS + WAIST_JOINTS
HELD_JOINTS = ARM_JOINTS["left"] + ARM_JOINTS["right"] + HEAD_JOINTS + HAND_JOINT_REGEX
FEET = ".*_foot_link"


@configclass
class HruhRewards(RewardsCfg):
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    lin_vel_z_l2 = None
    track_lin_vel_xy_exp = RewTerm(func=mdp.track_lin_vel_xy_yaw_frame_exp, weight=1.0,
                                   params={"command_name": "base_velocity", "std": 0.5})
    track_ang_vel_z_exp = RewTerm(func=mdp.track_ang_vel_z_world_exp, weight=1.0,
                                  params={"command_name": "base_velocity", "std": 0.5})
    feet_air_time = RewTerm(func=mdp.feet_air_time_positive_biped, weight=0.25,
                            params={"command_name": "base_velocity", "threshold": 0.4,
                                    "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FEET)})
    feet_slide = RewTerm(func=mdp.feet_slide, weight=-0.25,
                         params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=FEET),
                                 "asset_cfg": SceneEntityCfg("robot", body_names=FEET)})
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.0,
                             params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_.*", ".*_toe_joint"])})
    joint_deviation_hip = RewTerm(func=mdp.joint_deviation_l1, weight=-0.2,
                                  params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_yaw_joint", ".*_hip_roll_joint"])})
    joint_deviation_waist = RewTerm(func=mdp.joint_deviation_l1, weight=-0.2,
                                    params={"asset_cfg": SceneEntityCfg("robot", joint_names=WAIST_JOINTS)})
    joint_deviation_toes = RewTerm(func=mdp.joint_deviation_l1, weight=-0.05,
                                   params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*_toe_joint")})


@configclass
class HruhRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: HruhRewards = HruhRewards()

    def __post_init__(self):
        super().__post_init__()
        # scene
        self.scene.robot = HRUH_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        if self.scene.height_scanner:
            self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/base_link"

        # actions: legs + waist only
        self.actions.joint_pos.joint_names = POLICY_JOINTS
        self.actions.joint_pos.scale = 0.5

        # observations: deployable set (IMU + encoders + command)
        self.observations.policy.base_lin_vel = None
        self.observations.policy.joint_pos.params = {"asset_cfg": SceneEntityCfg("robot", joint_names=POLICY_JOINTS)}
        self.observations.policy.joint_vel.params = {"asset_cfg": SceneEntityCfg("robot", joint_names=POLICY_JOINTS)}

        # keep arms / head / hands in the standing pose (no action drives them)
        self.events.hold_upper_body = EventTerm(func=hruh_mdp.hold_default_pose, mode="reset",
                                                params={"asset_cfg": SceneEntityCfg("robot", joint_names=HELD_JOINTS)})
        self.events.hold_upper_body_startup = EventTerm(func=hruh_mdp.hold_default_pose, mode="startup",
                                                        params={"asset_cfg": SceneEntityCfg("robot", joint_names=HELD_JOINTS)})

        # randomization (robustness for sim-to-sim / sim-to-real)
        self.events.physics_material.params["static_friction_range"] = (0.6, 1.2)
        self.events.physics_material.params["dynamic_friction_range"] = (0.5, 1.0)
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_link"
        self.events.add_base_mass.params["mass_distribution_params"] = (-3.0, 3.0)
        self.events.base_com.params["asset_cfg"].body_names = "base_link"
        self.events.base_external_force_torque.params["asset_cfg"].body_names = "base_link"
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {k: (0.0, 0.0) for k in ("x", "y", "z", "roll", "pitch", "yaw")},
        }
        self.events.push_robot.params["velocity_range"] = {"x": (-0.4, 0.4), "y": (-0.4, 0.4)}

        # rewards
        self.rewards.undesired_contacts = None
        self.rewards.flat_orientation_l2.weight = -1.0
        self.rewards.dof_torques_l2.weight = 0.0
        self.rewards.action_rate_l2.weight = -0.005
        self.rewards.dof_acc_l2.weight = -1.25e-7

        # commands: plain (vx, vy, wz) like /cmd_vel
        self.commands.base_velocity.heading_command = False
        self.commands.base_velocity.rel_standing_envs = 0.1
        self.commands.base_velocity.ranges.lin_vel_x = (-0.3, 0.8)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.3, 0.3)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.8, 0.8)

        # terminations: torso / pelvis / head touching the ground, or pelvis too low
        self.terminations.base_contact.params["sensor_cfg"].body_names = ["base_link", "torso_link", "head_base"]
        self.terminations.base_height = DoneTerm(func=mdp.root_height_below_minimum, params={"minimum_height": 0.45})


@configclass
class HruhRoughEnvCfg_PLAY(HruhRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 32
        self.scene.env_spacing = 2.5
        self.episode_length_s = 40.0
        self.scene.terrain.max_init_terrain_level = None
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None


@configclass
class HruhFlatEnvCfg(HruhRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.curriculum.terrain_levels = None
        self.rewards.feet_air_time.weight = 1.0
        self.rewards.feet_air_time.params["threshold"] = 0.6


@configclass
class HruhFlatEnvCfg_PLAY(HruhFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 32
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
