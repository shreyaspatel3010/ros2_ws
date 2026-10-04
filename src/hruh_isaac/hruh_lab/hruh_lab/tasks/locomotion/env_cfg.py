"""HRUH velocity-tracking locomotion (adapted from Isaac Lab's H1 recipe).

Policy: legs + waist (17 joints, position targets around the standing pose).
Observations use only what the real robot / ROS runner has: pelvis IMU
(angular velocity, gravity direction), /cmd_vel, joint encoders, last action -
no ground-truth base velocity, no height scan on flat ground.
"""
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

import isaaclab_tasks.core.velocity.mdp as mdp
from isaaclab_tasks.core.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
    RewardsCfg,
)

from ... import mdp as hruh_mdp
from ...commands_cfg import ArmMotionCommandCfg, ControllableVelocityCommandCfg, MotionVelocityCommandCfg
from ...joints import ARM_JOINTS, HEAD_JOINTS, HAND_JOINT_REGEX, LEG_JOINTS, WAIST_JOINTS
from ...robots import HRUH_CFG, JOINT_LIMITS

POLICY_JOINTS = LEG_JOINTS + WAIST_JOINTS
HELD_JOINTS = ARM_JOINTS["left"] + ARM_JOINTS["right"] + HEAD_JOINTS + HAND_JOINT_REGEX
FEET = ".*_foot_link"


@configclass
class HruhRewards(RewardsCfg):
    # small: a large alive bonus made "survive while shuffling" a local optimum
    alive = RewTerm(func=mdp.is_alive, weight=0.25)
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    lin_vel_z_l2 = None
    # std 0.25, not H1's 0.5: HRUH's commands are slow (<= 0.4 m/s). With 0.5, standing still
    # against a 0.3 m/s command still earned 70% of this reward, and offline run auto_5ae95cbb56
    # learned to stand and shuffle (forward error 0.28 of 0.30 m/s, turning 0.1 of 0.4 rad/s).
    track_lin_vel_xy_exp = RewTerm(func=mdp.track_lin_vel_xy_yaw_frame_exp, weight=1.5,
                                   params={"command_name": "base_velocity", "std": 0.25})
    track_ang_vel_z_exp = RewTerm(func=mdp.track_ang_vel_z_world_exp, weight=1.0,
                                  params={"command_name": "base_velocity", "std": 0.25})
    # real steps (that run earned 0.001 here: it never lifted a foot long enough); slow
    # gaits have ~0.35 s swing phases
    feet_air_time = RewTerm(func=mdp.feet_air_time_positive_biped, weight=1.0,
                            params={"command_name": "base_velocity", "threshold": 0.35,
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
    # evaluation of balance_v1 showed ~0.66 rad/s pelvis yaw wobble with near-zero mean
    yaw_rate_error = RewTerm(func=hruh_mdp.yaw_rate_error_l2, weight=-0.3,
                             params={"command_name": "base_velocity"})
    joint_deviation_toes = RewTerm(func=mdp.joint_deviation_l1, weight=-0.05,
                                   params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*_toe_joint")})


@configclass
class EstimatorTargetCfg(ObsGroup):
    base_lin_vel = ObsTerm(func=mdp.base_lin_vel)

    def __post_init__(self):
        self.enable_corruption = False
        self.concatenate_terms = True


@configclass
class HruhRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: HruhRewards = HruhRewards()

    def __post_init__(self):
        super().__post_init__()
        # This robot uses Isaac Sim PhysX, including URDF conversion and implicit drives.
        from isaaclab_physx.physics import PhysxCfg
        self.sim.physics = PhysxCfg()
        self.scene.num_envs = 256
        self.scene.sky_light.spawn.texture_file = None
        self.scene.terrain.visual_material = None
        # scene
        self.scene.robot = HRUH_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        if self.scene.height_scanner:
            self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/base_link"

        # actions: legs + waist only
        self.actions.joint_pos.joint_names = POLICY_JOINTS
        self.actions.joint_pos.scale = 0.5
        self.actions.joint_pos.preserve_order = True
        self.actions.joint_pos.clip = {name: JOINT_LIMITS[name] for name in POLICY_JOINTS}

        # observations: deployable set (IMU + encoders + command)
        self.observations.policy.base_lin_vel = None
        self.observations.policy.joint_pos.params = {"asset_cfg": SceneEntityCfg("robot", joint_names=POLICY_JOINTS, preserve_order=True)}
        self.observations.policy.joint_vel.params = {"asset_cfg": SceneEntityCfg("robot", joint_names=POLICY_JOINTS, preserve_order=True)}
        # Same proprioceptive interface on flat and rough terrain; history lets the
        # actor infer motion and disturbances without ground-truth base velocity.
        self.observations.policy.height_scan = None
        self.observations.policy.history_length = 5
        self.scene.height_scanner = None
        self.observations.critic = self.observations.policy.copy()
        self.observations.critic.history_length = 1
        self.observations.critic.enable_corruption = False
        self.observations.critic.base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        # supervised target of the concurrent velocity estimator (hruh_lab/estimator.py):
        # simulator ground truth, used for training only, never by the deployed policy
        self.observations.estimator_target = EstimatorTargetCfg()

        # keep arms / head / hands in the standing pose (no action drives them)
        self.events.hold_upper_body = EventTerm(func=hruh_mdp.hold_default_pose, mode="reset",
                                                params={"asset_cfg": SceneEntityCfg("robot", joint_names=HELD_JOINTS)})
        self.events.hold_upper_body_startup = EventTerm(func=hruh_mdp.hold_default_pose, mode="startup",
                                                        params={"asset_cfg": SceneEntityCfg("robot", joint_names=HELD_JOINTS)})

        # randomization (robustness for sim-to-sim / sim-to-real)
        self.events.physics_material.params["static_friction_range"] = (0.6, 1.2)
        self.events.physics_material.params["dynamic_friction_range"] = (0.5, 1.0)
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_link"
        self.events.add_base_mass.params["mass_distribution_params"] = (0.85, 1.15)
        self.events.base_com = EventTerm(
            func=mdp.randomize_rigid_body_com, mode="startup",
            params={"asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
                    "com_range": {"x": (-0.02, 0.02), "y": (-0.02, 0.02), "z": (-0.01, 0.01)}})
        self.events.base_external_force_torque.params["asset_cfg"].body_names = "base_link"
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {k: (0.0, 0.0) for k in ("x", "y", "z", "roll", "pitch", "yaw")},
        }
        self.events.push_robot.params["velocity_range"] = {"x": (-0.4, 0.4), "y": (-0.4, 0.4)}
        self.events.actuator_gains = EventTerm(
            func=mdp.randomize_actuator_gains, mode="startup",
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=POLICY_JOINTS),
                    "stiffness_distribution_params": (0.85, 1.15),
                    "damping_distribution_params": (0.85, 1.15),
                    "operation": "scale", "distribution": "uniform"})

        # rewards
        self.rewards.undesired_contacts = None
        self.rewards.flat_orientation_l2.weight = -1.0
        self.rewards.dof_torques_l2.weight = 0.0
        self.rewards.action_rate_l2.weight = -0.005
        self.rewards.dof_acc_l2.weight = -1.25e-7

        # commands: plain (vx, vy, wz) like /cmd_vel
        previous = self.commands.base_velocity
        self.commands.base_velocity = ControllableVelocityCommandCfg(
            asset_name="robot", resampling_time_range=previous.resampling_time_range,
            ranges=previous.ranges)
        self.commands.base_velocity.heading_command = False
        self.commands.base_velocity.ranges.heading = None
        self.commands.base_velocity.debug_vis = False
        self.commands.base_velocity.resampling_time_range = (3.0, 7.0)
        self.commands.base_velocity.rel_standing_envs = 0.1
        # at least 0.4 m/s in every direction, faster forward: training beyond the speeds used
        # day to day gives better control at all speeds
        self.commands.base_velocity.ranges.lin_vel_x = (-0.4, 0.8)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.4, 0.4)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)

        # terminations: torso / pelvis / head touching the ground, or pelvis too low
        self.terminations.base_contact.params["sensor_cfg"].body_names = ["base_link", "torso_link", "head_base"]
        self.terminations.tilt = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 1.0})
        # World height is unsuitable on elevated terrain. Contact and tilt remain active.
        self.terminations.base_height = None
        terrain = self.scene.terrain.terrain_generator
        terrain.num_rows, terrain.num_cols = 5, 6
        self.scene.terrain.max_init_terrain_level = 1
        for name in ("pyramid_stairs", "pyramid_stairs_inv"):
            terrain.sub_terrains[name].step_height_range = (0.02, 0.10)
        terrain.sub_terrains["boxes"].grid_height_range = (0.02, 0.08)
        terrain.sub_terrains["random_rough"].noise_range = (0.01, 0.04)
        terrain.sub_terrains["random_rough"].noise_step = 0.01
        for name in ("hf_pyramid_slope", "hf_pyramid_slope_inv"):
            terrain.sub_terrains[name].slope_range = (0.0, 0.2)


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
        self.terminations.base_height = DoneTerm(func=mdp.root_height_below_minimum, params={"minimum_height": 0.45})


@configclass
class HruhFlatEnvCfg_PLAY(HruhFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 32
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None


ARMS = ARM_JOINTS["left"] + ARM_JOINTS["right"]
OBSERVED_JOINTS = POLICY_JOINTS + ARMS


@configclass
class HruhMotionEnvCfg(HruhFlatEnvCfg):
    """Whole-body movement: walk forward / backward, side-step, turn, stop and start,
    while the arms hold, swing with the gait or move to random poses.

    The policy still drives legs + waist (17 actions); it additionally observes the
    10 arm joints (positions + velocities) so it can balance whatever the arms do -
    MoveIt, the gamepad or the counter-swing that policy_runner.py adds while walking."""

    def __post_init__(self):
        super().__post_init__()
        # explicit movements with frequent starts / stops / direction changes
        previous = self.commands.base_velocity
        self.commands.base_velocity = MotionVelocityCommandCfg(
            asset_name="robot", resampling_time_range=(2.0, 5.0), ranges=previous.ranges,
            heading_command=False, rel_standing_envs=0.0, debug_vis=False)
        self.commands.base_velocity.ranges.heading = None
        # arms: hold / natural swing / random poses, rate limited
        self.commands.arm_motion = ArmMotionCommandCfg()
        # observe the arms (their motion shifts the centre of mass)
        for group in (self.observations.policy, self.observations.critic):
            group.joint_pos.params = {"asset_cfg": SceneEntityCfg("robot", joint_names=OBSERVED_JOINTS,
                                                                  preserve_order=True)}
            group.joint_vel.params = {"asset_cfg": SceneEntityCfg("robot", joint_names=OBSERVED_JOINTS,
                                                                  preserve_order=True)}
        # stop cleanly: legs return to the stand pose when the command is zero
        self.rewards.stand_still = RewTerm(func=mdp.stand_still_joint_deviation_l1, weight=-0.2,
                                           params={"command_name": "base_velocity",
                                                   "asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)})
        self.episode_length_s = 20.0


@configclass
class HruhMotionEnvCfg_PLAY(HruhMotionEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 32
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
