"""Position reaching and contact-based cube lifting with HRUH's right hand.

The pelvis is fixed for these prerequisite skills. Object state is simulator
ground truth; vision and simultaneous walking/manipulation are separate work.
"""
import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm, ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm, TerminationTermCfg as DoneTerm, SceneEntityCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass
from isaaclab_physx.physics import PhysxCfg
from isaaclab_tasks.core.reach.reach_env_cfg import ReachEnvCfg

from ...joints import ARM_JOINTS, STAND_POSE
from ...robots import HRUH_FIXED_CFG, JOINT_LIMITS
from ...mdp import hold_default_pose

ARM = ARM_JOINTS["right"]
HAND = ["right_palm_to_thomb", "right_thomb_to_thomb_middle", "right_thomb_middle_to_thomb_upper"] + [
    f"right_palm_to_finger{i}_lower" for i in range(1, 5)]
# Each distal segment mimics its base 1:1 but stops at 1.20 rad in the URDF.
HAND_LIMITS = {n: (JOINT_LIMITS[n][0], min(JOINT_LIMITS[n][1], 1.2))
               if "finger" in n else JOINT_LIMITS[n] for n in HAND}
START_POSE = {**STAND_POSE, "chest_to_right_shoulder": 0.6,
              "right_shoulder_to_bisecp": 0.3}
# Replace the shared elbow regex with explicit entries to avoid overlapping patterns.
START_POSE.pop(".*_elbow_inword_to_midle")
START_POSE.update(right_elbow_inword_to_midle=0.8, left_elbow_inword_to_midle=0.3)


@configclass
class HruhReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.sim.physics = PhysxCfg()
        self.sim.dt = 0.005
        self.decimation = 4
        self.sim.render_interval = 4
        self.scene.num_envs = 256
        self.scene.robot = HRUH_FIXED_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.init_state.joint_pos = START_POSE.copy()
        self.scene.ground.init_state.pos = (0.0, 0.0, 0.0)
        self.scene.table = None
        self.episode_length_s = 6.0
        # Position task: orientation is unconstrained with this five-axis arm.
        self.commands.ee_pose.body_name = "right_wrist"
        self.commands.ee_pose.debug_vis = False
        self.commands.ee_pose.resampling_time_range = (8.0, 8.0)
        # Target box in the pelvis frame. Forward-kinematics sampling over the joint limits
        # puts 99.8% of it within 1.5 cm of a reachable wrist position; the previous box
        # (x 0.25-0.40, y -0.38..-0.23, z 0.12-0.30) was only 86% reachable, so the 90%
        # success gate could not be met by any policy.
        self.commands.ee_pose.ranges.pos_x = (0.22, 0.34)
        self.commands.ee_pose.ranges.pos_y = (-0.36, -0.22)
        self.commands.ee_pose.ranges.pos_z = (0.16, 0.32)
        self.commands.ee_pose.ranges.pitch = (0.0, 0.0)
        self.commands.ee_pose.ranges.yaw = (0.0, 0.0)
        self.commands.ee_pose.position_success_threshold = 0.025
        self.commands.ee_pose.orientation_success_threshold = 10.0
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=ARM, preserve_order=True, scale=0.7,
            clip={n: JOINT_LIMITS[n] for n in ARM}, use_default_offset=True)
        for term in (self.observations.policy.joint_pos, self.observations.policy.joint_vel):
            term.params = {"asset_cfg": SceneEntityCfg("robot", joint_names=ARM, preserve_order=True)}
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["right_wrist"]
        self.rewards.end_effector_position_tracking.weight = -4.0
        self.rewards.position_fine = RewTerm(
            func=mdp.position_command_error_tanh, weight=1.0,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=["right_wrist"]),
                    "command_name": "ee_pose", "std": 0.08})
        # last-centimetre precision: tanh(d / 0.08) is almost flat between 2 and 3 cm, where
        # the previous policies stalled (mean best error ~3 cm, gate 2.5 cm)
        self.rewards.position_precise = RewTerm(
            func=mdp.position_command_error_tanh, weight=2.0,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=["right_wrist"]),
                    "command_name": "ee_pose", "std": 0.02})
        self.rewards.end_effector_orientation_tracking = None
        self.rewards.joint_vel.params["asset_cfg"].joint_names = ARM
        self.events.reset_robot_joints.params["position_range"] = (0.9, 1.1)
        self.events.hold = EventTerm(
            func=hold_default_pose, mode="reset",
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[n for n in JOINT_LIMITS if n not in ARM])})
        self.curriculum = None


@configclass
class HruhLiftEnvCfg(HruhReachEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # Small finger bodies and a light object need a finer contact solve.
        self.sim.dt = 0.0025
        self.decimation = 8
        self.sim.render_interval = 8
        self.scene.robot.spawn.articulation_props.solver_position_iteration_count = 8
        self.scene.robot.spawn.articulation_props.solver_velocity_iteration_count = 4
        self.episode_length_s = 8.0
        joints = ARM + HAND
        # Mimic finger segments follow their base joints; they must not fight independent drives.
        hands = self.scene.robot.actuators["hands"]
        hands.stiffness = {".*_thomb.*": 5.0, ".*_palm_to_finger.*": 5.0,
                           ".*_finger[1-4]_lower_to_.*": 0.0, ".*_finger[1-4]_middle_to_.*": 0.0}
        hands.damping = {".*_thomb.*": 0.2, ".*_palm_to_finger.*": 0.2,
                         ".*_finger[1-4]_lower_to_.*": 0.0, ".*_finger[1-4]_middle_to_.*": 0.0}
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=joints, preserve_order=True,
            class_type="hruh_lab.actions:RateLimitedJointPositionAction",
            scale={**{n: 0.35 for n in ARM}, **{n: (HAND_LIMITS[n][1] - HAND_LIMITS[n][0]) / 2 for n in HAND}},
            offset={**{n: START_POSE.get(n, 0.0) for n in ARM},
                    **{n: sum(HAND_LIMITS[n]) / 2 for n in HAND}},
            use_default_offset=False, clip={**{n: JOINT_LIMITS[n] for n in ARM}, **HAND_LIMITS})
        for term in (self.observations.policy.joint_pos, self.observations.policy.joint_vel):
            term.params = {"asset_cfg": SceneEntityCfg("robot", joint_names=joints, preserve_order=True)}
        self.commands = None
        self.observations.policy.pose_command = None
        self.observations.policy.object_state = ObsTerm(func="hruh_lab.tasks.reach.mdp:object_state")
        self.events.hold.params["asset_cfg"].joint_names = [n for n in JOINT_LIMITS if n not in joints]
        self.scene.table = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Table", init_state=AssetBaseCfg.InitialStateCfg(pos=(0.55, -0.3, 1.09)),
            spawn=sim_utils.CuboidCfg(size=(0.4, 0.45, 0.10),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.35))))
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object", init_state=RigidObjectCfg.InitialStateCfg(pos=(0.415, -0.29, 1.165)),
            spawn=sim_utils.CuboidCfg(size=(0.045, 0.045, 0.045),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    max_depenetration_velocity=0.5, max_linear_velocity=10.0, max_angular_velocity=20.0,
                    solver_position_iteration_count=8, solver_velocity_iteration_count=4),
                mass_props=sim_utils.MassPropertiesCfg(mass=0.08),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=0.8),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.25, 0.08))))
        self.events.object_reset = EventTerm(
            func=mdp.reset_root_state_uniform, mode="reset",
            params={"asset_cfg": SceneEntityCfg("object"),
                    "pose_range": {"x": (-0.02, 0.02), "y": (-0.02, 0.02), "yaw": (-0.3, 0.3)},
                    "velocity_range": {}})
        self.events.object_mass = EventTerm(
            func=mdp.randomize_rigid_body_mass, mode="startup",
            params={"asset_cfg": SceneEntityCfg("object"), "mass_distribution_params": (0.75, 1.25),
                    "operation": "scale", "distribution": "uniform"})
        for name, body in [("thumb", "right_thomb_upper")] + [(f"finger{i}", f"right_finger{i}_upper") for i in range(1, 5)]:
            setattr(self.scene, name, ContactSensorCfg(
                prim_path="{ENV_REGEX_NS}/Robot/" + body, update_period=0.0,
                filter_prim_paths_expr=["{ENV_REGEX_NS}/Object"]))
        self.rewards.end_effector_position_tracking = None
        self.rewards.position_fine = None
        self.rewards.position_precise = None
        self.rewards.reach_object = RewTerm(func="hruh_lab.tasks.reach.mdp:reach_object", weight=2.0)
        # stepping stones to a grasp (the previous run never touched the cube with thumb + finger):
        # touch it at all -> close the fingers around it -> opposed thumb / finger contact -> lift
        self.rewards.touch = RewTerm(func="hruh_lab.tasks.reach.mdp:touch", weight=1.0)
        self.rewards.close_when_near = RewTerm(func="hruh_lab.tasks.reach.mdp:close_when_near", weight=0.5)
        self.rewards.grasp_contact = RewTerm(func="hruh_lab.tasks.reach.mdp:grasp_contact", weight=2.0)
        self.rewards.lift = RewTerm(func="hruh_lab.tasks.reach.mdp:lift_height", weight=5.0)
        self.terminations.success = DoneTerm(func="hruh_lab.tasks.reach.mdp:SustainedGrasp")
        self.terminations.dropped = DoneTerm(func=mdp.root_height_below_minimum,
            params={"asset_cfg": SceneEntityCfg("object"), "minimum_height": 1.0})
