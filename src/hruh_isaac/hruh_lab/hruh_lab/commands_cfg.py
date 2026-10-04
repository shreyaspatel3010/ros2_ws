"""Configuration kept separate so USD implementation imports occur after launch."""
from isaaclab.envs.mdp import UniformVelocityCommandCfg
from isaaclab.managers import CommandTermCfg
from isaaclab.utils import configclass


@configclass
class ControllableVelocityCommandCfg(UniformVelocityCommandCfg):
    class_type: str = "hruh_lab.commands:ControllableVelocityCommand"


@configclass
class MotionVelocityCommandCfg(ControllableVelocityCommandCfg):
    """Explicit movements: stand / forward / backward / side-step / turn / mixed."""
    class_type: str = "hruh_lab.commands:MotionVelocityCommand"
    mode_probabilities: dict = {"stand": 0.15, "forward": 0.25, "backward": 0.10,
                                "side": 0.20, "turn": 0.10, "mixed": 0.20}
    stop_after_moving: float = 0.3   # moving > 0.3 m/s or 0.5 rad/s: chance the next command is 0


@configclass
class ArmMotionCommandCfg(CommandTermCfg):
    """Arms held / swinging with the gait / moving to random poses while walking."""
    class_type: str = "hruh_lab.commands:ArmMotionCommand"
    asset_name: str = "robot"
    resampling_time_range: tuple = (1.5, 4.0)
    mode_probabilities: dict = {"hold": 0.25, "swing": 0.45, "pose": 0.30}
    swing_gain_range: tuple = (0.15, 0.45)   # rad shoulder swing amplitude
    max_rate: float = 2.0                    # rad/s arm target rate limit
