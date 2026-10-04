from isaaclab.utils import configclass
from ..locomotion.agents import HruhFlatPPORunnerCfg


@configclass
class HruhReachPPORunnerCfg(HruhFlatPPORunnerCfg):
    experiment_name = "hruh_reach_right"
    obs_groups = {"actor": ["policy"], "critic": ["policy"]}

    def __post_init__(self):
        super().__post_init__()
        self.experiment_name = "hruh_reach_right"
        self.max_iterations = 1000
        # one arm, no pelvis velocity: plain PPO + MLP (no left-right symmetry, no estimator)
        self.actor.class_name = "MLPModel"
        self.algorithm.class_name = "PPO"
        self.algorithm.symmetry_cfg = None
        # Precise arm control: the walking setting (entropy 0.01) kept the action noise at ~0.6
        # (+-0.4 rad per joint), so the hand never settled inside 2.5 cm.
        self.algorithm.entropy_coef = 0.001
        self.actor.distribution_cfg.init_std = 0.3


@configclass
class HruhLiftPPORunnerCfg(HruhReachPPORunnerCfg):
    init_at_random_ep_len = False
    def __post_init__(self):
        super().__post_init__()
        self.experiment_name = "hruh_lift_right"
        self.max_iterations = 2500
        # the hand needs some exploration to find a grasp, but the noise must not grow
        # (it reached 1.15 with entropy 0.01 and the fingers moved randomly)
        self.algorithm.entropy_coef = 0.002
        self.actor.distribution_cfg.init_std = 0.5
