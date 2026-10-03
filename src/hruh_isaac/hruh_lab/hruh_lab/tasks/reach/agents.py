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


@configclass
class HruhLiftPPORunnerCfg(HruhReachPPORunnerCfg):
    init_at_random_ep_len = False
    def __post_init__(self):
        super().__post_init__()
        self.experiment_name = "hruh_lift_right"
        self.max_iterations = 2500
