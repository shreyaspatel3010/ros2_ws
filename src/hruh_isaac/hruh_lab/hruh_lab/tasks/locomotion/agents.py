from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlMLPModelCfg, RslRlPpoAlgorithmCfg


@configclass
class HruhRoughPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 3000
    save_interval = 100
    experiment_name = "hruh_rough"
    # raw actions in [-2, 2] -> +-1 rad around the stand pose (scale 0.5), then clipped to joint limits;
    # +-0.5 rad capped the knee near 0.86 rad and limited stepping
    clip_actions = 2.0
    obs_groups = {"actor": ["policy"], "critic": ["critic"]}
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 128],
        obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.5),
        activation="elu",
    )
    critic = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 128],
        obs_normalization=True,
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.01,
        num_learning_epochs=5, num_mini_batches=4, learning_rate=1.0e-3, schedule="adaptive",
        gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
    )


@configclass
class HruhFlatPPORunnerCfg(HruhRoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()
        self.max_iterations = 1500
        self.experiment_name = "hruh_flat"


@configclass
class HruhMotionPPORunnerCfg(HruhFlatPPORunnerCfg):
    """Walking / side-stepping / stopping with moving arms (more varied: train longer)."""
    def __post_init__(self):
        super().__post_init__()
        self.max_iterations = 3000
        self.experiment_name = "hruh_motion"
