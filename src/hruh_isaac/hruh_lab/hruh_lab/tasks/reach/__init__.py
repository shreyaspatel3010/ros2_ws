"""Fixed-pelvis manipulation prerequisites for the actual HRUH arms and hands."""
import gymnasium as gym

for name in ("Reach", "Lift"):
    gym.register(
        id=f"Hruh-{name}-Right-v0", entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={"env_cfg_entry_point": f"{__name__}.env_cfg:Hruh{name}EnvCfg",
                "rsl_rl_cfg_entry_point": f"{__name__}.agents:Hruh{name}PPORunnerCfg"},
    )
