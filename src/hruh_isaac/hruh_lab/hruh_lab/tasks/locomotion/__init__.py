"""Velocity-tracking locomotion: Hruh-Velocity-{Flat,Rough,Motion}[-Play]-v0."""
import gymnasium as gym

for terrain in ("Flat", "Rough", "Motion"):
    for play in ("", "_PLAY"):
        gym.register(
            id=f"Hruh-Velocity-{terrain}{'-Play' if play else ''}-v0",
            entry_point="isaaclab.envs:ManagerBasedRLEnv",
            disable_env_checker=True,
            kwargs={
                "env_cfg_entry_point": f"{__name__}.env_cfg:Hruh{terrain}EnvCfg{play}",
                "rsl_rl_cfg_entry_point": f"{__name__}.agents:Hruh{terrain}PPORunnerCfg",
            },
        )
