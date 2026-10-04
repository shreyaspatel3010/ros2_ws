"""Training-time overrides written by scripts/auto_tune.py between training rounds.

train_robot_offline.sh sets HRUH_TUNING=<run>/<skill>/tuning.json for the *training*
stage only, so evaluation always measures the task as defined in the source. Only
values that keep a checkpoint resumable are tuned: reward weights / parameters,
command and push curricula, and PPO exploration. Observations, actions and actuators
(the policy contract) are never changed.

    {"rewards":  {"track_lin_vel_xy_exp": {"weight": 1.8, "params": {"std": 0.22}}},
     "commands": {"base_velocity": {"mode_probabilities": {"side": 0.25}}},
     "events":   {"push_robot": {"interval_range_s": [8.0, 12.0]}},
     "agent":    {"entropy_coef": 0.005}}

Pure Python apart from the config objects it is given.
"""
import json
import os


def load():
    path = os.environ.get("HRUH_TUNING", "")
    if not path or not os.path.isfile(path):
        return {}
    with open(path) as f:
        return json.load(f)


def _update(obj, changes):
    for key, value in changes.items():
        current = getattr(obj, key, None)
        if isinstance(value, dict) and isinstance(current, dict):
            current.update(value)
        else:
            setattr(obj, key, tuple(value) if isinstance(value, list) else value)


def apply_env(cfg):
    """Apply reward / command / event overrides to an Isaac Lab env cfg (idempotent)."""
    tuning = load()
    for name, change in tuning.get("rewards", {}).items():
        term = getattr(cfg.rewards, name, None)
        if term is None:
            continue
        if "weight" in change:
            term.weight = float(change["weight"])
        term.params.update(change.get("params", {}))
    for group in ("commands", "events"):
        for name, change in tuning.get(group, {}).items():
            term = getattr(getattr(cfg, group, None), name, None)
            if term is not None:
                _update(term, change)
    return cfg


def apply_agent(agent):
    """Apply PPO overrides (exploration) to an RSL-RL runner cfg (idempotent)."""
    for key, value in load().get("agent", {}).items():
        if hasattr(agent.algorithm, key):
            setattr(agent.algorithm, key, value)
    return agent
