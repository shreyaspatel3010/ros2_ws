"""Reject checkpoints whose action or observation meanings differ from playback."""
import json
from pathlib import Path


def signature(config):
    """Extract policy semantics from an Isaac Lab environment config dictionary."""
    policy = config["observations"]["policy"]
    terms = []
    for name, term in policy.items():
        if isinstance(term, dict) and "func" in term:
            asset = term.get("params", {}).get("asset_cfg", {})
            terms.append({"name": name, "func": str(term["func"]), "scale": term.get("scale"),
                          "clip": term.get("clip"), "history_length": term.get("history_length"),
                          "joint_names": asset.get("joint_names"),
                          "preserve_order": asset.get("preserve_order")})
    robot = config["scene"]["robot"]
    result = {
        "dt": config["sim"]["dt"], "decimation": config["decimation"],
        "actions": [{"name": name, **{key: action.get(key) for key in (
            "class_type", "joint_names", "scale", "offset", "clip", "preserve_order", "use_default_offset")}}
            for name, action in config["actions"].items() if action is not None],
        "history": policy.get("history_length"), "flatten_history_dim": policy.get("flatten_history_dim"),
        "terms": terms, "stand_pose": robot["init_state"]["joint_pos"], "actuators": robot["actuators"],
    }
    # Normalize tuple/list differences introduced by YAML serialization.
    return json.loads(json.dumps(result))


def validate_checkpoint(checkpoint, config):
    import yaml

    class ConfigLoader(yaml.SafeLoader):
        pass

    def inert_python_tag(loader, suffix, node):
        # Isaac saves tuples and slices using Python tags. Read them as plain data;
        # never instantiate arbitrary Python objects from checkpoint-side YAML.
        if isinstance(node, yaml.SequenceNode):
            return loader.construct_sequence(node, deep=True)
        if isinstance(node, yaml.MappingNode):
            return loader.construct_mapping(node, deep=True)
        return loader.construct_scalar(node)

    ConfigLoader.add_multi_constructor("tag:yaml.org,2002:python/", inert_python_tag)
    path = Path(checkpoint).resolve().parent / "params/env.yaml"
    if not path.is_file():
        raise ValueError(f"Missing training contract: {path}; keep params/ next to the checkpoint")
    saved = signature(yaml.load(path.read_text(), Loader=ConfigLoader))
    current = signature(config)
    differences = [key for key in saved if saved[key] != current[key]]
    if differences:
        raise ValueError("Checkpoint is incompatible with this environment: " + ", ".join(differences))
