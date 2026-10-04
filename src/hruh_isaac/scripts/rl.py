#!/usr/bin/env python3
"""Register HRUH tasks and run the installed Isaac Lab 3 training/playback CLI."""
import os
import hashlib
import json
from datetime import datetime, timezone
from importlib.metadata import version
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/hruh_isaac/hruh_lab"))
os.environ.setdefault("HRUH_URDF", str(ROOT / "artifacts/hruh/hruh_isaac.urdf"))


def install_tuning():
    """Apply auto_tune.py's overrides (HRUH_TUNING, training only) after every HRUH config
    class builds itself; the most-derived class applies them last (idempotent)."""
    import inspect
    from hruh_lab import tuning
    from hruh_lab.tasks.locomotion import agents as loco_agents, env_cfg as loco_env
    from hruh_lab.tasks.reach import agents as reach_agents, env_cfg as reach_env
    for module, apply in ((loco_env, tuning.apply_env), (reach_env, tuning.apply_env),
                          (loco_agents, tuning.apply_agent), (reach_agents, tuning.apply_agent)):
        for name, cls in inspect.getmembers(module, inspect.isclass):
            if name.startswith("Hruh") and cls.__module__ == module.__name__ and hasattr(cls, "__post_init__"):
                def wrapped(self, _original=cls.__post_init__, _apply=apply):
                    _original(self)
                    _apply(self)
                cls.__post_init__ = wrapped
    print(f"HRUH auto-tuning overrides: {os.environ['HRUH_TUNING']} {json.dumps(tuning.load())}", flush=True)


def main():
    from hruh_lab import gpu_guard   # stay inside HRUH_GPU_MEM_GB (default 8) so the desktop survives
    gpu_guard.cap_torch()
    gpu_guard.start(name=__import__("os").path.basename(__file__))
    import warp as wp
    wp.config.enable_backward = False
    import hruh_lab  # noqa: F401: registers the external gym tasks
    if os.environ.get("HRUH_TUNING"):
        install_tuning()
    from isaaclab_rl.entrypoints import run_train_cli, run_play_cli, run_zero_agent_cli

    if len(sys.argv) < 2 or sys.argv[1] not in ("train", "play", "zero"):
        raise SystemExit("Usage: rl.py {train|play|zero} --task Hruh-Velocity-Flat-v0 [Isaac Lab options]")
    mode, args = sys.argv[1], sys.argv[2:]
    if mode == "train" and not any(arg in ("--help", "-h") for arg in args):
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        snapshot = ROOT / "artifacts/hruh/run_inputs" / stamp
        snapshot.mkdir(parents=True)
        # snapshots contain hruh_lab/setup.py: keep `colcon build` from finding them as packages
        (ROOT / "artifacts/COLCON_IGNORE").touch()
        hashes = {}
        for path in (ROOT / "src/hruh_isaac").rglob("*.py"):
            relative = path.relative_to(ROOT / "src/hruh_isaac")
            target = snapshot / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(path.read_bytes())
            hashes[str(relative)] = hashlib.sha256(target.read_bytes()).hexdigest()
        urdf = Path(os.environ["HRUH_URDF"])
        (snapshot / "manifest.json").write_text(json.dumps({
            "utc": stamp, "arguments": args, "source_sha256": hashes,
            "versions": {name: version(name) for name in ("isaacsim", "isaaclab", "rsl-rl-lib", "torch")},
            "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest() if urdf.exists() else None,
        }, indent=2) + "\n")
        print(f"HRUH source snapshot: {snapshot}", flush=True)
    if mode != "zero":
        args = ["--rl_library", "rsl_rl", *args]
    os.chdir(ROOT)
    return {"train": run_train_cli, "play": run_play_cli, "zero": run_zero_agent_cli}[mode](args)


if __name__ == "__main__":
    raise SystemExit(main())
