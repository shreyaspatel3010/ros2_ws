#!/usr/bin/env python3
"""Adjust training between rounds from what the last round measured.

    auto_tune.py --skill motion --round 2 --evaluation evaluation_2.json \
                 --train-log train_2.log --checkpoint <run>/model_1999.pt --state-dir <run>/<skill>

Called by train_robot_offline.sh after a round fails its benchmark. It
  1. reads the held-out evaluation, the end of the training log (reward terms,
     exploration noise) and the settings that round actually used (params/env.yaml,
     params/agent.yaml next to the checkpoint),
  2. applies fixed rules - each tied to a measured failure - to reward weights / strengths,
     the movement / arm / push curriculum and PPO exploration, with hard limits, and
     writes them to <state-dir>/tuning.json (read by rl.py through HRUH_TUNING),
  3. keeps the best checkpoint so far (best.json): the next round resumes from it, so a
     worse round never replaces progress, and counts rounds without improvement.

Nothing in the policy contract (observations, actions, actuators) is touched, so the
next round resumes the same network. Rules cannot invent new task code: when every
relevant setting is at its limit the history says so and the script's no-improvement
limit (PATIENCE) ends the skill. History: <state-dir>/tuning_history.jsonl.
System python3 + PyYAML only.
"""
import argparse
import json
from pathlib import Path
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from promote_policy import score as benchmark_score  # noqa: E402

LOCOMOTION = ("motion", "flat", "rough")


def load_yaml(path):
    import yaml

    class Loader(yaml.SafeLoader):
        pass

    def inert(loader, suffix, node):   # Isaac writes tuples / slices with python tags: read as data
        if isinstance(node, yaml.SequenceNode):
            return loader.construct_sequence(node, deep=True)
        if isinstance(node, yaml.MappingNode):
            return loader.construct_mapping(node, deep=True)
        return loader.construct_scalar(node)

    Loader.add_multi_constructor("tag:yaml.org,2002:python/", inert)
    with open(path) as f:
        return yaml.load(f, Loader=Loader)


def last_training_values(log):
    """Values of the last complete iteration block: Episode_Reward/*, Mean action std, ..."""
    text = Path(log).read_text(errors="replace") if Path(log).is_file() else ""
    text = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", text)
    blocks = text.split("Learning iteration")
    values = {}
    for line in (blocks[-2] if len(blocks) > 2 else blocks[-1]).splitlines():
        match = re.match(r"\s*([A-Za-z_/ ]+?):\s*([-+0-9.eE]+|nan|inf)\s*$", line)
        if match:
            try:
                values[match[1].strip()] = float(match[2])
            except ValueError:
                pass
    return values


class Tuner:
    def __init__(self, env, agent, tuning, train):
        self.env, self.agent, self.tuning, self.train = env, agent, tuning, train
        self.changes, self.at_limit = [], []
        self.noise_cut_before = False   # set from the history: entropy was cut for noise last round

    # ---- current values (what the last round trained with)
    def weight(self, name):
        term = (self.env.get("rewards") or {}).get(name)
        return None if not isinstance(term, dict) else float(term.get("weight", 0.0))

    def param(self, name, key):
        term = (self.env.get("rewards") or {}).get(name)
        return None if not isinstance(term, dict) else (term.get("params") or {}).get(key)

    def earned(self, name):
        """Fraction of a reward term's weight earned in training (how often it is active)."""
        w = self.weight(name)
        value = self.train.get(f"Episode_Reward/{name}")
        return None if not w or value is None else value / w

    # ---- changes (absolute values written to tuning.json)
    def _record(self, what, old, new, reason):
        self.changes.append({"setting": what, "from": old, "to": new, "reason": reason})

    def scale_weight(self, name, factor, limit, reason):
        old = self.weight(name)
        if old is None or old == 0:
            return
        new = max(-limit, min(limit, old * factor))
        if abs(new - old) < 1e-9:
            self.at_limit.append(f"rewards.{name}.weight")
            return
        self.tuning.setdefault("rewards", {}).setdefault(name, {})["weight"] = round(new, 6)
        self._record(f"rewards.{name}.weight", old, round(new, 6), reason)

    def scale_param(self, name, key, factor, low, high, reason):
        old = self.param(name, key)
        if old is None:
            return
        new = max(low, min(high, float(old) * factor))
        if abs(new - float(old)) < 1e-9:
            self.at_limit.append(f"rewards.{name}.{key}")
            return
        self.tuning.setdefault("rewards", {}).setdefault(name, {}).setdefault("params", {})[key] = round(new, 6)
        self._record(f"rewards.{name}.params.{key}", old, round(new, 6), reason)

    def shift_mode(self, command, mode, delta, high, reason):
        probs = ((self.env.get("commands") or {}).get(command) or {}).get("mode_probabilities")
        if not probs or mode not in probs:
            return
        old = float(probs[mode])
        new = min(high, old + delta)
        if new - old < 1e-9:
            self.at_limit.append(f"commands.{command}.{mode}")
            return
        self.tuning.setdefault("commands", {}).setdefault(command, {}).setdefault("mode_probabilities", {})[mode] = round(new, 3)
        self._record(f"commands.{command}.mode_probabilities.{mode}", old, round(new, 3),
                     reason + " (probabilities are renormalised)")

    def scale_push_interval(self, factor, floor, reason):
        old = ((self.env.get("events") or {}).get("push_robot") or {}).get("interval_range_s")
        if not old:
            return
        new = [max(floor, round(float(v) * factor, 2)) for v in old]
        if new == [float(v) for v in old]:
            self.at_limit.append("events.push_robot.interval_range_s")
            return
        self.tuning.setdefault("events", {}).setdefault("push_robot", {})["interval_range_s"] = new
        self._record("events.push_robot.interval_range_s", list(old), new, reason)

    def relax_weight(self, name, factor, floor, reason):
        """Reduce a penalty's magnitude (never below `floor`)."""
        old = self.weight(name)
        if old is None or abs(old) <= floor + 1e-9:
            if old is not None:
                self.at_limit.append(f"rewards.{name}.weight")
            return
        new = round(old * factor if abs(old * factor) >= floor else (floor if old > 0 else -floor), 6)
        self.tuning.setdefault("rewards", {}).setdefault(name, {})["weight"] = new
        self._record(f"rewards.{name}.weight", old, new, reason)

    def cap_noise(self, noise, reason):
        """Cap the policy's exploration std (RSL-RL GaussianDistribution std_range; applied by rl.py)."""
        old = self.tuning.get("actor", {}).get("max_std")
        new = round(max(0.3, min(old if old is not None else noise, noise) * 0.75), 3)
        if old is not None and new >= old - 1e-9:
            self.at_limit.append("actor.max_std")
            return
        self.tuning.setdefault("actor", {})["max_std"] = new
        self._record("actor.max_std", old, new, reason)

    def scale_entropy(self, factor, low, high, reason):
        old = float((self.agent.get("algorithm") or {}).get("entropy_coef", 0.0))
        new = max(low, min(high, old * factor))
        if abs(new - old) < 1e-12:
            self.at_limit.append("agent.entropy_coef")
            return
        self.tuning.setdefault("agent", {})["entropy_coef"] = round(new, 6)
        self._record("agent.entropy_coef", old, round(new, 6), reason)

    # ---- rules
    def locomotion(self, evaluation):
        res = {r["scenario"]: r for r in evaluation["results"]}
        mae = lambda n, i: res[n]["velocity_mae_while_alive_vx_vy_wz"][i] if n in res else 0.0  # noqa: E731
        surv = lambda n: res[n]["survival_fraction"] if n in res else 1.0  # noqa: E731
        lin = max(mae("forward", 0), mae("fast", 0), mae("reverse", 0), mae("sidestep", 1), mae("stop_reverse", 0),
                  mae("sudden_stop", 0), mae("sudden_stop", 1),
                  mae("forward_arms", 0), mae("sidestep_arms", 1))
        turn = max(mae("turn", 2), mae("turn_arms", 2))
        noise = self.train.get("Mean action std")
        if lin > 0.15:
            why = f"walking speed error {lin:.2f} m/s > 0.15"
            self.scale_weight("track_lin_vel_xy_exp", 1.2, 4.0, why)
            self.scale_param("track_lin_vel_xy_exp", "std", 0.9, 0.12, 1.0, why)
            stepping = self.earned("feet_air_time")
            if stepping is not None and stepping < 0.1:
                self.scale_weight("feet_air_time", 1.3, 3.0, f"hardly steps in training (earned {stepping:.0%} of the step reward)")
            if mae("sidestep", 1) > 0.15 and mae("forward", 0) <= 0.15:
                self.shift_mode("base_velocity", "side", 0.05, 0.4, f"side-step error {mae('sidestep', 1):.2f} m/s")
        if turn > 0.25:
            why = f"turning error {turn:.2f} rad/s > 0.25"
            self.scale_weight("track_ang_vel_z_exp", 1.2, 3.0, why)
            self.scale_param("track_ang_vel_z_exp", "std", 0.9, 0.12, 1.0, why)
            self.shift_mode("base_velocity", "turn", 0.03, 0.3, why)
        # pelvis yaw wobble: large yaw-rate error in scenarios that do not ask to turn, while the
        # signed mean stays small (an oscillation, not a drift) - standing *and* walking
        steady = [n for n in res if "turn" not in n]
        worst = max(steady, key=lambda n: mae(n, 2), default=None)
        if worst is not None and mae(worst, 2) > 0.25:
            self.scale_weight("yaw_rate_error", 1.3, 2.0,
                              f"pelvis yaw wobble {mae(worst, 2):.2f} rad/s (worst in {worst}; limit 0.25)")
            if worst.endswith("_arms"):   # the arms' reaction turns the pelvis: practise that more
                self.shift_mode("arm_motion", "pose", 0.05, 0.5,
                                f"pelvis yaw wobble is worst while the arms move ({worst})")
                # (relaxing the waist penalty was tried in run auto_499964175a round 5: pelvis yaw got
                #  worse in *every* scenario - a freer torso rocks the pelvis - so it is not a rule)
        if max(mae("stand", 0), mae("stand", 1)) > 0.08:
            self.scale_weight("stand_still", 1.3, 1.5, f"drifts while told to stand ({max(mae('stand', 0), mae('stand', 1)):.2f} m/s)")
        # the robot must never fall: any fall in any scenario is a failure to fix
        normal = [n for n in res if not n.startswith("push_")]
        worst = min((surv(n) for n in normal), default=1.0)
        if worst < 1.0:
            fell = ", ".join(n for n in normal if surv(n) < 1.0)
            self.scale_weight("flat_orientation_l2", 1.25, 5.0, f"falls while moving ({fell}; worst survival {worst:.0%})")
            self.scale_weight("termination_penalty", 1.25, 600.0, f"falls while moving ({fell})")
            arms = [surv(n) for n in normal if n.endswith("_arms")]
            if arms and min(arms) < 1.0 and min(surv(n) for n in normal if not n.endswith("_arms")) >= 1.0:
                self.shift_mode("arm_motion", "pose", 0.05, 0.5, f"falls when the arms move (survival {min(arms):.0%})")
        pushed = [surv(n) for n in res if n.startswith("push_")]
        if pushed and min(pushed) < 1.0:
            self.scale_push_interval(0.8, 4.0, f"falls when pushed (survival {min(pushed):.0%}): push more often in training")
        # sudden stops: must stand still within 2 s without falling
        settle = max((res[n].get("stop_settle_s_max") or 0.0 for n in res), default=0.0)
        stop_falls = min((surv(n) for n in ("stop_reverse", "sudden_stop") if n in res), default=1.0)
        if settle > 2.0 or stop_falls < 1.0:
            why = (f"sudden stops: {settle:.1f} s to stand still (limit 2 s)" if settle > 2.0
                   else f"falls on sudden stops (survival {stop_falls:.0%})")
            self.scale_weight("stand_still", 1.3, 1.5, why)
            self.shift_mode("base_velocity", "stand", 0.05, 0.35, why + ": practise stopping more")
        if noise is not None and noise > 0.8:
            entropy = float((self.agent.get("algorithm") or {}).get("entropy_coef", 0.0))
            if entropy <= 0.0002 + 1e-9:
                # the entropy bonus is already at its floor: cap the noise directly
                self.cap_noise(noise, f"exploration noise {noise:.2f} stays high with the minimum entropy bonus")
            else:
                # halving did not bring the noise down last round: cut harder
                factor = 0.25 if self.noise_cut_before else 0.5
                self.scale_entropy(factor, 0.0002, 0.02, f"exploration noise {noise:.2f} too high"
                                   + (" (still high after the last cut)" if self.noise_cut_before else ""))
        elif noise is not None and noise < 0.2 and lin > 0.15:
            self.scale_entropy(1.5, 0.001, 0.02, f"exploration noise {noise:.2f} collapsed before tracking works")

    def reach(self, evaluation):
        rounds = evaluation.get("rounds") or []
        best = sum(r.get("mean_best_position_error_m", 0.0) for r in rounds) / max(1, len(rounds))
        noise = self.train.get("Mean action std")
        if best > 0.02:
            why = f"hand stops {best * 100:.1f} cm from the target (gate 2.5 cm)"
            self.scale_weight("position_precise", 1.3, 10.0, why)
            self.scale_param("position_precise", "std", 0.85, 0.008, 0.1, why)
        else:
            why = f"gets within {best * 100:.1f} cm but success is {evaluation['success_fraction']:.0%}: settle, do not oscillate"
            self.scale_weight("action_rate", 1.5, 0.01, why)
            self.scale_weight("joint_vel", 1.5, 0.01, why)
        if noise is not None and noise > 0.25:
            self.scale_entropy(0.5, 0.0002, 0.01, f"exploration noise {noise:.2f} too high for 2.5 cm precision")

    def lift(self, evaluation):
        noise = self.train.get("Mean action std")
        touch, grasp, lift = self.earned("touch"), self.earned("grasp_contact"), self.earned("lift")
        if touch is not None and touch < 0.1:
            why = f"rarely touches the cube ({touch:.0%} of the touch reward)"
            self.scale_weight("touch", 1.5, 6.0, why)
            self.scale_weight("reach_object", 1.2, 6.0, why)
            self.scale_weight("close_when_near", 1.3, 3.0, why)
        elif grasp is not None and grasp < 0.1:
            why = f"touches but no thumb + finger grasp ({grasp:.0%} of the grasp reward)"
            self.scale_weight("grasp_contact", 1.5, 8.0, why)
            self.scale_weight("close_when_near", 1.3, 3.0, why)
        elif lift is not None and lift < 0.1:
            self.scale_weight("lift", 1.5, 20.0, f"grasps but does not lift ({lift:.0%} of the lift reward)")
        if noise is not None and noise > 0.8:
            self.scale_entropy(0.5, 0.0005, 0.01, f"exploration noise {noise:.2f}: fingers move randomly")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--skill", required=True, choices=LOCOMOTION + ("reach", "lift"))
    parser.add_argument("--round", type=int, required=True)
    parser.add_argument("--evaluation", type=Path, required=True, help="the evaluation to diagnose (it failed)")
    parser.add_argument("--score-evaluation", type=Path,
                        help="the round's main evaluation (same seed every round) used to rank rounds; "
                             "default --evaluation")
    parser.add_argument("--passes", type=int, default=0,
                        help="evaluations this round passed before failing (confirmations)")
    parser.add_argument("--train-log", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--state-dir", type=Path, required=True)
    args = parser.parse_args()
    state = args.state_dir
    history_path, best_path, tuning_path = state / "tuning_history.jsonl", state / "best.json", state / "tuning.json"
    history = [json.loads(line) for line in history_path.read_text().splitlines() if line.strip()] \
        if history_path.is_file() else []
    done = next((h for h in history if h["round"] == args.round), None)
    if done is None:   # (rerun after Ctrl+C: the round was already tuned - just report it)
        evaluation = json.loads(args.evaluation.read_text())
        kind = "locomotion" if args.skill in LOCOMOTION else args.skill
        # rank rounds fairly: first by passed evaluations, then by the score on the same main evaluation
        # (a worst-of-several confirmation score is not comparable with another round's single evaluation)
        round_score = benchmark_score(kind, json.loads((args.score_evaluation or args.evaluation).read_text()))
        params = args.checkpoint.resolve().parent / "params"
        env = load_yaml(params / "env.yaml") if (params / "env.yaml").is_file() else {}
        agent = load_yaml(params / "agent.yaml") if (params / "agent.yaml").is_file() else {}
        tuning = json.loads(tuning_path.read_text()) if tuning_path.is_file() else {}
        tuner = Tuner(env, agent, tuning, last_training_values(args.train_log))
        tuner.noise_cut_before = bool(history) and any(
            c["setting"] == "agent.entropy_coef" and "noise" in c["reason"] for c in history[-1]["changes"])
        if not evaluation.get("passed_benchmark"):
            {"locomotion": tuner.locomotion, "reach": tuner.reach, "lift": tuner.lift}[kind](evaluation)
        best = json.loads(best_path.read_text()) if best_path.is_file() else {"score": None, "since_improvement": 0}
        if best["score"] is None or (args.passes, round_score) > (best.get("passes", 0), best["score"] + 1e-6):
            best = {"score": round_score, "passes": args.passes, "round": args.round,
                    "checkpoint": str(args.checkpoint.resolve()), "since_improvement": 0}
        else:
            best["since_improvement"] += 1
        done = {"round": args.round, "score": round_score, "passes": args.passes,
                "best_score": best["score"], "best_round": best["round"],
                "since_improvement": best["since_improvement"], "changes": tuner.changes,
                "at_limit": sorted(set(tuner.at_limit))}
        state.mkdir(parents=True, exist_ok=True)
        tuning_path.write_text(json.dumps(tuner.tuning, indent=2) + "\n")
        best_path.write_text(json.dumps(best, indent=2) + "\n")
        with history_path.open("a") as f:
            f.write(json.dumps(done) + "\n")
    if done["since_improvement"] == 0:
        verdict = "new best"
    else:
        verdict = (f"best is round {done['best_round']} ({done['best_score']:.3f}), "
                   f"{done['since_improvement']} round(s) without improvement; next round resumes from the best")
    print(f"Round {done['round']} score {done['score']:.3f}: {verdict}")
    for change in done["changes"]:
        print(f"  auto-tune: {change['setting']} {change['from']} -> {change['to']}  ({change['reason']})")
    if not done["changes"]:
        print("  auto-tune: no rule applies (or all limits reached): training continues with the same settings")
    if done["at_limit"]:
        print(f"  at their limits: {', '.join(done['at_limit'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
