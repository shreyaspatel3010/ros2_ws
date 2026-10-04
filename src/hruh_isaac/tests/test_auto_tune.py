import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE / "scripts"))
spec = importlib.util.spec_from_file_location("auto_tune", HERE / "scripts/auto_tune.py")
auto_tune = importlib.util.module_from_spec(spec)
spec.loader.exec_module(auto_tune)

ENV_YAML = """
rewards:
  track_lin_vel_xy_exp: {weight: 1.5, params: {command_name: base_velocity, std: 0.25}}
  track_ang_vel_z_exp: {weight: 1.0, params: {command_name: base_velocity, std: 0.25}}
  feet_air_time: {weight: 1.0, params: {threshold: 0.35}}
  yaw_rate_error: {weight: -0.3, params: {}}
  stand_still: {weight: -0.2, params: {}}
  flat_orientation_l2: {weight: -1.0, params: {}}
  termination_penalty: {weight: -200.0, params: {}}
  lin_vel_z_l2: null
commands:
  base_velocity: {mode_probabilities: {stand: 0.15, forward: 0.25, backward: 0.1, side: 0.2, turn: 0.1, mixed: 0.2}}
  arm_motion: {mode_probabilities: {hold: 0.25, swing: 0.45, pose: 0.3}}
events:
  push_robot: {interval_range_s: !!python/tuple [10.0, 15.0]}
"""
TRAIN_LOG = """Learning iteration 998/1000
  Mean action std: 0.63
  Episode_Reward/feet_air_time: 0.0012
Learning iteration 999/1000
  Mean action std: 0.63
  Episode_Reward/feet_air_time: 0.0012
  ETA: 0:00:00
"""


def scenario(name, surv, vx, vy, wz):
    return {"scenario": name, "survival_fraction": surv, "velocity_mae_while_alive_vx_vy_wz": [vx, vy, wz]}


# the failure pattern of offline run auto_5ae95cbb56: stands and shuffles, weak pushes
SHUFFLE = {"passed_benchmark": False, "results": [
    scenario("stand", 1.0, 0.116, 0.06, 0.283), scenario("forward", 1.0, 0.289, 0.045, 0.251),
    scenario("reverse", 1.0, 0.172, 0.066, 0.305), scenario("sidestep", 1.0, 0.117, 0.175, 0.306),
    scenario("turn", 1.0, 0.107, 0.076, 0.372), scenario("push_stand", 0.69, 0.13, 0.07, 0.28),
    scenario("forward_arms", 0.94, 0.281, 0.051, 0.357)]}


class AutoTuneTest(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())
        run = self.dir / "run"
        (run / "params").mkdir(parents=True)
        (run / "params/env.yaml").write_text(ENV_YAML)
        (run / "params/agent.yaml").write_text("algorithm: {entropy_coef: 0.01}\n")
        self.checkpoint = run / "model_999.pt"
        self.checkpoint.write_bytes(b"x")
        (self.dir / "train.log").write_text(TRAIN_LOG)
        self.state = self.dir / "state"

    def tune(self, evaluation, round_=1, checkpoint=None):
        path = self.dir / f"eval_{round_}.json"
        path.write_text(json.dumps(evaluation))
        sys.argv = ["auto_tune.py", "--skill", "motion", "--round", str(round_), "--evaluation", str(path),
                    "--train-log", str(self.dir / "train.log"), "--checkpoint", str(checkpoint or self.checkpoint),
                    "--state-dir", str(self.state)]
        auto_tune.main()
        return json.loads((self.state / "tuning.json").read_text())

    def test_shuffling_policy_gets_stronger_tracking_and_stepping(self):
        t = self.tune(SHUFFLE)
        r = t["rewards"]
        self.assertAlmostEqual(r["track_lin_vel_xy_exp"]["weight"], 1.8)
        self.assertAlmostEqual(r["track_lin_vel_xy_exp"]["params"]["std"], 0.225)
        self.assertAlmostEqual(r["feet_air_time"]["weight"], 1.3)            # hardly stepped
        self.assertAlmostEqual(r["track_ang_vel_z_exp"]["weight"], 1.2)      # turn error 0.37
        self.assertAlmostEqual(r["yaw_rate_error"]["weight"], -0.39)         # stand wobble 0.28
        self.assertAlmostEqual(r["stand_still"]["weight"], -0.26)            # stand drift 0.116
        self.assertAlmostEqual(r["flat_orientation_l2"]["weight"], -1.25)    # forward_arms 94%
        self.assertEqual(t["events"]["push_robot"]["interval_range_s"], [8.0, 12.0])
        self.assertNotIn("agent", t)                                          # noise 0.63 is fine
        self.assertAlmostEqual(t["commands"]["base_velocity"]["mode_probabilities"]["turn"], 0.13)

    def test_limits_and_best_checkpoint(self):
        tuned = self.tune(SHUFFLE, 1)
        self.assertEqual(json.loads((self.state / "best.json").read_text())["round"], 1)
        # a worse round: best stays at round 1 and the no-improvement count rises
        worse = json.loads(json.dumps(SHUFFLE))
        worse["results"][1]["survival_fraction"] = 0.5
        self.tune(worse, 2)
        best = json.loads((self.state / "best.json").read_text())
        self.assertEqual((best["round"], best["since_improvement"]), (1, 1))
        # rerunning an already-tuned round changes nothing (resume after Ctrl+C)
        before = (self.state / "tuning_history.jsonl").read_text()
        self.tune(worse, 2)
        self.assertEqual((self.state / "tuning_history.jsonl").read_text(), before)
        self.assertGreaterEqual(tuned["rewards"]["track_lin_vel_xy_exp"]["params"]["std"], 0.12)

    def test_any_fall_and_slow_stops_are_fixed(self):
        ok = [scenario(n, 1.0, 0.05, 0.05, 0.1) for n in ("stand", "forward", "turn", "push_walk")]
        slow_stop = dict(scenario("sudden_stop", 1.0, 0.05, 0.05, 0.1), stop_settle_s_max=2.4)
        t = self.tune({"passed_benchmark": False, "results": ok + [slow_stop]})
        self.assertAlmostEqual(t["rewards"]["stand_still"]["weight"], -0.26)
        self.assertAlmostEqual(t["commands"]["base_velocity"]["mode_probabilities"]["stand"], 0.2)
        self.assertNotIn("termination_penalty", t["rewards"])                  # no fall here
        one_fall = ok + [scenario("fast", 31 / 32, 0.05, 0.05, 0.1)]            # a single fall
        t = self.tune({"passed_benchmark": False, "results": one_fall}, round_=2)
        self.assertAlmostEqual(t["rewards"]["termination_penalty"]["weight"], -250.0)
        self.assertIn("fast", next(c["reason"] for c in json.loads(
            (self.state / "tuning_history.jsonl").read_text().splitlines()[-1])["changes"]
            if c["setting"] == "rewards.termination_penalty.weight"))

    def test_passing_round_changes_nothing(self):
        t = self.tune({"passed_benchmark": True, "results": [scenario("stand", 1.0, 0.0, 0.0, 0.0)]})
        self.assertEqual(t, {})


if __name__ == "__main__":
    unittest.main()
