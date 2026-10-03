import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("promote_policy", HERE / "scripts/promote_policy.py")
promote = importlib.util.module_from_spec(spec)
spec.loader.exec_module(promote)
sys.path.insert(0, str(HERE / "hruh_lab"))
try:
    import numpy  # noqa: F401
    from hruh_lab.portable import PolicyIO
except ImportError:  # numpy-free environments still run the promotion tests
    PolicyIO = None

URDF = """<robot name="t">
  <link name="base_link"/><link name="thigh"/><link name="left_foot_link"/><link name="right_foot_link"/>
  <joint name="hip" type="revolute"><parent link="base_link"/><child link="thigh"/>
    <origin xyz="0 0 -0.1"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
  <joint name="lf" type="fixed"><parent link="thigh"/><child link="left_foot_link"/><origin xyz="0 0.1 -0.7"/></joint>
  <joint name="rf" type="fixed"><parent link="base_link"/><child link="right_foot_link"/><origin xyz="0 -0.1 -0.8"/></joint>
</robot>"""


def sha(data):
    return hashlib.sha256(data).hexdigest()


class PromoteTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.urdf = self.tmp / "current.urdf"
        self.urdf.write_text(URDF)
        self.dest, self.history = self.tmp / "policies", self.tmp / "history"

    def bundle(self, name, weights=b"weights", skill="reach", urdf=URDF):
        folder = self.tmp / name / "exported"
        folder.mkdir(parents=True)
        (folder / "policy.pt").write_bytes(weights)
        (folder / "robot.urdf").write_text(urdf)
        checkpoint = folder.parent / "model_9.pt"
        checkpoint.write_bytes(b"ckpt" + weights)
        (folder / "bundle.json").write_text(json.dumps({
            "skill": skill, "policy_sha256": sha(weights), "urdf_sha256": sha(urdf.encode()),
            "checkpoint": str(checkpoint)}))
        return folder

    def evaluation(self, name, passed=True, success=0.95):
        path = self.tmp / f"{name}.json"
        path.write_text(json.dumps({"passed_benchmark": passed, "success_fraction": success}))
        return path

    def run_promote(self, bundle, evaluation, **kw):
        return promote.promote(bundle, evaluation, dest_root=self.dest, history_root=self.history,
                               current_urdf=self.urdf, **kw)

    def test_failed_benchmark_is_never_promoted(self):
        ok, message = self.run_promote(self.bundle("a"), self.evaluation("a", passed=False))
        self.assertFalse(ok)
        self.assertIn("FAIL", message)
        self.assertFalse((self.dest / "reach").exists())

    def test_pass_is_promoted_with_manifest(self):
        ok, _ = self.run_promote(self.bundle("a"), self.evaluation("a"), run_id="r1")
        self.assertTrue(ok)
        manifest = json.loads((self.dest / "reach/PROMOTED.json").read_text())
        self.assertEqual(manifest["isaac_benchmark"], "PASS")
        self.assertEqual(manifest["run_id"], "r1")
        self.assertEqual(manifest["gazebo_transfer"], "untested")
        self.assertTrue((self.dest / "reach/policy.pt").is_file())
        self.assertTrue((self.dest / "reach/evaluation.json").is_file())

    def test_tampered_policy_rejected(self):
        folder = self.bundle("a")
        (folder / "policy.pt").write_bytes(b"other")
        ok, message = self.run_promote(folder, self.evaluation("a"))
        self.assertFalse(ok)
        self.assertIn("checksum", message)

    def test_policy_for_another_robot_rejected(self):
        ok, message = self.run_promote(self.bundle("a", urdf=URDF.replace("0.7", "0.6")), self.evaluation("a"))
        self.assertFalse(ok)
        self.assertIn("different robot", message)

    def test_worse_policy_keeps_current_and_better_replaces_with_backup(self):
        self.assertTrue(self.run_promote(self.bundle("a", b"A"), self.evaluation("a", success=0.95))[0])
        ok, message = self.run_promote(self.bundle("b", b"B"), self.evaluation("b", success=0.91))
        self.assertFalse(ok)
        self.assertIn("kept", message)
        ok, _ = self.run_promote(self.bundle("c", b"C"), self.evaluation("c", success=0.99))
        self.assertTrue(ok)
        self.assertEqual((self.dest / "reach/policy.pt").read_bytes(), b"C")
        backups = list((self.history / "reach").iterdir())
        self.assertEqual(len(backups), 1)
        self.assertEqual((backups[0] / "policy.pt").read_bytes(), b"A")

    def test_require_gazebo(self):
        gazebo = self.tmp / "gazebo.json"
        gazebo.write_text(json.dumps({"completed": True, "falls": 1, "policy_steps": 100}))
        ok, message = self.run_promote(self.bundle("a"), self.evaluation("a"), gazebo=gazebo, require_gazebo=True)
        self.assertFalse(ok)
        self.assertIn("Gazebo transfer fail", message)
        gazebo.write_text(json.dumps({"completed": True, "falls": 0, "policy_steps": 100}))
        self.assertTrue(self.run_promote(self.bundle("b"), self.evaluation("b"), gazebo=gazebo,
                                         require_gazebo=True)[0])

    def test_locomotion_score_prefers_survival(self):
        rows = lambda s, e: {"results": [{"survival_fraction": s, "velocity_mae_while_alive_vx_vy_wz": [e, e, e]}]}
        self.assertGreater(promote.score("locomotion", rows(1.0, 0.1)), promote.score("locomotion", rows(0.9, 0.0)))
        self.assertGreater(promote.score("locomotion", rows(1.0, 0.1)), promote.score("locomotion", rows(1.0, 0.2)))


@unittest.skipIf(PolicyIO is None, "numpy unavailable")
class PortableTest(unittest.TestCase):
    def setUp(self):
        tmp = Path(tempfile.mkdtemp())
        (tmp / "robot.urdf").write_text(URDF)
        self.cfg = {"policy_joints": ["hip"], "default_positions": {"hip": 0.0}, "raw_action_clip": 2.0,
                    "scale": [0.5], "offset": [0.0], "limits": [[-1.0, 1.0]], "control_dt": 0.02,
                    "initial_root_position": [0, 0, 0.85], "skill": "locomotion"}
        self.urdf = tmp / "robot.urdf"

    def test_action_clip_follows_bundle(self):
        io = PolicyIO(self.cfg, self.urdf)
        self.assertAlmostEqual(io.targets([1.6])["hip"], 0.8)    # clip 2.0: not cut at 1.0
        self.assertAlmostEqual(io.targets([3.0])["hip"], 1.0)    # 2.0 * 0.5
        io = PolicyIO(dict(self.cfg, raw_action_clip=None), self.urdf)
        self.assertAlmostEqual(io.targets([3.0])["hip"], 1.0)    # joint limit still applies

    def test_pelvis_height_calibrated_to_stand_pose(self):
        io = PolicyIO(self.cfg, self.urdf)
        self.assertAlmostEqual(io.pelvis_height({"hip": 0.0}, [0, 0, 0, 1]), 0.85)
        # thigh swings: the right foot (lower, fixed) still carries the pelvis
        self.assertAlmostEqual(io.pelvis_height({"hip": 0.5}, [0, 0, 0, 1]), 0.85)


if __name__ == "__main__":
    unittest.main()


class ArmMotionTest(unittest.TestCase):
    def setUp(self):
        from hruh_lab import joints
        self.joints = joints

    def test_counter_swing(self):
        t = self.joints.arm_swing_targets(-0.5, 0.1, gain=0.3)        # left leg forward
        self.assertAlmostEqual(t["chest_to_left_shoulder"], 0.3)        # left arm back
        self.assertAlmostEqual(t["chest_to_right_shoulder"], -0.3)      # right arm forward
        self.assertGreater(t["right_elbow_inword_to_midle"], t["left_elbow_inword_to_midle"])
        still = self.joints.arm_swing_targets(-0.21, -0.21)
        self.assertEqual(still, {n: self.joints.stand_pose_value(n) for n in still})

    def test_pose_ranges_cover_stand_pose(self):
        for side in self.joints.SIDES:
            for name in self.joints.ARM_JOINTS[side]:
                lo, hi = self.joints.arm_pose_range(name)
                self.assertLessEqual(lo, self.joints.stand_pose_value(name))
                self.assertGreaterEqual(hi, self.joints.stand_pose_value(name))


@unittest.skipIf(PolicyIO is None, "numpy unavailable")
class ObservedJointsTest(unittest.TestCase):
    def test_observation_includes_extra_joints(self):
        tmp = Path(tempfile.mkdtemp())
        (tmp / "robot.urdf").write_text(URDF)
        cfg = {"policy_joints": ["hip"], "observation_joints": ["hip", "arm"], "default_positions": {"hip": 0.0, "arm": 0.1},
               "raw_action_clip": 1.0, "scale": [0.5], "offset": [0.0], "limits": [[-1.0, 1.0]], "control_dt": 0.02,
               "initial_root_position": [0, 0, 0.85], "skill": "locomotion", "history_length": 1,
               "observation_terms": ["joint_pos", "joint_vel", "actions"], "observation_size": 5}
        io = PolicyIO(cfg, tmp / "robot.urdf")
        obs = io.observation({"hip": 0.2, "arm": 0.4}, {"hip": 1.0, "arm": 2.0}, [0, 0, 0, 1], [0, 0, 0])
        self.assertEqual([round(float(x), 3) for x in obs], [0.2, 0.3, 1.0, 2.0, 0.0])
