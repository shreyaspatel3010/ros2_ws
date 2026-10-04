import os
from pathlib import Path
import sys
import unittest

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE / "hruh_lab"))
URDF = HERE.parents[1] / "artifacts/hruh/hruh_isaac.urdf"


@unittest.skipUnless(URDF.is_file(), "robot model not exported")
class SymmetryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from hruh_lab import symmetry
        from hruh_lab.joints import ARM_JOINTS, LEG_JOINTS, WAIST_JOINTS, stand_pose_value
        from hruh_lab.portable import Kinematics
        cls.sym, cls.kin = symmetry, Kinematics(str(URDF))
        cls.names = LEG_JOINTS + WAIST_JOINTS + ARM_JOINTS["left"] + ARM_JOINTS["right"]
        cls.axes = symmetry.joint_axes(str(URDF))
        cls.stand = {n: stand_pose_value(n) for n in cls.names}
        cls.plan = symmetry.mirror_plan([("joint_pos", len(cls.names))], {"joint_pos": cls.names}, cls.axes, cls.stand)

    def test_mirrored_pose_moves_limbs_to_mirror_image(self):
        rng = np.random.default_rng(0)
        for _ in range(20):
            q = rng.uniform(-0.6, 0.6, size=len(self.names))
            mirrored = self.sym.apply_plan(q[None], self.plan)[0]
            pose, pose_m = dict(zip(self.names, q)), dict(zip(self.names, mirrored))
            for body, partner, tol in (("left_foot_link", "right_foot_link", 1e-6),
                                       ("right_foot_link", "left_foot_link", 1e-6),
                                       # the URDF arms are not exactly mirrored (shoulders 0.104 vs 0.117 m from
                                       # the centre): ~1.1 cm at zero pose, <= 2.5 cm over random poses
                                       ("left_wrist", "right_wrist", 0.03)):
                expected = self.sym.REFLECTION @ self.kin.transform(partner, pose)[:3, 3]
                got = self.kin.transform(body, pose_m)[:3, 3]
                np.testing.assert_allclose(got, expected, atol=tol, err_msg=body)

    def test_mirror_twice_is_identity_and_waist_signs(self):
        x = np.random.default_rng(1).normal(size=(4, 3 * len(self.names)))
        plan = self.sym.mirror_plan([("joint_pos", 3 * len(self.names))], {"joint_pos": self.names}, self.axes)
        np.testing.assert_allclose(self.sym.apply_plan(self.sym.apply_plan(x, plan), plan), x)
        perm, signs = self.sym.joint_mirror(["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"], self.axes)
        self.assertEqual((perm, signs), ([0, 1, 2], [-1, -1, 1]))

    def test_vectors_and_unknown_terms(self):
        plan = self.sym.mirror_plan([("base_ang_vel", 6), ("velocity_commands", 3)], {}, self.axes)
        out = self.sym.apply_plan(np.array([[1., 2, 3, 4, 5, 6, 0.4, 0.2, 0.5]]), plan)
        np.testing.assert_allclose(out, [[-1, 2, -3, -4, 5, -6, 0.4, -0.2, -0.5]])
        with self.assertRaises(ValueError):
            self.sym.mirror_plan([("height_scan", 10)], {}, self.axes)


if __name__ == "__main__":
    unittest.main()
