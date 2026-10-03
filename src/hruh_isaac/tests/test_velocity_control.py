import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / "hruh_lab/hruh_lab/velocity_control.py"
spec = importlib.util.spec_from_file_location("velocity_control", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class VelocityGuardTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.guard = module.VelocityGuard(clock=lambda: self.now)

    def test_idle_ramp_limits_and_timeout(self):
        self.assertEqual(self.guard.step(0.02), (0, 0, 0))
        self.guard.update((10, -10, 10))
        self.assertEqual(self.guard.step(0.02), (0.008, -0.008, 0.02))
        for _ in range(100):
            self.guard.step(0.02)
        self.assertEqual(self.guard.step(0.02), (0.4, -0.2, 0.5))
        self.now += 0.36
        self.assertEqual(self.guard.step(0.02), (0, 0, 0))

    def test_invalid_input_clears_motion(self):
        self.guard.update((0.3, 0, 0))
        self.guard.step(0.02)
        self.assertFalse(self.guard.update((float("nan"), 0, 0)))
        self.assertEqual(self.guard.step(0.02), (0, 0, 0))
        self.assertFalse(self.guard.update((0, float("inf"), 0)))
        self.assertFalse(self.guard.update((0, 0)))

    def test_fall_requires_fresh_neutral(self):
        self.guard.update((0.3, 0, 0))
        self.guard.fall()
        self.guard.update((0.3, 0, 0))
        self.assertEqual(self.guard.step(0.02), (0, 0, 0))
        self.guard.update((0, 0, 0))
        self.assertFalse(self.guard.latched)
        self.guard.update((0.3, 0, 0))
        self.assertGreater(self.guard.step(0.02)[0], 0)

    def test_bad_timestep(self):
        for dt in (0, -1, float("nan")):
            with self.assertRaises(ValueError):
                self.guard.step(dt)


if __name__ == "__main__":
    unittest.main()
