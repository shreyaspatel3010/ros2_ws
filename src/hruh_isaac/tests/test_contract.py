import copy
import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / "hruh_lab/hruh_lab/contract.py"
spec = importlib.util.spec_from_file_location("contract", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ContractTest(unittest.TestCase):
    def setUp(self):
        self.cfg = {
            "sim": {"dt": 0.005}, "decimation": 4,
            "scene": {"robot": {"init_state": {"joint_pos": {"knee": 0.36}}, "actuators": {"kp": 50}}},
            "actions": {"joint_pos": {"joint_names": ["hip", "knee"], "scale": 0.5}},
            "observations": {"policy": {"history_length": 5, "flatten_history_dim": True,
                "gravity": {"func": "gravity", "params": {}},
                "joints": {"func": "encoders", "params": {"asset_cfg": {"joint_names": ["hip", "knee"]}}}}},
        }

    def test_same_shape_wrong_joint_order_rejected(self):
        other = copy.deepcopy(self.cfg)
        other["actions"]["joint_pos"]["joint_names"].reverse()
        self.assertNotEqual(module.signature(self.cfg), module.signature(other))

    def test_observation_order_is_part_of_contract(self):
        other = copy.deepcopy(self.cfg)
        policy = other["observations"]["policy"]
        term = policy.pop("gravity")
        policy["gravity"] = term
        self.assertNotEqual(module.signature(self.cfg), module.signature(other))

    def test_timing_and_gains_are_part_of_contract(self):
        for change in (lambda c: c["sim"].update(dt=0.01),
                       lambda c: c["scene"]["robot"]["actuators"].update(kp=100)):
            other = copy.deepcopy(self.cfg)
            change(other)
            self.assertNotEqual(module.signature(self.cfg), module.signature(other))

    def test_yaml_tuple_equivalence(self):
        other = copy.deepcopy(self.cfg)
        other["actions"]["joint_pos"]["joint_names"] = ("hip", "knee")
        self.assertEqual(module.signature(self.cfg), module.signature(other))


if __name__ == "__main__":
    unittest.main()
