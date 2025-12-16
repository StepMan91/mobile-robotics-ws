
import torch
import unittest
import math

# MOCKING CLASSES
class MockData:
    def __init__(self):
        self.root_quat_w = torch.tensor([[1.0, 0.0, 0.0, 0.0], [0.707, 0.0, 0.707, 0.0]]) # Upright, Tilted 90 Y
        self.projected_gravity_b = torch.tensor([[0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]) # Upright, Tilted
        self.root_pos_w = torch.tensor([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]]) # Base at 1.0m
        self.body_pos_w = torch.zeros(2, 5, 3) # [Envs, Bodies, 3]
        self.net_forces_w_history = torch.zeros(2, 1, 5) # [Envs, Hist, Bodies] force magnitude

class MockScene:
    def __init__(self):
        self.data = MockData()
        self.robot = self # Mock scene["robot"]
        self.contact_forces = self # Mock scene["contact_forces"]
    
    def __getitem__(self, key):
        return self

class MockEnv:
    def __init__(self):
        self.scene = MockScene()
        self.device = "cpu"

class MockSensorCfg:
    def __init__(self, name):
        self.name = name
        self.body_ids = [0, 1] # Mock ankle indices

# FUNCTIONS TO TEST (Copy-Pasted logic for isolation testing)
def illegal_tilt(env, limit: float = 0.26):
    gravity_z = env.scene["robot"].data.projected_gravity_b[:, 2]
    # Tilted if z > -cos(limit) (because gravity is negative Z)
    # Wait, projected gravity_b. If upright, g_b = (0,0,-1). 
    # If tilted 15 deg, g_b z component is -cos(15) ~ -0.96.
    # If tilted 90 deg, g_b z = 0.
    # We want to die if z > -cos(limit). (e.g. z = 0 > -0.96 => True/Die).
    return gravity_z > -math.cos(limit)

def feet_too_high(env, sensor_cfg, limit: float = 0.3):
    feet_pos = env.scene[sensor_cfg.name].data.body_pos_w[:, sensor_cfg.body_ids, 2] # Z
    base_pos = env.scene["robot"].data.root_pos_w[:, 2]
    # Die if Foot > Base - 0.3
    return torch.any(feet_pos > (base_pos.unsqueeze(1) - limit), dim=1)

def no_ground_contact(env, sensor_cfg):
    forces = env.scene[sensor_cfg.name].data.net_forces_w_history[:, 0, sensor_cfg.body_ids].norm(dim=-1)
    in_air = torch.all(forces < 1.0, dim=1)
    return in_air

class TestRLComponents(unittest.TestCase):
    def test_illegal_tilt(self):
        env = MockEnv()
        # Env 0: Upright (-1.0). Env 1: Tilted (0.0).
        # Limit 15 deg -> -0.96.
        # Env 0: -1.0 > -0.96 -> False (Safe)
        # Env 1: 0.0 > -0.96 -> True (Die)
        res = illegal_tilt(env, limit=0.26)
        print(f"Tilt Result: {res}")
        self.assertFalse(res[0].item())
        self.assertTrue(res[1].item())

    def test_feet_high(self):
        env = MockEnv()
        # Base at 1.0. Limit 0.3 -> Threshold 0.7.
        # Set Env 0 feet to 0.0 (Safe).
        # Set Env 1 feet to 0.8 (Too High).
        env.scene.data.body_pos_w[0, :, 2] = 0.0
        env.scene.data.body_pos_w[1, :, 2] = 0.8
        
        cfg = MockSensorCfg("robot")
        res = feet_too_high(env, cfg, limit=0.3)
        print(f"Feet High Result: {res}")
        self.assertFalse(res[0].item())
        self.assertTrue(res[1].item())

    def test_contact(self):
        env = MockEnv()
        # Env 0: Force 10.0 (Contact).
        # Env 1: Force 0.0 (Flying).
        env.scene.data.net_forces_w_history[0, 0, :] = 10.0
        env.scene.data.net_forces_w_history[1, 0, :] = 0.0
        
        cfg = MockSensorCfg("contact_forces")
        res = no_ground_contact(env, cfg)
        print(f"Contact Result: {res}")
        self.assertFalse(res[0].item())
        self.assertTrue(res[1].item())

if __name__ == '__main__':
    unittest.main()
