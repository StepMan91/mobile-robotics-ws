
import unittest
import numpy as np
import torch
import sys
import os
from unittest.mock import MagicMock

# Add local path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Monkey patch rsl_rl for testing BEFORE importing per_components
sys.modules["rsl_rl"] = MagicMock()
sys.modules["rsl_rl.storage"] = MagicMock()
sys.modules["rsl_rl.algorithms"] = MagicMock()
sys.modules["rsl_rl.runners"] = MagicMock()
sys.modules["rsl_rl.modules"] = MagicMock()
sys.modules["rsl_rl.utils"] = MagicMock()

# Define MockRolloutStorage that looks like rsl_rl.storage.RolloutStorage
class MockRolloutStorage:
    class Transition: pass
    def __init__(self, *args, **kwargs): 
        self.observations = torch.zeros(2, 5, 1) # [num_envs, num_transitions, dim]
        self.actions = torch.zeros(2, 5, 1)
        self.values = torch.zeros(2, 5, 1)
        self.returns = torch.zeros(2, 5, 1)
        self.actions_log_prob = torch.zeros(2, 5, 1)
        self.advantages = torch.zeros(2, 5, 1)
        self.mu = torch.zeros(2, 5, 1)
        self.sigma = torch.zeros(2, 5, 1)
        self.device = 'cpu'
        self.training_type = 'rl'
        self.num_envs = 2
        self.num_transitions_per_env = 5
        self.batch_size = 10
        
    def compute_returns(self, *args, **kwargs):
        pass

sys.modules["rsl_rl.storage"].RolloutStorage = MockRolloutStorage

from per_components import SumTree, PrioritizedRolloutStorage

class TestSumTree(unittest.TestCase):
    def test_add_and_sample(self):
        capacity = 10
        tree = SumTree(capacity)
        
        # Add priorities
        for i in range(capacity):
            tree.add(1.0, i)
            
        self.assertAlmostEqual(tree.total_priority, 10.0)
        
        # Update one priority
        # Data index 0 is at tree_idx = 0 + 10 - 1 = 9.
        tree.update(9, 10.0)
        self.assertAlmostEqual(tree.total_priority, 19.0)
        
        # Sample
        # Based on analysis, index 0 (tree 9) covers mass in range [4, 14) or similar depending on topology.
        # But we know it has mass 10.0. All others 1.0.
        # It's the dominant element.
        # Let's verify we CAN hit it.
        # We can scan or just check specific value.
        # Let's just update ALL others to 0 to be sure where it is.
        for i in range(capacity):
             if i != 0:
                 # tree_idx = i + 9
                 tree.update(i+9, 0.0)
        
        # Now total priority = 10.0 (only index 0).
        # Any sample in [0, 10] should return index 0.
        leaf_idx, p, data_idx = tree.get_leaf(5.0)
        self.assertEqual(data_idx, 0)
        self.assertEqual(p, 10.0)

class TestPrioritizedStorage(unittest.TestCase):
    def test_mini_batch_generator(self):
        storage = PrioritizedRolloutStorage("rl", 2, 5, None, None)
        
        # Manually set data size to match constructor
        storage.observations = torch.zeros(2, 5, 1)
        # ... others initialized by MockRolloutStorage
        
        # Compute returns (initializes priorities)
        storage.compute_returns(None, 0.99, 0.95)
        
        # Generate batch
        gen = storage.mini_batch_generator(num_mini_batches=2, num_epochs=1)
        for batch in gen:
            obs, act, val, adv, ret, logp, mu, sigma, hid, mask, weights, indices = batch
            
            self.assertEqual(len(weights), 5) 
            self.assertEqual(len(indices), 5)
            
            # Update priorities
            new_p = np.ones(5)
            storage.update_priorities(indices, new_p)
            
        print("Generator test passed")
