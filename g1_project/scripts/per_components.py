
import torch
import numpy as np
import os
import sys

# Attempt to import rsl_rl components
try:
    from rsl_rl.storage import RolloutStorage
    from rsl_rl.algorithms import PPO
    from rsl_rl.runners import OnPolicyRunner
    from rsl_rl.modules import ActorCritic, ActorCriticRecurrent, resolve_rnd_config, resolve_symmetry_config
    from rsl_rl.utils import resolve_obs_groups
except ImportError:
    # If running in environment without rsl_rl, define mocks or fail
    # For now we assume this runs in the correct env
    class RolloutStorage:
        class Transition: pass
        def __init__(self, *args, **kwargs): pass
    class PPO:
        def __init__(self, *args, **kwargs): pass
    class OnPolicyRunner:
        def __init__(self, *args, **kwargs): pass
    def resolve_rnd_config(*args): return {}
    def resolve_symmetry_config(*args): return {}
    def resolve_obs_groups(*args): return {}
    class ActorCritic: pass
    class ActorCriticRecurrent: pass


class SumTree:
    """
    SumTree structure for Prioritized Experience Replay.
    Stores priorities in a binary tree for valid sampling in O(log N).
    """
    def __init__(self, capacity):
        self.capacity = capacity
        # Tree size is 2 * capacity - 1
        # but for simplicity we use array of size 2 * capacity
        self.tree = np.zeros(2 * capacity - 1)
        self.data_pointer = 0
        self.count = 0 
        
    def add(self, priority, data_index):
        """
        Add priority to the tree.
        data_index: index in the external buffer corresponding to this priority.
        """
        tree_idx = self.data_pointer + self.capacity - 1
        self.update(tree_idx, priority)
        
        self.data_pointer += 1
        if self.data_pointer >= self.capacity:
            self.data_pointer = 0
            
        if self.count < self.capacity:
            self.count += 1
            
    def update(self, tree_idx, priority):
        """
        Update priority at tree_idx and propagate changes up.
        """
        change = priority - self.tree[tree_idx]
        self.tree[tree_idx] = priority
        
        # Propagate
        while tree_idx != 0:
            tree_idx = (tree_idx - 1) // 2
            self.tree[tree_idx] += change
            
    def get_leaf(self, v):
        """
        Get leaf index and priority for a given value v.
        """
        parent_idx = 0
        while True:
            left_child_idx = 2 * parent_idx + 1
            right_child_idx = left_child_idx + 1
            
            # If we reach bottom, end
            if left_child_idx >= len(self.tree):
                leaf_idx = parent_idx
                break
                
            if v <= self.tree[left_child_idx]:
                parent_idx = left_child_idx
            else:
                v -= self.tree[left_child_idx]
                parent_idx = right_child_idx
                
        data_idx = leaf_idx - self.capacity + 1
        return leaf_idx, self.tree[leaf_idx], data_idx
    
    @property
    def total_priority(self):
        return self.tree[0]

class PrioritizedRolloutStorage(RolloutStorage):
    """
    Extends RolloutStorage to support Prioritized sampling.
    """
    def __init__(self, training_type, num_envs, num_transitions_per_env, obs, actions_shape, device="cpu", alpha=0.6, beta_start=0.4, beta_frames=1000):
        super().__init__(training_type, num_envs, num_transitions_per_env, obs, actions_shape, device)
        
        self.alpha = alpha
        self.beta_start = beta_start
        self.beta = beta_start
        self.beta_frames = beta_frames # For scheduling
        self.frame = 1
        
        self.capacity = num_envs * num_transitions_per_env
        self.tree = SumTree(self.capacity)
        self.min_priority = 1.0 # Default max priority

    def update_priorities(self, indices, priorities):
        """
        Update priorities of sampled transitions.
        indices: tree indices
        priorities: new priorities (abs TC error)
        """
        for idx, p in zip(indices, priorities):
            self.tree.update(idx, p ** self.alpha)
            self.min_priority = min(self.min_priority, p ** self.alpha)
            
    def compute_returns(self, last_values, gamma, lam, normalize_advantage=True):
        # Call super to compute returns and advantages
        super().compute_returns(last_values, gamma, lam, normalize_advantage)
        
        # Initialize priorities based on Advantages (which mimic TD error)
        # Flatten advantages to match tree structure
        advantages_flat = self.advantages.flatten(0, 1).cpu().numpy()
        priorities = np.abs(advantages_flat) + 1e-5
        
        # Re-build tree (brute force for now, efficient enough for <10k items)
        # Ideally we would update incrementally but this is a batch buffer reset.
        # Actually PPO buffer is cleared every update.
        # So we just fill the tree.
        for i in range(len(priorities)):
             # tree_idx = i + capacity - 1
             # We can just call add, but we need to reset pointer first?
             # SumTree implementation assumes circular buffer.
             # Here we linear fill.
             pass
        
        # Optimization: Just rebuild tree array directly?
        # For simplicity, let's just loop add.
        self.tree.data_pointer = 0
        self.tree.count = 0
        for p in priorities:
            self.tree.add(p ** self.alpha, 0) # data_index 0 is dummy, we map by index logic
            
    def mini_batch_generator(self, num_mini_batches, num_epochs=8):
        # Override to sample using SumTree
        if self.training_type != "rl":
            raise ValueError("Prioritized sampling only for RL")
            
        batch_size = self.num_envs * self.num_transitions_per_env
        mini_batch_size = batch_size // num_mini_batches
        
        # Beta scheduling
        # self.beta = min(1.0, self.beta_start + self.frame * (1.0 - self.beta_start) / self.beta_frames)
        # self.frame += 1 # Update frame count? Handled by runner?
        
        # Flattened data
        observations = self.observations.flatten(0, 1)
        actions = self.actions.flatten(0, 1)
        values = self.values.flatten(0, 1)
        returns = self.returns.flatten(0, 1)
        old_actions_log_prob = self.actions_log_prob.flatten(0, 1)
        advantages = self.advantages.flatten(0, 1)
        old_mu = self.mu.flatten(0, 1)
        old_sigma = self.sigma.flatten(0, 1)
        
        for epoch in range(num_epochs):
            for i in range(num_mini_batches):
                # Sampling
                batch_indices = []
                tree_indices = []
                priorities = []
                
                segment = self.tree.total_priority / mini_batch_size
                
                for k in range(mini_batch_size):
                    a = segment * k
                    b = segment * (k + 1)
                    s = np.random.uniform(a, b)
                    (tree_idx, p, data_idx) = self.tree.get_leaf(s)
                    
                    # data_idx from SumTree logic corresponds to absolute index if filled linearly
                    # SumTree returns data_idx = leaf_idx - capacity + 1
                    # Since we filled 0 to N-1, data_idx should be correct
                    
                    batch_indices.append(data_idx)
                    tree_indices.append(tree_idx)
                    priorities.append(p)
                
                batch_indices = torch.tensor(batch_indices, dtype=torch.long, device=self.device)
                
                # Importance Sampling Weights
                # w = (N * P)^-beta / max_w
                probabilities = np.array(priorities) / self.tree.total_priority
                weights = (self.capacity * probabilities) ** (-self.beta)
                weights = weights / weights.max()
                weights = torch.tensor(weights, dtype=torch.float32, device=self.device)
                
                # Fetch Data
                obs_batch = observations[batch_indices]
                actions_batch = actions[batch_indices]
                target_values_batch = values[batch_indices]
                returns_batch = returns[batch_indices]
                old_actions_log_prob_batch = old_actions_log_prob[batch_indices]
                advantages_batch = advantages[batch_indices]
                old_mu_batch = old_mu[batch_indices]
                old_sigma_batch = old_sigma[batch_indices]
                
                hidden_state_a_batch = None
                hidden_state_c_batch = None
                masks_batch = None
                
                # Yield extra components: weights and tree_indices (to update priorities later)
                yield (
                    obs_batch,
                    actions_batch,
                    target_values_batch,
                    advantages_batch,
                    returns_batch,
                    old_actions_log_prob_batch,
                    old_mu_batch,
                    old_sigma_batch,
                    (hidden_state_a_batch, hidden_state_c_batch),
                    masks_batch,
                    weights,    # [NEW]
                    tree_indices # [NEW]
                )

class PrioritizedPPO(PPO):
    """
    PPO variant that uses PrioritizedRolloutStorage.
    """
    def init_storage(self, training_type, num_envs, num_transitions_per_env, obs, actions_shape):
        # Instantiate Prioritized Storage
        self.storage = PrioritizedRolloutStorage(
            training_type, num_envs, num_transitions_per_env, obs, actions_shape, self.device
        )
        
    def update(self):
        # Copy of PPO.update but unpacking extra items and applying weights
        mean_value_loss = 0
        mean_surrogate_loss = 0
        mean_entropy = 0
        mean_rnd_loss = 0 if self.rnd else None
        mean_symmetry_loss = 0 if self.symmetry else None

        # Get mini batch generator
        if self.policy.is_recurrent:
             # Recurrent PER not fully implemented yet
             raise NotImplementedError("Recurrent PER not supported")
        else:
            generator = self.storage.mini_batch_generator(self.num_mini_batches, self.num_learning_epochs)

        for (
            obs_batch,
            actions_batch,
            target_values_batch,
            advantages_batch,
            returns_batch,
            old_actions_log_prob_batch,
            old_mu_batch,
            old_sigma_batch,
            hidden_states_batch,
            masks_batch,
            weights_batch, # [NEW]
            tree_indices,  # [NEW]
        ) in generator:
            
            # ... (Symmetry/Augmentation logic omitted for brevity if not used, else copy)
            # Assuming no symmetry/rnd for now or copy it if needed. 
            # I'll stick to core PPO logic for brevity, but referencing standard logic.
            
            # Recompute policy
            self.policy.act(obs_batch, masks=masks_batch, hidden_state=hidden_states_batch[0])
            actions_log_prob_batch = self.policy.get_actions_log_prob(actions_batch)
            value_batch = self.policy.evaluate(obs_batch, masks=masks_batch, hidden_state=hidden_states_batch[1])
            entropy_batch = self.policy.entropy
            
            # Surrogate loss
            ratio = torch.exp(actions_log_prob_batch - torch.squeeze(old_actions_log_prob_batch))
            surrogate = -torch.squeeze(advantages_batch) * ratio
            surrogate_clipped = -torch.squeeze(advantages_batch) * torch.clamp(
                ratio, 1.0 - self.clip_param, 1.0 + self.clip_param
            )
            # Apply Weights for PER
            surrogate_loss = torch.max(surrogate, surrogate_clipped) * weights_batch # Weighted
            surrogate_loss = surrogate_loss.mean()

            # Value function loss
            value_losses = (value_batch - returns_batch).pow(2)
            if self.use_clipped_value_loss:
                value_clipped = target_values_batch + (value_batch - target_values_batch).clamp(
                    -self.clip_param, self.clip_param
                )
                value_losses_clipped = (value_clipped - returns_batch).pow(2)
                value_loss = torch.max(value_losses, value_losses_clipped)
            else:
                value_loss = value_losses
                
            value_loss = (value_loss * weights_batch.unsqueeze(1)).mean() # Weighted

            loss = surrogate_loss + self.value_loss_coef * value_loss - self.entropy_coef * entropy_batch.mean()

            # ... (RND/Symmetry loss would be added here)

            self.optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
            self.optimizer.step()
            
            # UPDATE PRIORITIES
            # Calculate new TD errors for the batch
            with torch.no_grad():
                new_values = self.policy.evaluate(obs_batch)
                # TD Error approximation: Returns - NewValues (or use Advantages recomputed?)
                # Actually, standard PER uses TD error.
                # Here we can use the absolute advantage or value error.
                # Let's use value error |V_target - V_pred|
                new_td_errors = torch.abs(returns_batch - new_values).cpu().numpy().flatten()
                
            self.storage.update_priorities(tree_indices, new_td_errors)
            
            mean_value_loss += value_loss.item()
            mean_surrogate_loss += surrogate_loss.item()
            mean_entropy += entropy_batch.mean().item()

        num_updates = self.num_learning_epochs * self.num_mini_batches
        mean_value_loss /= num_updates
        mean_surrogate_loss /= num_updates
        mean_entropy /= num_updates
        
        self.storage.clear()
        
        return {
            "value_function": mean_value_loss,
            "surrogate": mean_surrogate_loss,
            "entropy": mean_entropy,
        }






