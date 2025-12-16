from isaaclab.utils import configclass
from isaaclab_tasks.utils.wrappers.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)

@configclass
class UnitreeG1RoughPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    def __post_init__(self):
        super().__post_init__()
        self.max_iterations = 1500
        self.experiment_name = "unitree_g1_rough"
        self.policy.actor_hidden_dims = [512, 256, 128]
        self.policy.critic_hidden_dims = [512, 256, 128]
        # G1 is complex, might need more capacity
