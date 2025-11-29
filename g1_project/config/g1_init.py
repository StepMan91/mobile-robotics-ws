import gymnasium as gym

from . import agents
from .g1_stair_env_cfg import G1StairEnvCfg

##
# Register Gym environments.
##

gym.register(
    id="Isaac-Velocity-G1-Stairs-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": G1StairEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:UnitreeG1RoughPPORunnerCfg",
    },
)
