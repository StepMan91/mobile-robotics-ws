import gymnasium as gym
from .g1_env_cfg import G1LocomotionEnvCfg

gym.register(
    id="Isaac-Locomotion-G1-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": G1LocomotionEnvCfg,
    },
)
