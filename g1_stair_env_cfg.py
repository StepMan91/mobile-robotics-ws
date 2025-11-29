from isaaclab.utils import configclass
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.sensors import RayCasterCfg, patterns

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg

# Define the path to the G1 USD file
# Using absolute path for now to ensure it works
G1_USD_PATH = r"c:\Users\basti\.gemini\antigravity\playground\cobalt-cosmos\assets\g1_29dof_rev_1_0\g1_29dof_rev_1_0.usd"

@configclass
class G1StairEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        
        # 1. Robot Configuration
        self.scene.robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UsdFileCfg(
                usd_path=G1_USD_PATH,
                activate_contact_sensors=True,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=False,
                    retain_accelerations=False,
                    linear_damping=0.0,
                    angular_damping=0.0,
                    max_linear_velocity=1000.0,
                    max_angular_velocity=1000.0,
                    max_depenetration_velocity=1.0,
                ),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False, # G1 might have self-collisions if not careful
                    solver_position_iteration_count=4,
                    solver_velocity_iteration_count=0,
                    sleep_threshold=0.005,
                    stabilization_threshold=0.001,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.0, 0.8), # Start slightly above ground
                rot=(1.0, 0.0, 0.0, 0.0),
                joint_pos={
                    # Default joint positions (standing)
                    # These names need to match the USD!
                    # I'll assume standard naming for now, might need to adjust
                    ".*": 0.0, 
                },
            ),
            actuators={
                "legs": ImplicitActuatorCfg(
                    joint_names_expr=[".*_hip_.*", ".*_knee_.*", ".*_ankle_.*"],
                    stiffness=100.0,
                    damping=2.0,
                ),
                "arms": ImplicitActuatorCfg(
                    joint_names_expr=[".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_.*"],
                    stiffness=50.0,
                    damping=1.0,
                ),
            },
        )
        
        # 2. Terrain Configuration (Stairs)
        # We override the terrain generator to focus on stairs
        # For now, keep rough terrain but we can tune it later
        
        # 3. Events
        # Adjust push interval etc.
        self.events.push_robot.interval_range_s = (10.0, 15.0)
        
        # 4. Rewards
        # Use default rewards for now
