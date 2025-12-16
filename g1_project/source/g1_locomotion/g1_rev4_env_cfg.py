from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg, TerrainGeneratorCfg
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
import math
import torch

# --- CUSTOM REWARDS (REV4) ---

def hand_rail_distance(env, asset_cfg: SceneEntityCfg, rail_start: tuple, rail_end: tuple):
    """Reward for keeping the hand close to the rail line."""
    # Approximate rail geometry
    hand_pos = env.scene[asset_cfg.name].data.body_pos_w[:, asset_cfg.body_ids[0], :]
    p1 = torch.tensor(rail_start, device=env.device)
    p2 = torch.tensor(rail_end, device=env.device)
    line_vec = p2 - p1
    line_len_sq = torch.sum(line_vec**2)
    t = torch.sum((hand_pos - p1) * line_vec, dim=1) / line_len_sq
    t = torch.clamp(t, 0.0, 1.0)
    closest_point = p1 + t.unsqueeze(1) * line_vec
    distance = torch.norm(hand_pos - closest_point, dim=1)
    # Strong geometric reward
    return torch.exp(-distance * 3.0) 

def look_at_stairs(env, asset_cfg: SceneEntityCfg):
    """Reward for tilting torso forward/down to 'look' at stairs."""
    # Get quaternion of the torso
    body_quat = env.scene["robot"].data.body_quat_w[:, asset_cfg.body_ids[0]]
    
    # We want local X-axis (Forward) to point slightly DOWN in world frame.
    # Transform local X-axis (1,0,0) by body quaternion
    # Standard quaternion rotation: q * v * q_inv
    # Simplified math for rotating vector [1, 0, 0] by quat [w, x, y, z]:
    # x_w = 1 - 2y^2 - 2z^2
    # y_w = 2xy + 2wz
    # z_w = 2xz - 2wy
    
    w = body_quat[:, 0]
    x = body_quat[:, 1]
    y = body_quat[:, 2]
    z = body_quat[:, 3]
    
    # We care about Z component of the forward vector (z_w)
    # If z_w < 0, it points DOWN. If z_w > 0, it points UP.
    vec_forward_z = 2 * (x * z - w * y)
    
    # Reward for negative Z (looking down). 
    # Target: -0.2 (approx 10-15 degrees down)
    error = torch.abs(vec_forward_z - (-0.3)) 
    return torch.exp(-error * 5.0)

def climb_progress_reward(env, command_name: str):
    """Reward for moving Forward (+X) and Up (+Z)."""
    root_vel = env.scene["robot"].data.root_lin_vel_w
    vel_x = root_vel[:, 0]
    vel_z = root_vel[:, 2]
    # Reward UP significantly during climbing
    return vel_x + vel_z * 2.0 

def feet_air_time(env, sensor_cfg: SceneEntityCfg, command_name: str, threshold: float):
    sensor = env.scene[sensor_cfg.name]
    # Reward air time ONLY if command is active (moving)
    air_time = sensor.data.current_air_time[:, sensor_cfg.body_ids]
    return torch.sum(torch.clamp(air_time, max=threshold), dim=1)

# ----------------------

@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-1.5, n_max=1.5))
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        actions = ObsTerm(func=mdp.last_action)
        
        # PERCEPTION (The Eyes)
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )

    policy: PolicyCfg = PolicyCfg()

@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.25, use_default_offset=True)

@configclass
class RewardsCfg:
    # -- Task --
    climb_progress = RewTerm(
        func=climb_progress_reward,
        weight=5.0, # Balanced Driver
        params={"command_name": "base_velocity"}
    )
    
    feet_air_time = RewTerm(
        func=feet_air_time,
        weight=1.0, # Encourages stepping
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_ankle_roll_link"),
            "command_name": "base_velocity",
            "threshold": 0.5,
        }
    )
    
    # Hand Rail (User Request)
    hand_rail = RewTerm(
        func=hand_rail_distance,
        weight=1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="left_wrist_roll_link"), # Left Hand on Rail?
            "rail_start": (3.0, 0.5, 1.05), # Approx start of rail
            "rail_end": (6.75, 0.5, 3.3),   # Approx top of rail
        }
    )
    
    # Look At Stairs (Modified for valid body)
    look_at_stairs = RewTerm(
        func=look_at_stairs,
        weight=2.0,
        params={"asset_cfg": SceneEntityCfg("robot", body_names="torso_link")}
    )
    
    # -- Regularization --
    dof_torques_l2 = RewTerm(func=mdp.rewards.joint_torques_l2, weight=-1.0e-5)
    action_rate_l2 = RewTerm(func=mdp.rewards.action_rate_l2, weight=-0.01) # Standard smooth penalty
    dof_pos_limits = RewTerm(func=mdp.rewards.joint_pos_limits, weight=-5.0)
    
    # Stability
    orientation = RewTerm(func=mdp.rewards.flat_orientation_l2, weight=-1.0) # Stay upright

@configclass
class CommandsCfg:
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.02,
        rel_heading_envs=1.0,
        heading_command=False, 
        debug_vis=False,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.4, 0.8), # Conservative forward speed
            lin_vel_y=(-0.0, 0.0),
            ang_vel_z=(-0.1, 0.1),
        ),
    )

@configclass
class EventCfg:
    # --- DOMAIN RANDOMIZATION (SIM-TO-REAL) ---
    
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.5, 1.2),  # Varied floors
            "dynamic_friction_range": (0.4, 1.0),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )
    
    add_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="torso_link"),
            "mass_distribution_params": (-5.0, 5.0), # Payload variance
            "operation": "add",
        },
    )
    
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(2.5, 5.0),
        params={"asset_cfg": SceneEntityCfg("robot"), "velocity_range": {"x": (-0.2, 0.2), "y": (-0.2, 0.2)}},
    )
    
    # --- IMPLICIT CURRICULUM ---
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {"x": (0.0, 3.0), "y": (-0.2, 0.2), "yaw": (-0.05, 0.05)}, # 0 to 3m from origin
            "velocity_range": {},
        },
    )
    
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        },
    )

from isaaclab.managers import TerminationTermCfg as TermTerm

@configclass
class TerminationsCfg:
    time_out = TermTerm(func=mdp.time_out, params={})
    check_tilt = TermTerm(func=mdp.bad_orientation, params={"limit_angle": 0.6}) # ~35 deg treshold
    base_low = TermTerm(func=mdp.root_height_below_minimum, params={"minimum_height": 0.3})

@configclass
class G1Rev4EnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for G1 Rev4 (Robust Climber)."""
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=2.5)
    episode_length_s = 20.0
    decimation = 4
    
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    commands: CommandsCfg = CommandsCfg()
    
    def __post_init__(self):
        super().__post_init__()
        self.sim.dt = 0.005 
        self.sim.render_interval = 4
        
        self.scene.robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UsdFileCfg(
                usd_path="c:/Users/basti/source/repos/mobile-robotics-ws/assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd",
                activate_contact_sensors=True,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=False,
                    max_depenetration_velocity=1.0,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(1.0, 0.0, 0.78), 
            ),
            # --- PHYSICS (REV4: COMPLIANT) ---
            actuators={
                "legs": ImplicitActuatorCfg(
                    joint_names_expr=[".*_hip_.*", ".*_knee_.*", ".*_ankle_.*"],
                    stiffness=40.0,  # REV4 KEY: LOW STIFFNESS
                    damping=5.0,     # Standard Damping
                    friction=0.05,   # Non-zero friction
                ),
                "upper_body": ImplicitActuatorCfg(
                    joint_names_expr=["waist_.*", ".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_.*"],
                    stiffness=60.0,  # Slightly stiffer upper body
                    damping=5.0,
                    friction=0.05,
                ),
            },
        )

        # 1. Standard Ground Mesh (Generated) - Safer than 'plane' which crashed
        self.scene.terrain = TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="generator",
            terrain_generator=TerrainGeneratorCfg(
                size=(20.0, 20.0), 
                border_width=5.0,
                num_rows=1,
                num_cols=1,
                sub_terrains={"flat": mdp.MeshPlaneTerrainCfg(flat_patch=True)}
            ),
            debug_vis=False,
        )
        
        # 2. Test Environment (Cubes/Stairs) loaded as STATIC ASSET
        # This bypasses TerrainImporter logic and loads raw USD as RigidObject.
        self.scene.environment = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Environment",
            spawn=sim_utils.UsdFileCfg(
                usd_path="c:/Users/basti/source/repos/mobile-robotics-ws/g1_project/assets/test_env.usd",
                scale=(1.0, 1.0, 1.0),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
        )

        # --- LEGACY SENSOR (Required for Old Policy Playback) ---
        # Kept hidden to allow loading v11 policy without crash
        self.scene.height_scanner = RayCasterCfg(
            prim_path="{ENV_REGEX_NS}/Robot/torso_link/head_link",
            offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.0)),
            ray_alignment="yaw",
            pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
            debug_vis=False, # Hidden
            mesh_prim_paths=["/World/ClimbEnv"],
        )
        
        # 1. Livox Mid-360 (Approximation: 360 deg lidar)
        # Mounted on TORSO (Offset to Head Height)
        # Head attachment failed (immobile), so we use Torso as the moving reference frame.
        self.scene.livox_lidar = RayCasterCfg(
            prim_path="{ENV_REGEX_NS}/Robot/torso_link",
            offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.6)), # Z+0.6m ~ Head Height
            pattern_cfg=patterns.LidarPatternCfg(
                channels=16, 
                vertical_fov_range=(-30.0, 30.0), 
                horizontal_fov_range=(-180.0, 180.0),
                horizontal_res=2.0 
            ),
            debug_vis=True,
            mesh_prim_paths=["/World"], # Check ALL collisions
        )

        # 2. RealSense D435i (Approximation: Forward Grid)
        # Mounted on TORSO (Offset to Face Height)
        self.scene.realsense_depth = RayCasterCfg(
            prim_path="{ENV_REGEX_NS}/Robot/torso_link", 
            offset=RayCasterCfg.OffsetCfg(pos=(0.25, 0.0, 0.5)), # Offset forward and up
            pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[1.0, 0.6]),
            debug_vis=True,
            mesh_prim_paths=["/World"], # Check ALL collisions
        )
        self.scene.contact_forces = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True,
        )
