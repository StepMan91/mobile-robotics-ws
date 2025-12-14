from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
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
from isaaclab.managers import ManagerTermBase, RewardTermCfg

# --- CUSTOM REWARDS ---

def hand_rail_distance(env, asset_cfg: SceneEntityCfg, rail_start: tuple, rail_end: tuple):
    """Reward for keeping the hand close to the rail line."""
    # Extract hand position
    # asset_cfg should target the specific hand link (e.g., left_wrist_roll_link)
    hand_pos = env.scene[asset_cfg.name].data.body_pos_w[:, asset_cfg.body_ids[0], :]  # [NumEnvs, 3]
    
    # Define Rail Line Segment
    p1 = torch.tensor(rail_start, device=env.device)
    p2 = torch.tensor(rail_end, device=env.device)
    
    # Vector form: Line = p1 + t * (p2 - p1)
    line_vec = p2 - p1
    line_len_sq = torch.sum(line_vec**2)
    
    # Project hand_pos onto line to find closest point
    # t = dot(hand - p1, line_vec) / line_len_sq
    t = torch.sum((hand_pos - p1) * line_vec, dim=1) / line_len_sq
    t = torch.clamp(t, 0.0, 1.0) # Clamp to segment
    
    closest_point = p1 + t.unsqueeze(1) * line_vec
    
    distance = torch.norm(hand_pos - closest_point, dim=1)
    
    return torch.exp(-distance * 5.0) # Sharp reward for being close

def torso_upright_reward(env, asset_cfg: SceneEntityCfg):
    """Penalize torso roll/pitch deviation from vertical."""
    # Root rotation (Quat: w, x, y, z)
    root_quat = env.scene[asset_cfg.name].data.root_quat_w
    
    # Convert to Projected Gravity or Z-vector
    # Simplest: Project [0,0,1] local to world? Or World Z to Local?
    # projected_gravity in obs uses inverse rotation.
    
    # We want local Z to be close to world Z.
    # Rotated Z axis:
    # row 2 of rotation matrix.
    # z_axis_z = 1 - 2(x^2 + y^2) 
    # We want this to be 1.0
    
    x = root_quat[:, 1]
    y = root_quat[:, 2]
    
    # Deviation metric: x^2 + y^2 shoud be 0
    deviation = x**2 + y**2
    return torch.exp(-deviation * 10.0)

def climb_progress_reward(env, command_name: str):
    """Reward for moving Forward (+X) and Up (+Z)."""
    # Simply track root velocity
    root_vel = env.scene["robot"].data.root_lin_vel_w
    
    vel_x = root_vel[:, 0]
    vel_z = root_vel[:, 2]
    
    # Reward both
    return vel_x + vel_z * 2.0 # Emphasize Upward

def feet_air_time(env, sensor_cfg: SceneEntityCfg, command_name: str, threshold: float):
    """Reward for feet being in the air (steps)."""
    sensor = env.scene[sensor_cfg.name]
    # air_time is [NumEnvs, NumBodies]
    air_time = sensor.data.current_air_time[:, sensor_cfg.body_ids]
    # Reward air time up to a threshold (step duration)
    return torch.sum(torch.clamp(air_time, max=threshold), dim=1)

def feet_slide(env, sensor_cfg: SceneEntityCfg, asset_cfg: SceneEntityCfg):
    """Penalize horizontal velocity when feet are in contact."""
    # Contact: Force > 1.0
    contacts = env.scene[sensor_cfg.name].data.net_forces_w_history[:, 0, sensor_cfg.body_ids].norm(dim=-1) > 1.0
    
    # Feet Vel XY
    body_vel = env.scene[asset_cfg.name].data.body_lin_vel_w[:, asset_cfg.body_ids, :2]
    vel_norm = torch.norm(body_vel, dim=-1)
    
    # Penalize velocity where contact exists
    return torch.sum(vel_norm * contacts.float(), dim=1)

def look_at_stairs(env, asset_cfg: SceneEntityCfg):
    """Reward head facing the stairs (+X)."""
    # Head forward vector
    # Project Local X of head to World
    # Need head body index.
    # Assuming body_ids[0] is head/torso?
    # Better: Use Root Quaternion if head isn't tracked separately?
    # Or use asset_cfg to pick "head_link" if available.
    # Let's use Root for now as "General Gaze".
    root_quat = env.scene[asset_cfg.name].data.root_quat_w
    
    # Forward vector from quat:
    # 2(xy + wz), 1 - 2(y^2 + z^2), ... 
    # forward X = 1 - 2(y^2 + z^2)
    # forward Y = 2(xy + wz)
    
    # We want Forward X to be 1.0 (looking at stairs at X+)
    # We want Flat (Z=0) ?
    
    # Use built-in math utils or simplify.
    # q = (w, x, y, z)
    w, x, y, z = root_quat[:, 0], root_quat[:, 1], root_quat[:, 2], root_quat[:, 3]
    
    forward_x = 1 - 2 * (y**2 + z**2)
    
    return torch.clamp(forward_x, min=0.0)

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
        
        # [NEW] PERCEPTION
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )

    policy: PolicyCfg = PolicyCfg()

@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True)

@configclass
class RewardsCfg:
    # -- Task --
    hand_rail = RewTerm(
        func=hand_rail_distance,
        weight=2.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_wrist_roll_link"), 
            "rail_start": (3.0, 0.5, 1.05),
            "rail_end": (3.0 + 3.75, 0.5, 3.3), 
        }
    )
    
    climb_progress = RewTerm(
        func=climb_progress_reward,
        weight=1.5,
        params={"command_name": "base_velocity"}
    )
    
    # [NEW] GAIT REWARDS
    feet_air_time = RewTerm(
        func=feet_air_time,
        weight=1.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="left_ankle_roll_link|right_ankle_roll_link"),
            "command_name": "base_velocity",
            "threshold": 0.5,
        }
    )
    
    feet_slide = RewTerm(
        func=feet_slide,
        weight=-1.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="left_ankle_roll_link|right_ankle_roll_link"),
            "asset_cfg": SceneEntityCfg("robot", body_names="left_ankle_roll_link|right_ankle_roll_link"),
        }
    )
    
    look_at_stairs = RewTerm(
        func=look_at_stairs,
        weight=0.5,
        params={"asset_cfg": SceneEntityCfg("robot")}
    )
    
    stability = RewTerm(
        func=torso_upright_reward,
        weight=2.0, # Increased for staying upright
        params={"asset_cfg": SceneEntityCfg("robot")}
    )
    
    # -- Penalties --
    dof_torques_l2 = RewTerm(func=mdp.rewards.joint_torques_l2, weight=-1.0e-5)
    action_rate_l2 = RewTerm(func=mdp.rewards.action_rate_l2, weight=-0.05) # Increased to suppress jitter
    
    # STRICT JOINT LIMITS
    dof_pos_limits = RewTerm(func=mdp.rewards.joint_pos_limits, weight=-10.0)

@configclass
class CommandsCfg:
    # No commands, just Climb Task
    null_command = mdp.NullCommandCfg()

@configclass
class EventCfg:
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.8, 0.8),
            "dynamic_friction_range": (0.6, 0.6),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )
    
    # Init State: Start on Blue Path (X=0 to 2)
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "pose_range": {"x": (0.5, 1.5), "y": (-0.2, 0.2), "yaw": (-0.1, 0.1)}, # Face forward (+X)
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
    # Fall detection
    base_stability = TermTerm(func=mdp.root_height_below_minimum, params={"minimum_height": 0.3})

@configclass
class G1ClimbEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the G1 Climbing environment."""
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
                # Use absolute path to G1 USD
                usd_path="c:/Users/basti/source/repos/mobile-robotics-ws/assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd",
                activate_contact_sensors=True,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=False,
                    max_depenetration_velocity=1.0,
                ),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False, # Stability
                    solver_position_iteration_count=4,
                    solver_velocity_iteration_count=0,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(1.0, 0.0, 0.78), # Slightly Lower to ensure ground contact
                rot=(1.0, 0.0, 0.0, 0.0),
            ),
            actuators={
                "legs": ImplicitActuatorCfg(
                    joint_names_expr=[".*_hip_.*", ".*_knee_.*", ".*_ankle_.*"],
                    stiffness=200.0, # Increased for support
                    damping=5.0,
                ),
                "arms": ImplicitActuatorCfg(
                    joint_names_expr=[".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_.*"],
                    stiffness=100.0, damping=2.0,
                ),
                "torso": ImplicitActuatorCfg(
                    joint_names_expr=["waist_.*"],
                    stiffness=200.0, damping=5.0,
                ),
            },
        )

        self.scene.terrain = TerrainImporterCfg(
            prim_path="/World/ClimbEnv",
            terrain_type="usd",
            usd_path="c:/Users/basti/source/repos/mobile-robotics-ws/g1_project/assets/climb_world.usd",
        )

        # SENSORS (Added for Perception and Gait Rewards)
        self.scene.height_scanner = RayCasterCfg(
            prim_path="{ENV_REGEX_NS}/Robot/torso_link",
            offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
            attach_yaw_only=True,
            pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
            debug_vis=False,
            mesh_prim_paths=["/World/ClimbEnv"], # Scan the custom terrain
        )
        self.scene.contact_forces = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True,
            debug_vis=False,
        )
