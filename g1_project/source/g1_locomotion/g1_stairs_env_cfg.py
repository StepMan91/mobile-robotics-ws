
import math
import torch
from isaaclab.utils import configclass
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.terrains import TerrainImporterCfg
import isaaclab.envs.mdp as mdp

from .g1_env_cfg import G1LocomotionEnvCfg

# Custom Reward Function
def hand_rail_distance(env, asset_cfg: SceneEntityCfg, rail_start=(3.0, 0.45, 0.9), rail_end=(6.75, 0.45, 3.15), sigma=0.5):
    # Extract hand positions
    # asset_cfg should point to left_hand or right_hand link
    # asset_cfg should point to left_hand or right_hand link
    # env.scene[asset_cfg.name] gives the Articulation (or RigidObject)
    asset = env.scene[asset_cfg.name]
    body_idx = asset.find_bodies(asset_cfg.body_names)[0] # Get indices
    # body_pos_w shape: (num_envs, num_bodies, 3). We take the first matched body.
    # Note: body_idx is likely a list or tensor of indices.
    target_idx = body_idx[0] 
    pos = asset.data.body_pos_w[:, target_idx, :3]
    
    # Line Segment Math
    # P = Start, Q = End. AB = Q-P.
    # Dist(X) = ||Cross(X-P, Q-P)|| / ||Q-P|| (Infinite line)
    # For segment, we clamp t.
    
    p = torch.tensor(rail_start, device=env.device)
    q = torch.tensor(rail_end, device=env.device)
    pq = q - p
    len_sq = torch.sum(pq**2)
    
    # Project X-P onto PQ
    xp = pos - p # (N, 3)
    t = torch.sum(xp * pq, dim=1) / len_sq # (N,)
    t = torch.clamp(t, 0.0, 1.0)
    
    # Nearest point
    closest = p + t.unsqueeze(1) * pq
    dist = torch.norm(pos - closest, dim=1)
    
    return torch.exp(-dist / sigma)

@configclass
class G1StairsEnvCfg(G1LocomotionEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        
        # Override Terrain to use Exported USD
        self.scene.terrain = TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="usd",
            usd_path=r"c:/Users/basti/source/repos/mobile-robotics-ws/g1_project/assets/stairs_env.usd",
        )
        
        # Adjust Episode Length (Climbing takes time)
        self.episode_length_s = 40.0
        
        # Init State (Randomize X start)
        # Stairs started at [2.0, 2.0, 0.0]. We want to be in front (-X).
        # X=1.0 is 1m in front of first step. Y=2.0 is centered.
        self.scene.robot.init_state.pos = (1.0, 2.0, 0.8) 
        
        # Add Hand Rail Reward
        # Track LEFT Hand (assuming left rail)
        # Rail is at Y=2.0 + WIDTH/2 = 2.5 (Left side if facing +X?)
        # Or Y=2.0 - WIDTH/2 = 1.5.
        # G1 in T-Pose: Left is +Y.
        # We target Y=2.5 line.
        
        self.rewards.hand_rail_tracking = RewTerm(
            func=hand_rail_distance,
            weight=2.0,
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=".*_wrist_roll_link"), 
                "rail_start": (2.0, 2.5, 0.9 + 0.15), # Approx start of rail (World Coords)
                "rail_end": (2.0 + 3.75, 2.5, 3.15), # Run ~3.75m
            },
        )
        
        # Update Body Name for specific hand
        # Assuming "left_wrist_roll_link" exists.
        
        # Penalize Falling more
        self.rewards.lin_vel_z_l2.weight = 0.0 # Don't penalize vertical movement (we want climb)
        self.terminations.base_stability.params["minimum_height"] = 0.3 # Allow some crouching but not falling
        
        # Increase generic forward velocity reward (Climb Up)
        self.rewards.track_lin_vel_xy_exp.weight = 1.0
        
        # STRICT JOINT LIMITS (User request)
        # Prevent splits by penalizing limits heavily
        self.rewards.dof_pos_limits = RewTerm(func=mdp.rewards.joint_pos_limits, weight=-10.0)

        # Custom Penalty for Hip Abduction (Splits)
        # We can use joint_pos_limits but maybe stricter limits for hip_roll?
        # For now, rely on dof_pos_limits with high weight.
        
        # DEBUG: Disable Height Scanner to find paths
        # self.scene.height_scanner.mesh_prim_paths = ["/World/ground/World/defaultGroundPlane/Environment/Geometry"]
        del self.scene.height_scanner
        del self.observations.policy.height_scan
        
        # Add RayCaster for visual debug?
        # self.scene.height_scanner.debug_vis = True

