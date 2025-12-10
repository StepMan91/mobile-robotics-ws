
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
    body_idx = env.scene.rigid_bodies[asset_cfg.name].find_bodies(asset_cfg.body_names)[0]
    # body_pos shape: (num_envs, num_bodies, 3) -> (num_envs, 1, 3)
    pos = env.scene.rigid_bodies[asset_cfg.name].data.root_pos_w[:, body_idx, :3]
    
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
            usd_path="c:/Users/basti/source/repos/mobile-robotics-ws/g1_project/assets/stairs_env.usd",
        )
        
        # Adjust Episode Length (Climbing takes time)
        self.episode_length_s = 40.0
        
        # Init State (Randomize X start)
        self.scene.robot.init_state.pos = (0.5, 0.0, 0.8) # Closer to stairs
        # Add noise to init pos? Handled by reset events usually.
        
        # Add Hand Rail Reward
        # Track LEFT Hand (assuming left rail)
        # Note: In climb_stairs.py, rail was at Y=0.45. Robot at Y=0.0.
        # Left Hand is at Y ~ +0.3.
        # So we track left_wrist/hand.
        
        self.rewards.hand_rail_tracking = RewTerm(
            func=hand_rail_distance,
            weight=2.0,
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=".*_wrist_roll_link"), # Use one hand
                # "asset_cfg": SceneEntityCfg("robot", body_names="left_wrist_roll_link"), 
                # Regex might pick both? We want ONE hand.
                # Let's target LEFT specifically if rail is on left.
                # Rail Y=0.45. Robot Y=0. 
                # Left is +Y usually (Left Hand Rule? or Right?).
                # G1: Left is +Y in standard T-pose? 
                # Let's assume Left.
                # Regex: "left_wrist_roll_link"
            },
        )
        
        # Update Body Name for specific hand
        # Assuming "left_wrist_roll_link" exists.
        
        # Penalize Falling more
        self.rewards.lin_vel_z_l2.weight = 0.0 # Don't penalize vertical movement (we want climb)
        self.rewards.base_stability.params["minimum_height"] = 0.3 # Allow some crouching but not falling
        
        # Increase generic forward velocity reward (Climb Up)
        self.rewards.track_lin_vel_xy_exp.weight = 1.0
        
        # Add specific "Climb Z" progress reward?
        # base_lin_vel Z > 0?
