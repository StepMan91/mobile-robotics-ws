import math
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import (
    TerrainImporterCfg, 
    TerrainGeneratorCfg, 
    MeshPlaneTerrainCfg, 
    MeshRandomGridTerrainCfg, 
    MeshPyramidStairsTerrainCfg, 
    MeshInvertedPyramidStairsTerrainCfg
)
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

@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""
    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for the policy."""
        # Joint positions
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        # Joint velocities
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-1.5, n_max=1.5))
        # Base linear velocity
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        # Base angular velocity
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        # Projected gravity
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        # Velocity commands
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        # Height scan
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )
        # Actions
        actions = ObsTerm(func=mdp.last_action)

    policy: PolicyCfg = PolicyCfg()

@configclass
class ActionsCfg:
    """Action specifications for the environment."""
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True)

@configclass
class CommandsCfg:
    """Command specifications for the environment."""
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.02,
        rel_heading_envs=1.0,
        heading_command=True,
        heading_control_stiffness=0.5,
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-1.0, 1.0), lin_vel_y=(-1.0, 1.0), ang_vel_z=(-1.0, 1.0), heading=(-math.pi, math.pi)
        ),
    )

@configclass
class RewardsCfg:
    """Reward specifications for the environment."""
    # -- Task --
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp, weight=1.0, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp, weight=0.5, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    # -- Penalties --
    lin_vel_z_l2 = RewTerm(func=mdp.rewards.lin_vel_z_l2, weight=-2.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.rewards.ang_vel_xy_l2, weight=-0.05)
    dof_torques_l2 = RewTerm(func=mdp.rewards.joint_torques_l2, weight=-1.0e-5)
    dof_acc_l2 = RewTerm(func=mdp.rewards.joint_acc_l2, weight=-2.5e-7)
    action_rate_l2 = RewTerm(func=mdp.rewards.action_rate_l2, weight=-0.01)
    # feet_air_time removed as it is missing in this version
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_thigh_link|.*_calf_link")},
    )
    # -- Optional --
    flat_orientation_l2 = RewTerm(func=mdp.rewards.flat_orientation_l2, weight=-0.0)
    dof_pos_limits = RewTerm(func=mdp.rewards.joint_pos_limits, weight=-0.0)

@configclass
class EventCfg:
    """Configuration for events."""
    # Startup
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

    # Reset
    base_external_force_torque = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "lin_vel_range": (-1.0, 1.0),
            "ang_vel_range": (-1.0, 1.0),
        },
    )

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        },
    )

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        },
    )

from isaaclab.envs.mdp import time_out, root_height_below_minimum
from isaaclab.managers import TerminationTermCfg as TermTerm
from isaaclab.managers import CurriculumTermCfg as CurrTerm

@configclass
class CurriculumCfg:
    """Curriculum terms for the environment."""
    pass

@configclass
class TerminationsCfg:
    """Termination terms for the environment."""
    # Time out
    time_out = TermTerm(func=time_out, params={})
    # Base stability
    base_stability = TermTerm(func=root_height_below_minimum, params={"minimum_height": 0.2})

@configclass
class G1LocomotionEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the G1 locomotion environment."""
    
    # Scene settings
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=2.5)
    
    # Basic settings
    episode_length_s = 20.0
    decimation = 4
    
    # Observations
    observations: ObservationsCfg = ObservationsCfg()
    # Actions
    actions: ActionsCfg = ActionsCfg()
    # Events
    events: EventCfg = EventCfg()
    # Rewards
    rewards: RewardsCfg = RewardsCfg()
    # Terminations
    terminations: TerminationsCfg = TerminationsCfg()
    # Curriculum
    curriculum: CurriculumCfg = CurriculumCfg()
    # Commands
    commands: CommandsCfg = CommandsCfg()
    
    def __post_init__(self):
        """Post initialization."""
        super().__post_init__()
        
        # Viewer settings
        self.viewer.eye = [3.0, 3.0, 3.0]
        self.viewer.lookat = [0.0, 0.0, 0.0]
        
        # Simulation settings
        self.sim.dt = 0.005  # 200Hz
        self.sim.render_interval = 4  # 50Hz rendering
        
        # Define the robot
        self.scene.robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UsdFileCfg(
                usd_path="c:/Users/basti/source/repos/mobile-robotics-ws/assets/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd",
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
                    enabled_self_collisions=False,
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
                    ".*": 0.0, # Default to zero for now, need to tune
                },
            ),
            actuators={
                "legs": ImplicitActuatorCfg(
                    joint_names_expr=[".*_hip_.*", ".*_knee_.*", ".*_ankle_.*"],
                    stiffness=150.0,
                    damping=5.0,
                ),
                "arms": ImplicitActuatorCfg(
                    joint_names_expr=[".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_.*"],
                    stiffness=100.0,
                    damping=2.0,
                ),
                "torso": ImplicitActuatorCfg(
                    joint_names_expr=[".*_waist_.*"],
                    stiffness=200.0,
                    damping=5.0,
                ),
            },
        )

        # Define the terrain
        self.scene.terrain = TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="generator",
            terrain_generator=TerrainGeneratorCfg(
                seed=42,
                curriculum=True,
                difficulty_range=(0.0, 1.0),
                num_rows=10,
                num_cols=20,
                size=(20.0, 20.0),
                sub_terrains={
                    "flat": MeshPlaneTerrainCfg(
                        proportion=0.2,
                    ),
                    "rough": MeshRandomGridTerrainCfg(
                        proportion=0.2,
                        grid_width=0.5,
                        grid_height_range=(0.05, 0.1),
                        platform_width=2.0,
                    ),
                    "stairs_pyramid": MeshPyramidStairsTerrainCfg(
                        proportion=0.3,
                        step_height_range=(0.05, 0.2),
                        step_width=1.0,
                        platform_width=2.0,
                    ),
                    "stairs_stepped": MeshInvertedPyramidStairsTerrainCfg(
                        proportion=0.3,
                        step_height_range=(0.05, 0.2),
                        step_width=1.0,
                        platform_width=2.0,
                    ),
                },
            ),
            max_init_terrain_level=5,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.5, 0.5)),
            debug_vis=False,
        )

        # Define light
        self.scene.light = AssetBaseCfg(
            prim_path="/World/light",
            spawn=sim_utils.DistantLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
        )

        # Sensors
        self.scene.height_scanner = RayCasterCfg(
            prim_path="{ENV_REGEX_NS}/Robot/torso_link",
            offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
            attach_yaw_only=True,
            pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
            debug_vis=False,
            mesh_prim_paths=["/World/ground"],
        )
        self.scene.contact_forces = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True
        )
