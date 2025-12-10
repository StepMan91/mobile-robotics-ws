import math

# Gait Profiles Configuration
# Defines limits and parameters for different locomotion states.

GAIT_PROFILES = {
    "WALK": {
        "description": "Strict walking on flat ground",
        "step_length": 0.25,      # Max 0.33m requested
        "max_knee_deg": 45.0,     # Strict limit
        "arm_amp": 0.1,           # Minimal sway
        "cycle_time": 1.0,
        "use_head_look": False
    },
    "CLIMB": {
        "description": "Climbing stairs - Permissive",
        "step_length": 0.25,
        "max_knee_deg": 110.0,    # Allow full bend for stairs
        "arm_amp": 0.4,           # Normal sway
        "cycle_time": 1.2,        # Slower
        "use_head_look": False
    },
    "WAIT": {
        "description": "Standing on Catwalk",
        "step_length": 0.0,
        "max_knee_deg": 10.0,     # Straight legs
        "arm_amp": 0.0,           # Frozen arms
        "cycle_time": 2.0,        # Slow breathing/looking
        "use_head_look": True     # Enable head animation
    }
}
