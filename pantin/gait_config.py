import math

# Gait Profiles Configuration
# Defines limits and parameters for different locomotion states.

GAIT_PROFILES = {
    "WALK": {
        "description": "Strict walking on flat ground",
        "step_length": 0.25,      
        "max_knee_deg": 45.0,     
        "arm_amp": 0.1,           
        "cycle_time": 1.0,
        "use_head_look": False,
        "swing_height": 0.1       # Low lift for walk
    },
    "CLIMB": {
        "description": "Climbing stairs - Permissive",
        "step_length": 0.25,
        "max_knee_deg": 110.0,    
        "arm_amp": 0.4,           
        "cycle_time": 1.2,        
        "use_head_look": False,
        "swing_height": 0.35      # High lift to clear 0.15m steps + margin
    },
    "WAIT": {
        "description": "Standing on Catwalk",
        "step_length": 0.0,
        "max_knee_deg": 10.0,     
        "arm_amp": 0.0,           
        "cycle_time": 2.0,        
        "use_head_look": True,
        "swing_height": 0.0
    }
}
