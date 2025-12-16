
import torch
import os

checkpoint_path = r"c:\Users\basti\source\repos\mobile-robotics-ws\g1_project\scripts\logs_per_climb_rev3_7k\model_2200.pt"

if not os.path.exists(checkpoint_path):
    print(f"Error: File not found {checkpoint_path}")
else:
    print(f"Loading {checkpoint_path}...")
    state = torch.load(checkpoint_path, map_location="cpu")
    print("\nKeys in checkpoint:")
    for key in state.keys():
        print(f" - {key}")

    if "model_state_dict" in state:
        print("\nKeys in model_state_dict:")
        msd = state["model_state_dict"]
        # Print first 20 keys
        for i, key in enumerate(msd.keys()):
            if i < 20:
                print(f" - {key}: {msd[key].shape}")
            else:
                print(" ... (truncated)")
                break
        
        # Check for normalization explicit keys
        print("\nChecking for normalization statistics:")
        norm_keys = [k for k in msd.keys() if "running_mean" in k or "running_var" in k or "norm" in k]
        if norm_keys:
            for k in norm_keys:
                print(f" - {k}")
        else:
            print(" [WARNING] No normalization keys found in model_state_dict!")
