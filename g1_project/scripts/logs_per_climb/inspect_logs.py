import os
import glob
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
import datetime

def inspect_logs(log_dir):
    # Search recursively for all tfevents files
    event_files = glob.glob(os.path.join(log_dir, "**", "events.out.tfevents.*"), recursive=True)
    report_lines = []
    report_lines.append(f"Found {len(event_files)} event files in {log_dir}")
    
    print(f"Scanning {len(event_files)} files...")

    for event_file in event_files:
        try:
            # size_guidance={'scalars': 0} loads all scalar events
            event_acc = EventAccumulator(event_file, size_guidance={'scalars': 0})
            event_acc.Reload()
            
            # basic tags check
            tags = event_acc.Tags()['scalars']
            if not tags:
                report_lines.append(f"\nFile: {os.path.basename(event_file)}\n  -> No scalars found.")
                continue

            # check first and last step
            first_step = float('inf')
            last_step = float('-inf')
            count = 0
            
            # Identify a common tag to use for step counting, e.g. 'Train/mean_reward' or just the first available
            sample_tag = tags[0]
            if 'Episode_Reward/hand_rail' in tags: # Prefer this tag if available
                sample_tag = 'Episode_Reward/hand_rail'
            elif 'Train/mean_reward' in tags:
                sample_tag = 'Train/mean_reward'
                
            events = event_acc.Scalars(sample_tag)
            
            if events:
                first_step = events[0].step
                last_step = events[-1].step
                count = len(events)
                start_time = datetime.datetime.fromtimestamp(events[0].wall_time).strftime('%Y-%m-%d %H:%M:%S')
                end_time = datetime.datetime.fromtimestamp(events[-1].wall_time).strftime('%Y-%m-%d %H:%M:%S')
                
                marker = ""
                if last_step > 1200:
                    marker = "  <=== POTENTIAL MATCH"

                report_lines.append(f"\nFile: {event_file}{marker}")
                report_lines.append(f"  -> {count} entries for tag '{sample_tag}'")
                report_lines.append(f"  -> Step Range: {first_step} to {last_step}")
                report_lines.append(f"  -> Time Range: {start_time} to {end_time}")
            else:
                 report_lines.append(f"\nFile: {os.path.basename(event_file)}\n  -> Tag found but no events.")

        except Exception as e:
            report_lines.append(f"\nFile: {os.path.basename(event_file)}\n  -> Error reading file: {e}")

    with open("log_inspection_report.txt", "w") as f:
        f.write("\n".join(report_lines))
    
    print("Inspection complete. Report saved to log_inspection_report.txt")

if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    inspect_logs(current_dir)
