import json
from pathlib import Path

DATASET_DIR = Path("C:/Users/parks/esp32_robot_arm/datasets")
JUMP_THRESHOLD = 20

def check_episodes():
    episodes = sorted(DATASET_DIR.glob("episode_*"))
    if not episodes:
        print("No episodes found.")
        return 0, []

    bad = []
    for ep_dir in episodes:
        angles_file = ep_dir / "angles.json"
        if not angles_file.exists():
            continue
        with open(angles_file) as f:
            data = json.load(f)
        frames = data["frames"]
        issues = []
        for i in range(1, len(frames)):
            for s in range(6):
                delta = abs(frames[i][s] - frames[i-1][s])
                if delta >= JUMP_THRESHOLD:
                    issues.append((i, s+1, frames[i-1][s], frames[i][s], delta))
        if issues:
            bad.append((ep_dir.name, len(frames), issues))

    total = len(episodes)
    print(f"\n=== Dataset Check: {total} episodes ===")

    if not bad:
        print("All episodes OK.")
    else:
        print(f"Issues found in {len(bad)}/{total} episodes:\n")
        for ep_name, total_frames, issues in bad:
            servo_counts = {}
            for _, s, _, _, _ in issues:
                servo_counts[s] = servo_counts.get(s, 0) + 1
            servo_summary = " ".join(f"S{s}:{cnt}x" for s, cnt in sorted(servo_counts.items()))
            worst = sorted(issues, key=lambda x: x[4], reverse=True)[:2]
            worst_str = " | ".join(f"S{s} fr{fr}: {p}->{c}" for fr, s, p, c, _ in worst)
            print(f"  [{ep_name}] {len(issues)} jumps ({servo_summary}) -- {worst_str}")

    print(f"\nClean: {total - len(bad)}/{total}")
    return total, bad

if __name__ == "__main__":
    check_episodes()
