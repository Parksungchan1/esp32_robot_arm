"""
기존 데이터셋 angles.json에 phase 정보를 자동 추가.
백업: angles.json.bak 생성 후 frames를 7차원으로 확장.

Phase 판별 기준:
  1: 잡기/들기  — 시작 ~ S1이 100° 초과 전
  2: 입쪽 이동  — S1 100°+ ~ S1 피크
  3: 복귀       — S1 피크 ~ S1 90° 이하 복귀
  4: 내려놓기   — S1 복귀 + S6 < 100°
"""

import json
import shutil
from pathlib import Path

DATASET_DIR = Path("C:/Users/parks/esp32_robot_arm/datasets")


def moving_avg(arr, w=9):
    result = []
    for i in range(len(arr)):
        s = max(0, i - w // 2)
        e = min(len(arr), i + w // 2 + 1)
        result.append(sum(arr[s:e]) / (e - s))
    return result


def detect_phases(frames):
    n = len(frames)
    s1 = moving_avg([f[0] for f in frames], 9)
    s6 = moving_avg([f[5] for f in frames], 5)

    # S6가 처음 100° 이상 (그립 완료)
    grab_frame = next((i for i in range(n) if s6[i] >= 100), n // 4)

    # S1 피크 (그립 이후)
    s1_after = s1[grab_frame:]
    peak_val = max(s1_after) if s1_after else max(s1)
    peak_frame = grab_frame + s1_after.index(peak_val) if s1_after else s1.index(peak_val)

    # Phase 1→2: 그립 이후 S1이 처음 100° 초과
    p1_to_p2 = next(
        (i for i in range(grab_frame, peak_frame) if s1[i] >= 100),
        peak_frame
    )

    # Phase 2→3: S1 피크 직후
    p2_to_p3 = peak_frame

    # Phase 3→4: S1 복귀(<=90°) + S6 하강(<100°)
    p3_to_p4 = next(
        (i for i in range(p2_to_p3, n) if s1[i] <= 90 and s6[i] < 100),
        None
    )
    if p3_to_p4 is None:
        p3_to_p4 = next(
            (i for i in range(p2_to_p3, n) if s6[i] < 100),
            n - 1
        )

    phases = []
    for i in range(n):
        if i >= p3_to_p4:
            phases.append(4)
        elif i >= p2_to_p3:
            phases.append(3)
        elif i >= p1_to_p2:
            phases.append(2)
        else:
            phases.append(1)

    return phases, dict(grab=grab_frame, p1_to_p2=p1_to_p2,
                        peak=peak_frame, p2_to_p3=p2_to_p3, p3_to_p4=p3_to_p4)


def main():
    episodes = sorted(DATASET_DIR.glob("episode_*"))
    print(f"총 {len(episodes)}개 에피소드 처리\n")

    for ep_dir in episodes:
        angles_path = ep_dir / "angles.json"
        if not angles_path.exists():
            print(f"[SKIP] {ep_dir.name} — angles.json 없음")
            continue

        with open(angles_path) as f:
            data = json.load(f)

        frames = data["frames"]

        if len(frames[0]) >= 7:
            print(f"[SKIP] {ep_dir.name} — 이미 phase 포함됨")
            continue

        phases, t = detect_phases(frames)
        counts = {p: phases.count(p) for p in [1, 2, 3, 4]}

        print(f"{ep_dir.name} ({len(frames)}f): "
              f"P1={counts[1]}f P2={counts[2]}f P3={counts[3]}f P4={counts[4]}f  "
              f"| grab@{t['grab']} 1→2@{t['p1_to_p2']} peak@{t['peak']} "
              f"2→3@{t['p2_to_p3']} 3→4@{t['p3_to_p4']}")

        shutil.copy(angles_path, angles_path.with_suffix(".json.bak"))

        for i, frame in enumerate(frames):
            frames[i] = list(frame) + [phases[i]]

        data["frames"] = frames
        with open(angles_path, "w") as f:
            json.dump(data, f)

    print("\n[완료] 모든 에피소드 phase 추가됨. 원본 백업: .json.bak")


if __name__ == "__main__":
    main()
