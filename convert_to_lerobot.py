import json
import sys
import numpy as np
import cv2
from pathlib import Path

sys.path.insert(0, "C:/Users/parks/lerobot/src")
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# ── 설정 ──────────────────────────────────────────────────────────────
SRC_DIR    = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("C:/Users/parks/esp32_robot_arm/datasets")
DST_ROOT   = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("C:/Users/parks/esp32_robot_arm/lerobot_dataset")
REPO_ID    = "local/robot_arm"
TASK       = "pick and place"
NUM_JOINTS = 6
FPS        = 30

FEATURES = {
    "observation.images.wrist": {
        "dtype": "video",
        "shape": (480, 640, 3),
        "names": ["height", "width", "channels"],
    },
    "observation.images.full": {
        "dtype": "video",
        "shape": (480, 640, 3),
        "names": ["height", "width", "channels"],
    },
    "observation.state": {
        "dtype": "float32",
        "shape": (NUM_JOINTS + 1,),  # 관절 6개 + phase 1
        "names": ["s1", "s2", "s3", "s4", "s5", "s6", "phase"],
    },
    "action": {
        "dtype": "float32",
        "shape": (NUM_JOINTS,),
        "names": ["s1", "s2", "s3", "s4", "s5", "s6"],
    },
}


def filter_s6_noise(frames, jump_threshold=20, s6_threshold=1):
    # S6(조이스틱) 이동 프레임에서 S1~S5 ADC 노이즈 제거
    frames = [list(f) for f in frames]
    for i in range(1, len(frames)):
        s6_delta = abs(frames[i][5] - frames[i-1][5])
        if s6_delta >= s6_threshold:
            for s in range(5):
                if abs(frames[i][s] - frames[i-1][s]) >= jump_threshold:
                    frames[i][s] = frames[i-1][s]
    return frames


def filter_spikes(frames, threshold=20):
    # 단독 서보 단일 프레임 ADC 스파이크 제거
    frames = [list(f) for f in frames]
    for i in range(1, len(frames) - 1):
        for s in range(6):
            delta_prev = abs(frames[i][s] - frames[i-1][s])
            delta_next = abs(frames[i][s] - frames[i+1][s])
            if delta_prev >= threshold and delta_next >= threshold:
                frames[i][s] = frames[i-1][s]
    return frames


def main():
    episodes = sorted(SRC_DIR.glob("episode_*"))
    if not episodes:
        print("[ERROR] 데이터셋 폴더에 에피소드가 없습니다.")
        sys.exit(1)

    print(f"[INFO] 에피소드 {len(episodes)}개 발견")

    if DST_ROOT.exists():
        import shutil
        shutil.rmtree(DST_ROOT)

    dataset = LeRobotDataset.create(
        repo_id=REPO_ID,
        fps=FPS,
        features=FEATURES,
        root=DST_ROOT,
        robot_type="custom_6dof",
        use_videos=True,
        vcodec="h264",
    )

    for ep_num, ep_dir in enumerate(episodes):
        angles_path = ep_dir / "angles.json"
        wrist_video = ep_dir / "cam_wrist.mp4"
        full_video  = ep_dir / "cam_full.mp4"   # ✅ 추가

        if not (angles_path.exists() and wrist_video.exists() and full_video.exists()):
            print(f"[SKIP] {ep_dir.name} — 파일 없음")
            continue

        with open(angles_path) as f:
            data = json.load(f)

        frames_angles = filter_s6_noise(data["frames"])
        frames_angles = filter_spikes(frames_angles)

        cap_wrist = cv2.VideoCapture(str(wrist_video))
        cap_full  = cv2.VideoCapture(str(full_video))  # ✅ 추가

        total_frames = min(
            len(frames_angles),
            int(cap_wrist.get(cv2.CAP_PROP_FRAME_COUNT)),
            int(cap_full.get(cv2.CAP_PROP_FRAME_COUNT))
        )

        print(f"[CONV] {ep_dir.name}  {total_frames}프레임")

        for i in range(total_frames):
            ret1, bgr1 = cap_wrist.read()
            ret2, bgr2 = cap_full.read()

            if not ret1 or not ret2:
                break

            rgb1 = cv2.cvtColor(bgr1, cv2.COLOR_BGR2RGB)
            rgb2 = cv2.cvtColor(bgr2, cv2.COLOR_BGR2RGB)

            frame_data = frames_angles[i]
            joints = np.array(frame_data[:6], dtype=np.float32)
            phase  = float(frame_data[6]) if len(frame_data) >= 7 else 1.0
            state  = np.array(list(frame_data[:6]) + [phase], dtype=np.float32)

            dataset.add_frame({
                "task": TASK,
                "observation.images.wrist": rgb1,
                "observation.images.full": rgb2,
                "observation.state": state,   # 7차원 (관절6 + phase)
                "action": joints,             # 6차원 (관절만)
            })

        cap_wrist.release()
        cap_full.release()

        dataset.save_episode()
        print("       저장 완료")

    dataset.finalize()
    print(f"\n[DONE] 변환 완료 → {DST_ROOT}")


if __name__ == "__main__":
    main()