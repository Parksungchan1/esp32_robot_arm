"""
수집된 데이터셋을 LeRobot 포맷으로 변환

사용법:
  python convert_to_lerobot.py
"""

import json
import sys
import numpy as np
import cv2
from pathlib import Path

# LeRobot 소스 경로 추가
sys.path.insert(0, "C:/Users/parks/lerobot/src")

from lerobot.datasets.lerobot_dataset import LeRobotDataset

# ── 설정 ──────────────────────────────────────────────────────────────
SRC_DIR    = Path("C:/Users/parks/esp32_robot_arm/datasets")
DST_ROOT   = Path("C:/Users/parks/esp32_robot_arm/lerobot_dataset")
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
    "observation.state": {
        "dtype": "float32",
        "shape": (NUM_JOINTS,),
        "names": ["s1", "s2", "s3", "s4", "s5", "s6"],
    },
    "action": {
        "dtype": "float32",
        "shape": (NUM_JOINTS,),
        "names": ["s1", "s2", "s3", "s4", "s5", "s6"],
    },
}


def main():
    episodes = sorted(SRC_DIR.glob("episode_*"))
    if not episodes:
        print("[ERROR] 데이터셋 폴더에 에피소드가 없습니다.")
        sys.exit(1)

    print(f"[INFO] 에피소드 {len(episodes)}개 발견")

    # 기존 폴더 있으면 삭제
    if DST_ROOT.exists():
        import shutil
        shutil.rmtree(DST_ROOT)
        print(f"[INFO] 기존 폴더 삭제: {DST_ROOT}")

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
        video_path  = ep_dir / "cam_wrist.mp4"

        if not angles_path.exists() or not video_path.exists():
            print(f"[SKIP] {ep_dir.name} — 파일 없음")
            continue

        with open(angles_path) as f:
            data = json.load(f)

        frames_angles = data["frames"]

        cap = cv2.VideoCapture(str(video_path))
        total_video_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        n_frames = min(len(frames_angles), total_video_frames)

        print(f"[CONV] {ep_dir.name}  {n_frames}프레임 ({n_frames/FPS:.1f}초)")

        for i in range(n_frames):
            ret, bgr = cap.read()
            if not ret:
                break

            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            angles = np.array(frames_angles[i], dtype=np.float32)

            dataset.add_frame({
                "task": TASK,
                "observation.images.wrist": rgb,
                "observation.state": angles,
                "action": angles,
            })

        cap.release()
        dataset.save_episode()
        print(f"       저장 완료")

    dataset.finalize()
    print(f"\n[DONE] 변환 완료 → {DST_ROOT}")
    print(f"       총 에피소드: {len(episodes)}개")


if __name__ == "__main__":
    main()
