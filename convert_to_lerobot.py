import json
import sys
import numpy as np
import cv2
from pathlib import Path

sys.path.insert(0, "C:/Users/parks/lerobot/src")
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# ── 설정 ──────────────────────────────────────────────────────────────
SRC_DIR    = Path("C:/Users/parks/esp32_robot_arm/datasets")
DST_ROOT   = Path("C:/Users/parks/esp32_robot_arm/lerobot_dataset")
REPO_ID    = "local/robot_arm"
TASK       = "pick and place"
NUM_JOINTS = 6
FPS        = 30

# 🔥 카메라 2개로 확장
FEATURES = {
    "observation.images.wrist": {
        "dtype": "video",
        "shape": (480, 640, 3),
        "names": ["height", "width", "channels"],
    },
    "observation.images.full": {   # ✅ 추가됨
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

        frames_angles = data["frames"]

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

            angles = np.array(frames_angles[i], dtype=np.float32)

            dataset.add_frame({
                "task": TASK,
                "observation.images.wrist": rgb1,
                "observation.images.full": rgb2,   # ✅ 추가
                "observation.state": angles,
                "action": angles,
            })

        cap_wrist.release()
        cap_full.release()

        dataset.save_episode()
        print("       저장 완료")

    dataset.finalize()
    print(f"\n[DONE] 변환 완료 → {DST_ROOT}")


if __name__ == "__main__":
    main()