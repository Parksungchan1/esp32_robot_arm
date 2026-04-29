"""
데이터 수집 스크립트
  Space : 에피소드 시작 / 종료
  d     : 마지막 에피소드 삭제 (잘못 찍었을 때)
  q     : 전체 종료
"""

import cv2
import serial
import threading
import time
import json
import sys
import shutil
from pathlib import Path

# ── 설정 ──────────────────────────────────────────────────────────────
COM_PORT     = "COM6"
BAUD_RATE    = 115200
FPS          = 30
FRAME_MS     = int(1000 / FPS)
DATASET_DIR  = Path("C:/Users/parks/esp32_robot_arm/datasets")

# ── 카메라 자동 탐지 ──────────────────────────────────────────────────
def find_webcam():
    """연결된 USB 웹캠 탐지 — 열린 VideoCapture 객체 반환"""
    for i in range(6):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None and frame.std() > 10:
                print(f"[INFO] 웹캠 발견: index {i}")
                return cap
            cap.release()
    return None

# ── 시리얼 각도 리더 (백그라운드 스레드) ──────────────────────────────
class AngleReader:
    def __init__(self, port, baud):
        self.angles = [90, 90, 90, 90, 90, 80]
        self._lock   = threading.Lock()
        self._running = True
        self._ser = serial.Serial(port, baud, timeout=1)
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._running:
            try:
                line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("s1:"):
                    vals = [int(p.split(":")[1]) for p in line.split(",")]
                    if len(vals) == 6:
                        with self._lock:
                            self.angles = vals
            except Exception:
                pass

    def get(self):
        with self._lock:
            return list(self.angles)

    def close(self):
        self._running = False
        self._ser.close()


# ── 에피소드 저장 ─────────────────────────────────────────────────────
def finish_episode(ep_dir, vw, angle_log, frame_count):
    vw.release()
    with open(ep_dir / "angles.json", "w") as f:
        json.dump({"fps": FPS, "frames": angle_log}, f)
    print(f"[SAVE] {ep_dir.name}  —  {frame_count}프레임 ({frame_count/FPS:.1f}초)")


def delete_last(ep_idx, last_ep_dir):
    if last_ep_dir and last_ep_dir.exists():
        shutil.rmtree(last_ep_dir)
        print(f"[DEL]  {last_ep_dir.name} 삭제됨")
        return ep_idx - 1
    print("[WARN] 삭제할 에피소드 없음")
    return ep_idx


# ── 메인 ─────────────────────────────────────────────────────────────
def main():
    DATASET_DIR.mkdir(parents=True, exist_ok=True)

    ep_idx     = len(sorted(DATASET_DIR.glob("episode_*")))
    last_saved = None

    # 시리얼
    print(f"[INFO] 시리얼 연결 중... {COM_PORT}")
    try:
        reader = AngleReader(COM_PORT, BAUD_RATE)
    except Exception as e:
        print(f"[ERROR] 시리얼 연결 실패: {e}")
        sys.exit(1)
    time.sleep(0.5)
    print(f"[INFO] 시리얼 OK  초기 각도: {reader.get()}")

    # 카메라 자동 탐지
    cap = find_webcam()
    if cap is None:
        print("[ERROR] USB 웹캠을 찾을 수 없습니다.")
        reader.close()
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FPS, FPS)

    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[INFO] 카메라 {W}x{H} @ {FPS}fps")
    print(f"[INFO] 저장 경로: {DATASET_DIR}")
    print()
    print("  Space : 에피소드 시작 / 종료")
    print("  d     : 마지막 에피소드 삭제")
    print("  q     : 전체 종료")
    print()

    recording   = False
    vw          = None
    angle_log   = []
    frame_count = 0
    ep_dir      = None
    fourcc      = cv2.VideoWriter_fourcc(*"mp4v")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            angles = reader.get()

            if recording:
                vw.write(frame)
                angle_log.append(angles)
                frame_count += 1

            # 미리보기 오버레이
            color  = (0, 0, 255) if recording else (0, 200, 0)
            status = f"REC  ep{ep_idx:03d}  {frame_count/FPS:.1f}s" if recording else f"STANDBY  ep{ep_idx:03d}"
            disp = frame.copy()
            cv2.putText(disp, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            angle_str = " ".join(f"s{i+1}:{v}" for i, v in enumerate(angles))
            cv2.putText(disp, angle_str, (10, H - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Wrist", disp)

            key = cv2.waitKey(FRAME_MS) & 0xFF

            if key == ord("q"):
                if recording:
                    finish_episode(ep_dir, vw, angle_log, frame_count)
                    ep_idx += 1
                break

            elif key == ord(" "):
                if not recording:
                    ep_dir      = DATASET_DIR / f"episode_{ep_idx:03d}"
                    ep_dir.mkdir(parents=True, exist_ok=True)
                    vw          = cv2.VideoWriter(str(ep_dir / "cam_wrist.mp4"), fourcc, FPS, (W, H))
                    angle_log   = []
                    frame_count = 0
                    recording   = True
                    print(f"[REC]  에피소드 {ep_idx:03d} 시작")
                else:
                    recording  = False
                    last_saved = ep_dir
                    finish_episode(ep_dir, vw, angle_log, frame_count)
                    ep_idx += 1

            elif key == ord("d") and not recording:
                ep_idx     = delete_last(ep_idx, last_saved)
                last_saved = None

    except KeyboardInterrupt:
        print("\n[INFO] 중단됨")
        if recording:
            finish_episode(ep_dir, vw, angle_log, frame_count)

    finally:
        cap.release()
        cv2.destroyAllWindows()
        reader.close()
        print(f"\n[DONE] 총 저장 에피소드: {len(sorted(DATASET_DIR.glob('episode_*')))}개")


if __name__ == "__main__":
    main()
