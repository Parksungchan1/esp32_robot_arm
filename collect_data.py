import cv2
import serial
import threading
import time
import json
import sys
import shutil
from pathlib import Path

COM_PORT     = "COM3"
BAUD_RATE    = 115200
FPS          = 30
FRAME_MS     = int(1000 / FPS)
DATASET_DIR  = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("C:/Users/parks/esp32_robot_arm/datasets")


def init_cameras():
    cam_wrist = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cam_full  = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    if not cam_wrist.isOpened() or not cam_full.isOpened():
        print("[ERROR] 카메라 2개 모두 연결되어야 함")
        return None, None

    print("[INFO] WristCam = 0 / FullCam = 1")
    return cam_wrist, cam_full


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


def finish_episode(ep_dir, vw1, vw2, angle_log, frame_count):
    vw1.release()
    vw2.release()

    with open(ep_dir / "angles.json", "w") as f:
        json.dump({"fps": FPS, "frames": angle_log}, f)

    print(f"[SAVE] {ep_dir.name}  —  {frame_count}프레임")


def main():
    DATASET_DIR.mkdir(parents=True, exist_ok=True)

    ep_idx = len(sorted(DATASET_DIR.glob("episode_*")))

    print(f"[INFO] 시리얼 연결 중... {COM_PORT}")
    reader = AngleReader(COM_PORT, BAUD_RATE)
    time.sleep(0.5)


    cap_wrist, cap_full = init_cameras()
    if cap_wrist is None:
        reader.close()
        sys.exit(1)

    cap_wrist.set(cv2.CAP_PROP_FPS, FPS)
    cap_full.set(cv2.CAP_PROP_FPS, FPS)

    W = int(cap_wrist.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap_wrist.get(cv2.CAP_PROP_FRAME_HEIGHT))

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")

    recording = False
    vw1 = None
    vw2 = None
    angle_log = []
    frame_count = 0
    phase = 1  # 1:잡기/들기  2:입쪽이동  3:복귀  4:내려놓기

    print("[INFO] Space: 시작/종료 | 1/2/3/4: phase 전환 | q: 종료")

    while True:
        ret1, frame1 = cap_wrist.read()
        ret2, frame2 = cap_full.read()

        if not ret1 or not ret2:
            print("[ERROR] 카메라 프레임 실패")
            break

        angles = reader.get()

        if recording:
            vw1.write(frame1)
            vw2.write(frame2)
            angle_log.append(angles + [phase])
            frame_count += 1

        # 미리보기 오버레이
        color  = (0, 0, 255) if recording else (0, 200, 0)
        phase_labels = {1:"잡기/들기", 2:"입쪽이동", 3:"복귀", 4:"내려놓기"}
        status = f"REC  ep{ep_idx:03d}  {frame_count/FPS:.1f}s  [P{phase}:{phase_labels[phase]}]" if recording else f"STANDBY  ep{ep_idx:03d}"
        angle_str = " ".join(f"s{i+1}:{v}" for i, v in enumerate(angles))

        disp1 = frame1.copy()
        cv2.putText(disp1, status,    (10, 30),   cv2.FONT_HERSHEY_SIMPLEX, 0.65, color,           2)
        cv2.putText(disp1, angle_str, (10, H-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,  (255, 255, 255), 1)
        cv2.putText(disp1, "Space:시작/종료  1/2/3/4:phase  q:종료", (10, H-30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

        disp2 = frame2.copy()
        cv2.putText(disp2, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

        cv2.imshow("WristCam", disp1)
        cv2.imshow("FullCam",  disp2)

        key = cv2.waitKey(FRAME_MS) & 0xFF

        if key == ord("q"):
            break

        elif key in (ord("1"), ord("2"), ord("3"), ord("4")):
            phase = int(chr(key))
            print(f"[PHASE] {phase} — {phase_labels[phase]}")

        elif key == ord(" "):
            if not recording:
                ep_dir = DATASET_DIR / f"episode_{ep_idx:03d}"
                ep_dir.mkdir(parents=True, exist_ok=True)

                vw1 = cv2.VideoWriter(str(ep_dir / "cam_wrist.mp4"), fourcc, FPS, (W, H))
                vw2 = cv2.VideoWriter(str(ep_dir / "cam_full.mp4"),  fourcc, FPS, (W, H))

                angle_log = []
                frame_count = 0
                phase = 1
                recording = True

                print(f"[REC] episode {ep_idx:03d} 시작 (Phase 1부터)")

            else:
                recording = False
                finish_episode(ep_dir, vw1, vw2, angle_log, frame_count)
                ep_idx += 1

    cap_wrist.release()
    cap_full.release()
    cv2.destroyAllWindows()
    reader.close()


if __name__ == "__main__":
    main()
