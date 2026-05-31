import sys
import time
import threading
import numpy as np
import cv2
import serial
import torch

sys.path.insert(0, "C:/Users/parks/lerobot/src")
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.processor.converters import PolicyAction

MODEL_PATH = "C:/Users/parks/esp32_robot_arm/training_output/checkpoints/last/pretrained_model"
COM_PORT   = "COM3"
BAUD_RATE  = 115200
FPS        = 30
FRAME_MS   = int(1000 / FPS)

ANGLE_MIN = np.array([0,   0,   0,   0,   0,  20], dtype=np.float32)
ANGLE_MAX = np.array([180, 180, 180, 180, 180, 110], dtype=np.float32)
MAX_DELTA = 0.5  # 프레임당 최대 이동 각도 (도). 줄이면 더 느려짐


def init_cameras():
    cam_wrist = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cam_full  = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    if not cam_wrist.isOpened() or not cam_full.isOpened():
        print("[ERROR] 카메라 2개 필요")
        return None, None

    print("[INFO] WristCam=0 / FullCam=1")
    return cam_wrist, cam_full


class SerialHandler:
    """시리얼 읽기(실제 각도) + 쓰기(명령) 통합 처리"""
    def __init__(self, port, baud):
        self._ser = serial.Serial(port, baud, timeout=1)
        self._angles = np.array([77, 136, 180, 0, 90, 69], dtype=np.float32)
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._read_loop, daemon=True).start()
        time.sleep(0.5)

    def _read_loop(self):
        while self._running:
            try:
                line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("s1:"):
                    vals = [float(p.split(":")[1]) for p in line.split(",")]
                    if len(vals) == 6:
                        with self._lock:
                            self._angles = np.array(vals, dtype=np.float32)
            except Exception:
                pass

    def get_angles(self):
        with self._lock:
            return self._angles.copy()

    def send(self, angles):
        cmd = "a:" + ",".join(str(int(round(float(a)))) for a in angles) + "\n"
        self._ser.write(cmd.encode())

    def close(self):
        self._running = False
        self._ser.close()


def main():
    print(f"[INFO] 모델 로드 중...")
    policy = ACTPolicy.from_pretrained(MODEL_PATH)
    policy.eval()
    policy.reset()

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy.config,
        pretrained_path=MODEL_PATH,
    )

    print("[INFO] 시리얼 연결")
    handler = SerialHandler(COM_PORT, BAUD_RATE)

    cap_wrist, cap_full = init_cameras()
    if cap_wrist is None:
        handler.close()
        sys.exit(1)

    # ESP32 리셋 후 서보가 이동하는 ANGLE_INIT 위치
    angles = np.array([72, 108, 180, 31, 158, 69], dtype=np.float32)

    running = False
    start_time = time.time()

    print("[INFO] 5초 후 시작 | q: 종료")

    while True:
        ret1, frame1 = cap_wrist.read()
        ret2, frame2 = cap_full.read()

        if not ret1 or not ret2:
            continue

        elapsed = time.time() - start_time

        if not running and elapsed >= 5:
            running = True
            policy.reset()
            print(f"[INFO] 시작")

        if running:
            # 실제 서보 각도를 state로 사용
            angles = handler.get_angles()

            rgb1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
            rgb2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2RGB)

            img1 = torch.from_numpy(rgb1).permute(2, 0, 1).float() / 255.0
            img2 = torch.from_numpy(rgb2).permute(2, 0, 1).float() / 255.0

            obs = {
                "observation.images.wrist": img1,
                "observation.images.full":  img2,
                "observation.state": torch.from_numpy(angles),
            }

            with torch.no_grad():
                obs_pre = preprocessor(obs)
                action_raw = policy.select_action(obs_pre)
                action_out = postprocessor(PolicyAction(action_raw))

            target = action_out.squeeze(0).cpu().numpy()
            target = np.clip(target, ANGLE_MIN, ANGLE_MAX)
            cmd = np.clip(target, angles - MAX_DELTA, angles + MAX_DELTA)
            handler.send(cmd)

        cv2.imshow("Wrist", frame1)
        cv2.imshow("Full",  frame2)

        if cv2.waitKey(FRAME_MS) & 0xFF == ord("q"):
            break

    cap_wrist.release()
    cap_full.release()
    cv2.destroyAllWindows()
    handler.close()


if __name__ == "__main__":
    main()
