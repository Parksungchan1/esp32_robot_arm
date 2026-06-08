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
MAX_DELTA = 5.0  # 프레임당 최대 이동 각도 (도). 줄이면 더 느려짐

# 가변저항(POT) 읽기값 → 실제 서보 PWM 보정 오프셋
# 자율동작 테스트 후 빗나가는 방향/크기 측정하여 채울 것
# 예: S1이 항상 +15° 더 가면 → 0: -15
POT_TO_PWM = np.array([1,0, 0, 0, 0, 3], dtype=np.float32)  # S1~S6


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
    angles = np.array([104, 106, 128, 24, 90, 72], dtype=np.float32)

    running = False
    start_time = time.time()
    phase = 1  # 1:잡기/들기  2:입쪽이동  3:복귀  4:내려놓기
    phase_labels = {1:"잡기/들기", 2:"입쪽이동", 3:"복귀", 4:"내려놓기"}

    print("[INFO] 5초 후 시작 | 1/2/3/4: phase 전환 | q: 종료")

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
            angles = handler.get_angles()
            state = np.array(list(angles) + [float(phase)], dtype=np.float32)

            rgb1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
            rgb2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2RGB)

            img1 = torch.from_numpy(rgb1).permute(2, 0, 1).float() / 255.0
            img2 = torch.from_numpy(rgb2).permute(2, 0, 1).float() / 255.0

            obs = {
                "observation.images.wrist": img1,
                "observation.images.full":  img2,
                "observation.state": torch.from_numpy(state),
            }

            with torch.no_grad():
                obs_pre = preprocessor(obs)
                action_raw = policy.select_action(obs_pre)
                action_out = postprocessor(PolicyAction(action_raw))

            target = action_out.squeeze(0).cpu().numpy()
            target = np.clip(target, ANGLE_MIN, ANGLE_MAX)
            cmd = np.clip(target, angles - MAX_DELTA, angles + MAX_DELTA)
            print(f"[P{phase}:{phase_labels[phase]}] S6 target:{target[5]:.1f} cmd:{cmd[5]:.1f}", end="\r")
            handler.send(cmd + POT_TO_PWM)

        cv2.imshow("Wrist", frame1)
        cv2.imshow("Full",  frame2)

        key = cv2.waitKey(FRAME_MS) & 0xFF
        if key == ord("q"):
            break
        elif key in (ord("1"), ord("2"), ord("3"), ord("4")):
            phase = int(chr(key))
            print(f"\n[PHASE] {phase} — {phase_labels[phase]}")

    cap_wrist.release()
    cap_full.release()
    cv2.destroyAllWindows()
    handler.close()


if __name__ == "__main__":
    main()
