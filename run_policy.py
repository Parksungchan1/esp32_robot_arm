"""
학습된 ACT 모델로 자율 동작 실행

  q : 종료 (Policy 창에서)
"""

import sys
import time
import numpy as np
import cv2
import serial
import torch

sys.path.insert(0, "C:/Users/parks/lerobot/src")

from lerobot.policies.act.modeling_act import ACTPolicy

# ── 설정 ──────────────────────────────────────────────────────────────
MODEL_PATH = "C:/Users/parks/esp32_robot_arm/training_output/checkpoints/last/pretrained_model"
COM_PORT   = "COM6"
BAUD_RATE  = 115200
FPS        = 30
FRAME_MS   = int(1000 / FPS)
DEVICE     = "cuda" if torch.cuda.is_available() else "cpu"

# ── 정규화 통계 (학습 시 사용된 값) ───────────────────────────────────
STATE_MEAN = np.array([80.4144, 81.3236, 132.7538,  0.0,  96.4020, 70.3869], dtype=np.float32)
STATE_STD  = np.array([13.7177, 25.4828,  25.6123,  1.0,  20.1296, 27.9390], dtype=np.float32)  # s4 std=0 → 1로 대체
ACT_MEAN   = np.array([80.4144, 81.3236, 132.7538,  0.0,  96.4020, 70.3869], dtype=np.float32)
ACT_STD    = np.array([13.7177, 25.4828,  25.6123,  1.0,  20.1296, 27.9390], dtype=np.float32)

# ── 각도 제한 (ESP32 펌웨어와 동일) ──────────────────────────────────
ANGLE_MIN = np.array([  0,   0,   0,  0,   0,  40], dtype=np.float32)
ANGLE_MAX = np.array([180, 180, 180,  0, 180, 110], dtype=np.float32)

# ── 카메라 탐지 ───────────────────────────────────────────────────────
def find_webcam():
    for i in range(6):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None and frame.std() > 10:
                print(f"[INFO] 웹캠 발견: index {i}")
                return cap
            cap.release()
    return None


# ── 시리얼 각도 전송 ──────────────────────────────────────────────────
class AngleWriter:
    def __init__(self, port, baud):
        self._ser = serial.Serial(port, baud, timeout=1)
        time.sleep(0.5)

    def send(self, angles):
        cmd = "a:" + ",".join(str(int(round(float(a)))) for a in angles) + "\n"
        self._ser.write(cmd.encode())

    def close(self):
        self._ser.close()


# ── 메인 ─────────────────────────────────────────────────────────────
def main():
    print(f"[INFO] 모델 로드 중... ({DEVICE})")
    policy = ACTPolicy.from_pretrained(MODEL_PATH)
    policy.to(DEVICE)
    policy.eval()
    policy.reset()
    print("[INFO] 모델 로드 완료")

    print(f"[INFO] 시리얼 연결 중... {COM_PORT}")
    try:
        writer = AngleWriter(COM_PORT, BAUD_RATE)
    except Exception as e:
        print(f"[ERROR] 시리얼 연결 실패: {e}")
        sys.exit(1)
    print("[INFO] 시리얼 OK")

    cap = find_webcam()
    if cap is None:
        print("[ERROR] 웹캠을 찾을 수 없습니다.")
        writer.close()
        sys.exit(1)

    print()
    print("[INFO] 5초 후 자율 동작 시작... (Policy 창에서 q=종료)")
    print()

    running    = False
    start_time = time.time()
    angles     = np.array([77, 136, 180, 0, 90, 69], dtype=np.float32)  # ESP32 초기값

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            elapsed = time.time() - start_time

            # 5초 후 자동 시작
            if not running and elapsed >= 5.0:
                running = True
                policy.reset()
                print("[INFO] 자율 동작 시작")

            if running:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img_tensor = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0
                img_tensor = img_tensor.unsqueeze(0).to(DEVICE)

                # 상태 정규화
                state_norm = (angles - STATE_MEAN) / STATE_STD
                state_tensor = torch.from_numpy(state_norm).unsqueeze(0).to(DEVICE)

                with torch.no_grad():
                    batch = {
                        "observation.images.wrist": img_tensor,
                        "observation.state": state_tensor,
                    }
                    action_norm = policy.select_action(batch).squeeze(0).cpu().numpy()

                # 액션 역정규화 → 실제 각도
                angles = action_norm * ACT_STD + ACT_MEAN
                angles = np.clip(angles, ANGLE_MIN, ANGLE_MAX)
                writer.send(angles)

            # 미리보기
            if running:
                color  = (0, 0, 255)
                status = "RUNNING"
            else:
                color  = (0, 200, 0)
                remain = max(0, 5.0 - elapsed)
                status = f"START IN {remain:.1f}s"

            disp = frame.copy()
            cv2.putText(disp, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            angle_str = " ".join(f"s{i+1}:{int(a)}" for i, a in enumerate(angles))
            cv2.putText(disp, angle_str, (10, frame.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Policy", disp)

            key = cv2.waitKey(FRAME_MS) & 0xFF
            if key == ord("q"):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        writer.close()
        print("[DONE] 종료")


if __name__ == "__main__":
    main()
