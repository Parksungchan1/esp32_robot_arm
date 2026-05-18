# ESP32 Robot Arm — 프로젝트 컨텍스트

## 프로젝트 개요
ESP32 6축 로봇 팔에 ACT(Action Chunking with Transformers) 모방 학습을 적용하는 프로젝트.
리드스루 티칭 방식: 팔을 손으로 직접 움직여 시연 → 카메라+각도 기록 → ACT 학습 → 자율 동작.

## 하드웨어 구성
- **ESP32 Dev Module** (COM6, 115200 baud)
- **서보 6개** (MG996R): 핀 13, 12, 14, 27, 25, 26 → S1~S6
- **가변저항 5개**: 핀 34, 35, 36, 39, 32 → S1~S5 각도 읽기 (ADC)
- **조이스틱**: 핀 33 → S6 그리퍼 제어 (JOY_HIGH=2700, JOY_LOW=1400)
- **카메라 2대**: cam_wrist (index 0), cam_full (index 1)

## 서보 설정
```
ANGLE_INIT: S1=77,  S2=136, S3=180, S4=0,  S5=90,  S6=69
ANGLE_MIN:  S1=0,   S2=0,   S3=0,   S4=0,  S5=0,   S6=20
ANGLE_MAX:  S1=180, S2=180, S3=180, S4=180,S5=180,  S6=110
ADC_LOW=200, ADC_HIGH=4075
```

## 동작 모드
1. **수집 모드**: 가변저항으로 S1~S5 읽기 + 조이스틱으로 S6 제어 → 시리얼 출력 `s1:xx,...`
2. **자율동작 모드**: PC에서 `a:xx,xx,xx,xx,xx,xx\n` 수신 → 모든 서보 제어
3. **초기화**: ESP32 리셋 버튼 → ANGLE_INIT 위치로 이동 → S1~S5 detach (손으로 이동 가능), S6만 attach 유지

## 파일 구조
```
esp32_robot_arm/
├── esp32_firmware/esp32_firmware.ino  ← ESP32 펌웨어 (메인)
├── collect_data.py    ← 데이터 수집 (카메라 2대 + 시리얼)
├── convert_to_lerobot.py  ← LeRobot 포맷 변환
├── run_policy.py      ← 학습된 모델로 자율 동작
├── collect.bat / convert.bat / run.bat  ← venv 활성화 후 실행
└── datasets/          ← 수집 데이터 (gitignore)
```

## 실행 순서
1. **데이터 수집**: `collect.bat` → Space: 녹화 시작/종료, q: 종료
2. **변환**: `convert.bat` → datasets/ → lerobot_dataset/
3. **학습**: LeRobot ACT 학습 명령 (lerobot/ 폴더에서)
4. **자율동작**: `run.bat`

## 알려진 미해결 이슈
- **ADC 캘리브레이션 불일치**: S1 초기화 77°지만 가변저항은 50°로 읽힘, S2 136°→91°. 서보 PWM 각도 ≠ 실제 기구학적 각도
- **S4 서보 하드웨어 소손**: 과열로 VCC 선 탔음 → 서보 교체 + 외부 5V 배선 재확인 필요
- **cam_full 미통합**: convert_to_lerobot.py와 run_policy.py에 cam_full 채널 아직 미추가

## 주요 설정값
- `COM_PORT = "COM6"` (collect_data.py, run_policy.py)
- `DATASET_DIR = Path("C:/Users/parks/esp32_robot_arm/datasets")`
- `MODEL_PATH = training_output/checkpoints/last/pretrained_model`
- venv 경로: `esp32_robot_arm/venv/`
