import cv2

def check_camera(index, name):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"[FAIL] {name} (index {index}) — 연결 안 됨")
        return False
    ret, frame = cap.read()
    cap.release()
    if not ret or frame is None:
        print(f"[FAIL] {name} (index {index}) — 프레임 읽기 실패")
        return False
    h, w = frame.shape[:2]
    print(f"[ OK ] {name} (index {index}) — {w}x{h}")
    return True

def preview_cameras():
    cap0 = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap1 = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    print("카메라 미리보기 중... 'q' 누르면 종료")
    while True:
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()

        if ret0:
            cv2.imshow("cam_wrist (index 0)", frame0)
        if ret1:
            cv2.imshow("cam_full  (index 1)", frame1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap0.release()
    cap1.release()
    cv2.destroyAllWindows()


wrist_ok = check_camera(0, "cam_wrist")
full_ok  = check_camera(1, "cam_full")

if wrist_ok and full_ok:
    print("\n카메라 2개 모두 정상 → run.bat 실행 가능")
    preview_cameras()
else:
    print("\n카메라 문제 있음 → 연결 확인 후 재시도")
