@echo off
echo [1/3] 가상환경 생성 중...
python -m venv venv

echo [2/3] 패키지 설치 중...
call venv\Scripts\activate.bat
pip install opencv-python pyserial numpy torch torchvision --index-url https://download.pytorch.org/whl/cu121

echo [3/3] 완료!
echo.
echo 이제 아래 파일들로 실행하세요:
echo   collect.bat   - 데이터 수집
echo   convert.bat   - LeRobot 변환
echo   run.bat       - 자율 동작
pause
