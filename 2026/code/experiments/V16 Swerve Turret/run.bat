@echo off
cd /d "%~dp0"
call "../../../../.venv/Scripts/activate.bat"
pip install flask-sock
python teleop_dashboard.py
if errorlevel 1 (
    echo.
    echo ERROR: teleop_dashboard.py failed with error code %errorlevel%
    pause
)

