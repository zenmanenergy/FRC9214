@echo off
cd /d "%~dp0"
call ".\.venv\Scripts\activate.bat"
pip install flask-sock
python dashboard_server.py
if errorlevel 1 (
    echo.
    echo ERROR: dashboard_server.py failed with error code %errorlevel%
    pause
)

