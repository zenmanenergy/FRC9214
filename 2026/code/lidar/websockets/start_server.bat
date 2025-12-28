@echo off
echo Starting YDLidar GS2 WebSocket Server...
echo.

REM Get the directory where this batch file is located
cd /d "%~dp0"

REM Run the Python script using the virtual environment
"C:\Apache24\htdocs\personalProjects\FRC9214\.venv\Scripts\python.exe" lidar.py

REM Keep window open if there's an error
if %errorlevel% neq 0 (
    echo.
    echo Error occurred. Press any key to exit...
    pause
)
