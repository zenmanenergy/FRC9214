@echo off
echo Starting YDLidar GS2 WebSocket Server...
echo.

REM Get the directory where this batch file is located
cd /d "%~dp0"

REM Configure COM6 to 921600 baud (as specified in LIDAR docs)
echo Configuring COM6 port...
mode COM6: baud=921600 parity=n data=8 stop=1

REM Run the Python script using the virtual environment
"C:\Apache24\htdocs\personalProjects\FRC9214\.venv\Scripts\python.exe" lidar.py

REM Keep window open if there's an error
if %errorlevel% neq 0 (
    echo.
    echo Error occurred. Press any key to exit...
    pause
)
