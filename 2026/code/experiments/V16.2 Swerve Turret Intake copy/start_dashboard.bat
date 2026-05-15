@echo off
REM Start both robot code (via FRC deploy) and dashboard server
REM This is for testing on your laptop - robot will run via normal FRC deployment

cd /d "%~dp0"

echo.
echo ========================================
echo Team 9214 Swerve Robot Dashboard
echo ========================================
echo.
echo Starting dashboard server...
echo Open browser at: http://localhost:5000
echo.
echo Note: Robot must be deployed separately via FRC driver station
echo.

python robot_dashboard_server.py 9214

pause
