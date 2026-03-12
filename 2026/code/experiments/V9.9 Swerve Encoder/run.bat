@echo off
echo Installing dependencies...
pip install flask-sock
echo.
echo Starting FRC 9214 Web Dashboard (WebSocket)...
echo Listening on http://localhost:5000
echo.
python web_dashboard_ws.py

