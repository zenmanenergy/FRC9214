@echo off
cd /d "%~dp0"
call "../../../../.venv/Scripts/activate.bat"
pip install flask-sock
python teleop_dashboard.py
pause

