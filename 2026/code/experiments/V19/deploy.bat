@echo off
cd /d "%~dp0"
call "../../../../.venv/Scripts/activate.bat"
python -m robotpy deploy
pause
