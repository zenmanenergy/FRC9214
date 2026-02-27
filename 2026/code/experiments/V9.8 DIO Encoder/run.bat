@echo off
echo Starting FRC 9214 Web Dashboard...
echo Listening on http://localhost:5000
echo.
python -m flask --app web_dashboard run --host=0.0.0.0 --port=5000 --reload

