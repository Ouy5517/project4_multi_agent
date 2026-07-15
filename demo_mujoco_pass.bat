@echo off
REM MuJoCo 3D 传球最小演示
cd /d "%~dp0"
call .venv\Scripts\activate.bat 2>nul
set PYTHONUTF8=1
python -m pip install mujoco -q 2>nul
python demo_mujoco_pass.py %*
if errorlevel 1 (
    echo.
    echo [错误] 演示未能正常启动，请查看上方报错信息。
    pause
)
