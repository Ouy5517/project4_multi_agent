@echo off
REM 传球场景图形可视化演示
cd /d "%~dp0"
call .venv\Scripts\activate.bat 2>nul
set PYTHONUTF8=1
echo.
echo  [1] 打开实时 2D 图形窗口 (传球连线可视化)
echo  [2] 同时导出 GIF 到 outputs/videos/pass_demo.gif
echo.
python main.py --scenario pass --duration 30 --viz matplotlib --export-gif outputs/videos/pass_demo.gif --export-csv
pause
