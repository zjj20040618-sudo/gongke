@echo off
setlocal
chcp 65001 >nul
set "PYTHONUTF8=1"
set "LOG_PYTHON=E:\setup\anaconda\envs\yolo_train\python.exe"
if not exist "%LOG_PYTHON%" (
    echo Cannot find Python: %LOG_PYTHON%
    echo Please edit LOG_PYTHON in this launcher to an existing Python with Tkinter.
    pause
    exit /b 1
)
"%LOG_PYTHON%" -B "%~dp0uart_log_collector.py"
if errorlevel 1 pause
