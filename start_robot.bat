@echo off
REM ====================================================================
REM Disaster Robot - Launch Script
REM
REM Usage:
REM   start_robot.bat                  -- Local Wi-Fi + laptop (default)
REM   start_robot.bat dev-wired        -- Dev (wired ICS + desktop)
REM   start_robot.bat dev-wireless     -- Local Wi-Fi + laptop
REM
REM Setup (once per mode):
REM   ssh-keygen -t ed25519
<<<<<<< HEAD
REM   ssh-copy-id pi@0.0.0.0        (Tailscale)
=======
REM   ssh-copy-id pi@10.92.223.146        (local Wi-Fi)
>>>>>>> temp
REM   ssh-copy-id pi@192.168.137.126      (wired ICS)
REM ====================================================================

setlocal

REM ---- Parse mode argument ----
set MODE=%1
if "%MODE%"=="" (
<<<<<<< HEAD
    set RPI_HOST=0.0.0.0
=======
    set RPI_HOST=10.92.223.146
>>>>>>> temp
    set MODE_ARG=
    set MODE_LABEL=demo
) else if "%MODE%"=="dev-wired" (
    set RPI_HOST=192.168.137.126
    set MODE_ARG=--mode dev-wired
    set MODE_LABEL=dev-wired
) else if "%MODE%"=="dev-wireless" (
<<<<<<< HEAD
    set RPI_HOST=0.0.0.0
=======
    set RPI_HOST=10.92.223.146
>>>>>>> temp
    set MODE_ARG=--mode dev-wireless
    set MODE_LABEL=dev-wireless
) else (
    echo Unknown mode: %MODE%
    echo Usage: start_robot.bat [dev-wired^|dev-wireless]
    exit /b 1
)

REM ---- Settings ----
set RPI_USER=pi
set RPI_PROJECT_DIR=~/disaster-robot
set RPI_VENV=~/dr-env/bin/activate
set CONFIG=config/real.yaml

REM ---- Paths (auto) ----
set SCRIPT_DIR=%~dp0
cd /d %SCRIPT_DIR%

echo ====================================================================
echo  Disaster Robot Launch
echo  Mode: %MODE_LABEL%
echo  RPi: %RPI_USER%@%RPI_HOST%
echo  Config: %CONFIG%
echo ====================================================================
echo.

REM ---- Launch RPi main.py (separate window) ----
echo [1/2] Starting RPi main...
set REMOTE_CMD=cd %RPI_PROJECT_DIR% ^&^& source %RPI_VENV% ^&^& python raspberry-pi/main.py --config %CONFIG% %MODE_ARG%
start "RPi Main" cmd /k ssh -t %RPI_USER%@%RPI_HOST% "%REMOTE_CMD%"

REM ---- Wait for RPi to be ready ----
echo Waiting 3 seconds for RPi...
timeout /t 3 /nobreak >nul

REM ---- Launch laptop GUI ----
echo [2/2] Starting laptop GUI...
echo.

REM Try to activate virtualenv (conda or venv)
if exist "%SCRIPT_DIR%venv\Scripts\activate.bat" (
    call "%SCRIPT_DIR%venv\Scripts\activate.bat"
) else if defined CONDA_DEFAULT_ENV (
    echo (using conda env: %CONDA_DEFAULT_ENV%)
) else (
    echo (no virtualenv detected - using system Python)
)

python laptop/main.py --config %CONFIG%

REM ---- Shutdown handling ----
echo.
echo ====================================================================
echo Laptop GUI closed. Close the RPi window manually.
echo ====================================================================
pause
