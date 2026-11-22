@echo off
rem Run a Talos sim script with Python 3.12. Double-click it, or from anywhere:
rem
rem   sim\run.bat                      drive the arm, with the block and the box
rem   sim\run.bat drive.py             just the arm
rem   sim\run.bat check_model.py --fast
rem   sim\run.bat build_model.py
rem
rem The first run creates the Python environment, which needs uv
rem (https://docs.astral.sh/uv/). It goes in %USERPROFILE%\.venvs\talos-sim,
rem outside OneDrive, unless TALOS_SIM_VENV names another folder.
setlocal

set "REPO=%~dp0.."
if "%TALOS_SIM_VENV%"=="" set "TALOS_SIM_VENV=%USERPROFILE%\.venvs\talos-sim"
set "VENV=%TALOS_SIM_VENV%"

if not exist "%VENV%\pyvenv.cfg" (
    where uv >nul 2>nul || (
        echo uv isn't installed. Install it with:  winget install astral-sh.uv
        echo then open a new window and run this again.
        goto fail
    )
    echo First run: setting up Python 3.12 in %VENV% ...
    uv venv --python 3.12 "%VENV%" || goto fail
    uv pip install --python "%VENV%" -r "%REPO%\sim\requirements.txt" || goto fail
)

rem Run the environment's base Python with its packages on PYTHONPATH, rather
rem than the environment's own python.exe. That launcher is unsigned, and
rem Windows Smart App Control can block it while allowing the base Python.
for /f "tokens=1,* delims==" %%A in ('findstr /b /c:"home" "%VENV%\pyvenv.cfg"') do set "BASE=%%B"
for /f "tokens=*" %%A in ("%BASE%") do set "BASE=%%A"
if not exist "%BASE%\python.exe" (
    echo Can't find the Python that %VENV% was made from:
    echo   %BASE%
    echo Delete %VENV% and run this again to recreate it.
    goto fail
)
set "PYTHONPATH=%VENV%\Lib\site-packages"

cd /d "%REPO%"
if "%~1"=="" (
    "%BASE%\python.exe" sim\drive.py --pick
) else (
    "%BASE%\python.exe" sim\%*
)
if errorlevel 1 goto fail
exit /b 0

:fail
echo.
pause
exit /b 1
