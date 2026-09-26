@echo off
rem Полный прогон на Windows: окружение Python + tools\run_all.py.
rem   run_all.bat                     всё (метрики, results\metrics.html, results\demo.html)
rem   run_all.bat --quick             проверка окружения за несколько минут
rem   run_all.bat --data D:\lidar     данные не в .\data
rem Нужен Python 3.10+ (python.org, галочка "Add python.exe to PATH"). ROS 2 и Docker не нужны.
chcp 65001 >nul
setlocal
cd /d "%~dp0"
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
set VENV=.venv-win

if exist "%VENV%\Scripts\python.exe" goto :install
set PY=
where py >nul 2>nul && set PY=py -3
if not defined PY (where python >nul 2>nul && set PY=python)
if not defined PY (
  echo Не найден Python. Установите Python 3.10+ с python.org и отметьте "Add python.exe to PATH".
  goto :fail
)
%PY% -c "import sys; sys.exit(sys.version_info < (3, 10))" || (
  echo Нужен Python 3.10 или новее.
  goto :fail
)
echo Создаю окружение %VENV% ...
%PY% -m venv %VENV% || goto :fail

:install
"%VENV%\Scripts\python.exe" -c "import tunnel_od, rosbags, matplotlib, pytest" >nul 2>nul && goto :run
echo Ставлю зависимости (нужна сеть, один раз) ...
"%VENV%\Scripts\python.exe" -m pip install --upgrade pip || goto :fail
"%VENV%\Scripts\python.exe" -m pip install -e "core[tools]" pytest || goto :fail

:run
"%VENV%\Scripts\python.exe" tools\run_all.py %*
set RC=%ERRORLEVEL%
echo.
pause
exit /b %RC%

:fail
echo.
pause
exit /b 1
