@echo off
rem MID360-agras 点云查看器：双击运行；也可以把 .pcapng 拖到本文件上回放
cd /d "%~dp0"
where pythonw >nul 2>nul
if errorlevel 1 (
    echo 没找到 Python。请安装 Python 3.10+ 后再运行。
    pause
    exit /b 1
)
python -c "import numpy, PyQt5, pyqtgraph, OpenGL" 2>nul
if errorlevel 1 (
    echo 首次运行，正在安装依赖 numpy PyQt5 pyqtgraph PyOpenGL ...
    python -m pip install numpy PyQt5 pyqtgraph PyOpenGL
    if errorlevel 1 (
        pause
        exit /b 1
    )
)
start "" pythonw "%~dp0mid360_viewer.py" %*
