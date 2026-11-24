@echo off
rem FAST-LIO map viewer: double-click to pick a .pcd, or drag a .pcd onto this file
cd /d "%~dp0"
start "" pythonw "%~dp0pcd_viewer.py" %*
