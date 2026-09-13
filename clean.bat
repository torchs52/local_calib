@echo off

:: __pycache__ を削除
for /d /r %%i in (__pycache__) do rd /s /q %%i

:: typings ディレクトリを削除
if exist typings rd /s /q typings
if exist octotree\build rd /s /q octotree\build
if exist octotree\octotree.egg-info rd /s /q octotree\octotree.egg-info

del /f /q log\*.txt
del /f /q log\*.dat
del /f /q log\*.mmap
del /f /q *.profile
del /f /q *.engine
del /f /q *.timing
del /f /q *.so

echo Clean completed.
