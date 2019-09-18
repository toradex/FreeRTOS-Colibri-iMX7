@echo [7mCleaning build files[0m
@REM Folders
RD /s /Q CMakeFiles Debug Release 
@REM Files
DEL /s /Q /F CMakeCache.txt Makefile cmake_install.cmake
pause
