@echo off
cd release
call build_release.bat
cd ..
cd debug
call build_debug.bat
cd ..
