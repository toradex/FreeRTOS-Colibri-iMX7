cd debug
rd /s /Q CMakeFiles
del /s /Q /F Makefile cmake_install.cmake CMakeCache.txt *.elf *.bin *.map *.hex
cd ..
cd release
rd /s /Q CMakeFiles
del /s /Q /F Makefile cmake_install.cmake CMakeCache.txt *.elf *.bin *.map *.hex
cd ..
pause
