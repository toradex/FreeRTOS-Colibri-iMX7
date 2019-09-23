@echo [7mBuilding %~dp0[0m
@SET BSPROOT=%~dp0../../../../../../..
@SET ARMGCC_CMAKE=%BSPROOT%/tools/cmake_toolchain_files/armgcc.cmake
cmake -DCMAKE_TOOLCHAIN_FILE="%ARMGCC_CMAKE%" -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release ..
mingw32-make -j4
pause
