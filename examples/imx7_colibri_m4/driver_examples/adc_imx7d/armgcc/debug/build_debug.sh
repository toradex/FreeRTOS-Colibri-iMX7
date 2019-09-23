#!/bin/sh
cmake -DCMAKE_TOOLCHAIN_FILE="../../../../../tools/cmake_toolchain_files/armgcc.cmake" -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug  ..
make -j4
