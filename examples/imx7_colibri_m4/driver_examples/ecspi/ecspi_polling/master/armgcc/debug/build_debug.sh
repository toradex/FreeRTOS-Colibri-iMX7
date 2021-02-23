#!/bin/sh
# Absolute path to this script
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

BSPROOT=$SCRIPTPATH/../../../../../../../..
ARMGCC_CMAKE=$BSPROOT/tools/cmake_toolchain_files/armgcc.cmake

echo "\e[7mBuilding $SCRIPTPATH\e[0m"

cmake -DCMAKE_TOOLCHAIN_FILE="$ARMGCC_CMAKE" -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug  ..
make -j4
