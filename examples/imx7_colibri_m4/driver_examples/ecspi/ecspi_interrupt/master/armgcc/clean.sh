#!/bin/sh
echo "\e[7mCleaning build files\e[0m"
# folders
rm -rf CMakeFiles debug release 
# files
rm -rf CMakeCache.txt Makefile cmake_install.cmake 
