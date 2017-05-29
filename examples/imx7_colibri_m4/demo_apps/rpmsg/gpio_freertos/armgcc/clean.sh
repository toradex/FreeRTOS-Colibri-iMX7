#!/bin/sh
cd debug
rm -rf Makefile cmake_install.cmake CMakeCache.txt CMakeFiles *.elf *.bin *.map *.hex .cproject .project
cd ..
cd release
rm -rf Makefile cmake_install.cmake CMakeCache.txt CMakeFiles *.elf *.bin *.map *.hex
cd ..
