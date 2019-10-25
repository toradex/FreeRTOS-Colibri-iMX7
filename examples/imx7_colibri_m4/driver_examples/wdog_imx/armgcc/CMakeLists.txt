INCLUDE(CMakeForceCompiler)

# === Project-specific settings ===
SET(ProjectName wdog_imx)
SET(LinkerFile MCIMX7D_M4_tcm.ld)
SET(BspRootDirPath ${CMAKE_CURRENT_SOURCE_DIR}/../../../../..)

PROJECT(${ProjectName})
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

# cross compiler settings
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

SET(CMAKE_SYSTEM_NAME Generic)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

# don't generate installation rules and don't generate cmake_install.cmake
SET(CMAKE_SKIP_INSTALL_RULES TRUE)


# === ASM FLAGS ===
# enable asm
ENABLE_LANGUAGE(ASM)
#default
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mapcs  -std=gnu99")
# debug
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -g  -Og  -D__DEBUG")
# release
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -Os  -D__NDEBUG")


# === C FLAGS ===
# default
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -MMD  -MP  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mapcs  -std=gnu99  -DCPU_MCIMX7D_M4")
# debug
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -Og -D__DEBUG")
# release
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Os -D__NDEBUG")


# === LINKER FLAGS ===
# default
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${LinkerFile} -static -L ${BspRootDirPath}/platform/devices/MCIMX7D/linker/gcc/")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mcpu=cortex-m4 -mfloat-abi=hard  -mfpu=fpv4-sp-d16  --specs=nano.specs  -Wall  -fno-common -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mthumb -mapcs  -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z -z max-page-size=32 -Xlinker muldefs")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Xlinker --defsym=__stack_size__=0x400  -Xlinker --defsym=__heap_size__=0x200")
# debug
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g")


# === Linker Output ===
# .map 
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}  -Xlinker -Map=${ProjectName}.map")


# === Add Include directories here ===
INCLUDE_DIRECTORIES(${ProjDirPath}/../../..)
INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/CMSIS/Include)
INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/devices)
INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/devices/MCIMX7D/include)
INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/devices/MCIMX7D/startup)
INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/drivers/inc)
INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/utilities/inc)


# === Add Source code references here ===
ADD_EXECUTABLE(${ProjectName}
    "${BspRootDirPath}/platform/devices/MCIMX7D/startup/gcc/startup_MCIMX7D_M4.S"
    "${BspRootDirPath}/platform/devices/MCIMX7D/startup/system_MCIMX7D_M4.c"
    "${BspRootDirPath}/platform/drivers/src/ccm_analog_imx7d.c"
    "${BspRootDirPath}/platform/drivers/src/ccm_imx7d.c"
    "${BspRootDirPath}/platform/drivers/src/lmem.c"
    "${BspRootDirPath}/platform/drivers/src/rdc.c"
    "${BspRootDirPath}/platform/drivers/src/uart_imx.c"
    "${BspRootDirPath}/platform/drivers/src/wdog_imx.c"
    "${BspRootDirPath}/platform/utilities/src/debug_console_imx.c"
    "${BspRootDirPath}/platform/utilities/src/print_scan.c"
    "${ProjDirPath}/../../../board.c"
    "${ProjDirPath}/../../../clock_freq.c"
    "${ProjDirPath}/../../../pin_mux.c"
    "${ProjDirPath}/../hardware_init.c"
    "${ProjDirPath}/../main.c"
)

# additional system libraries
TARGET_LINK_LIBRARIES(${ProjectName} -Wl,--start-group m c gcc nosys -Wl,--end-group)
# .elf
SET_TARGET_PROPERTIES(${ProjectName} PROPERTIES OUTPUT_NAME "${ProjectName}.elf")
