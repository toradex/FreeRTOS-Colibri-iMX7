INCLUDE(CMakeForceCompiler)

SET(ProjectName gpio_imx)
PROJECT(${ProjectName})

# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

SET(CMAKE_SKIP_INSTALL_RULES TRUE)

# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})
SET(BspRootDirPath ${CMAKE_CURRENT_SOURCE_DIR}/../../../../..)

# RELEASE/DEBUG LINK FILE
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T MCIMX7D_M4_ocram.ld -static -L ${BspRootDirPath}/platform/devices/MCIMX7D/linker/gcc/")

# DEFAULT ASM FLAGS
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mapcs  -std=gnu99")

# DEFAULT C FLAGS
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -MMD  -MP  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mapcs  -std=gnu99")

# DEFAULT LD FLAGS
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mcpu=cortex-m4 -mfloat-abi=hard  -mfpu=fpv4-sp-d16  --specs=nano.specs  -Wall  -fno-common -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mthumb -mapcs  -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z -z max-page-size=4096 -Xlinker muldefs")

# DEBUG ASM FLAGS
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -g")

# DEBUG C FLAGS
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0")

# DEBUG LD FLAGS
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g")

# RELEASE C FLAGS
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Os")

# ASM MACRO
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG}  -D__DEBUG")

# C MACRO
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DCPU_MCIMX7D_M4")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -D__DEBUG")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -D__NDEBUG")

# INCLUDE_DIRECTORIES
INCLUDE_DIRECTORIES(${ProjDirPath}/..)
INCLUDE_DIRECTORIES(${ProjDirPath}/../../..)
INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/CMSIS/Include)
INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/devices)
INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/devices/MCIMX7D/include)
INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/devices/MCIMX7D/startup)
INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/drivers/inc)
INCLUDE_DIRECTORIES(${BspRootDirPath}/platform/utilities/inc)


# ADD_EXECUTABLE
ADD_EXECUTABLE(${ProjectName}
    "${BspRootDirPath}/platform/devices/MCIMX7D/startup/gcc/startup_MCIMX7D_M4.S"
    "${BspRootDirPath}/platform/utilities/src/debug_console_imx.c"
    "${BspRootDirPath}/platform/utilities/inc/debug_console_imx.h"
    "${BspRootDirPath}/platform/utilities/src/print_scan.c"
    "${BspRootDirPath}/platform/utilities/src/print_scan.h"
    "${BspRootDirPath}/platform/drivers/inc/ccm_analog_imx7d.h"
    "${BspRootDirPath}/platform/drivers/inc/ccm_imx7d.h"
    "${BspRootDirPath}/platform/drivers/inc/lmem.h"
    "${BspRootDirPath}/platform/drivers/src/rdc_semaphore.c"
    "${BspRootDirPath}/platform/drivers/inc/rdc_semaphore.h"
    "${BspRootDirPath}/platform/drivers/inc/rdc.h"
    "${BspRootDirPath}/platform/drivers/inc/rdc_defs_imx7d.h"
    "${BspRootDirPath}/platform/drivers/inc/wdog_imx.h"
    "${BspRootDirPath}/platform/drivers/src/ccm_analog_imx7d.c"
    "${BspRootDirPath}/platform/drivers/src/ccm_imx7d.c"
    "${BspRootDirPath}/platform/drivers/src/lmem.c"
    "${BspRootDirPath}/platform/drivers/src/rdc.c"
    "${BspRootDirPath}/platform/drivers/src/wdog_imx.c"
    "${BspRootDirPath}/platform/devices/MCIMX7D/startup/system_MCIMX7D_M4.c"
    "${BspRootDirPath}/platform/devices/MCIMX7D/startup/system_MCIMX7D_M4.h"
    "${BspRootDirPath}/platform/drivers/src/gpio_imx.c"
    "${BspRootDirPath}/platform/drivers/inc/gpio_imx.h"
    "${ProjDirPath}/../../../pin_mux.c"
    "${ProjDirPath}/../../../pin_mux.h"
    "${ProjDirPath}/../../../board.c"
    "${ProjDirPath}/../../../board.h"
    "${ProjDirPath}/../../../clock_freq.c"
    "${ProjDirPath}/../../../clock_freq.h"
    "${ProjDirPath}/../hardware_init.c"
    "${ProjDirPath}/../../../gpio_pins.c"
    "${ProjDirPath}/../../../gpio_pins.h"
    "${ProjDirPath}/../gpio_ctrl.c"
    "${ProjDirPath}/../gpio_ctrl.h"
    "${ProjDirPath}/../main.c"
    "${BspRootDirPath}/platform/drivers/src/uart_imx.c"
    "${BspRootDirPath}/platform/drivers/inc/uart_imx.h"
)
SET_TARGET_PROPERTIES(${ProjectName} PROPERTIES OUTPUT_NAME "${ProjectName}.elf")

TARGET_LINK_LIBRARIES(${ProjectName} -Wl,--start-group)
# LIBRARIES
IF(CMAKE_BUILD_TYPE MATCHES Debug)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
ENDIF()

# SYSTEM LIBRARIES
TARGET_LINK_LIBRARIES(${ProjectName} m)
TARGET_LINK_LIBRARIES(${ProjectName} c)
TARGET_LINK_LIBRARIES(${ProjectName} gcc)
TARGET_LINK_LIBRARIES(${ProjectName} nosys)
TARGET_LINK_LIBRARIES(${ProjectName} -Wl,--end-group)

# MAP FILE
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}  -Xlinker -Map=${ProjectName}.map")

# BIN AND HEX
ADD_CUSTOM_COMMAND(TARGET ${ProjectName} POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Oihex ${EXECUTABLE_OUTPUT_PATH}/${ProjectName}.elf ${EXECUTABLE_OUTPUT_PATH}/${ProjectName}.hex)
# bin does not support sparse memory area, which our default linker file uses...
#ADD_CUSTOM_COMMAND(TARGET ${ProjectName} POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Obinary ${EXECUTABLE_OUTPUT_PATH}/${ProjectName}.elf ${EXECUTABLE_OUTPUT_PATH}/${ProjectName}.bin)
