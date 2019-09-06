"""Convert Cmake project files to VSCode workspace for C_CPP extension

This script walks throug all subfolders and searches for Cmake project files.

Each project file gets converted into a VSCode settings file.


This file can also be imported as a module and contains the following
functions:

    * cmake_get_includes - Read CMakeLists.txt and extract a list
                           of all #include folders
    * cmake_get_configurations - Read CMakeList.txt and extract a list
                                 of all configurations
    * cmake_get_defines - Read CMakeLists.txt and extract a list of 
                          all preprocessor #defines which start with
                          a given keyword.

__author__     = "Andy Kiser"
__copyright__  = "Copyright 2019, Toradex AG"
__license__    = "Exclusively for use in projects based on Toradex products"
__version__    = "1.1.0"
__maintainer__ = "Andy Kiser"
__email__      = "andy.kiser@toradex.com"
__status__     = "dev"
"""

import json
import re
from pathlib import Path, PosixPath


def cmake_get_includes(cmake_content: str) -> list:
    """ extract a list of include directories from a CMakeLists.txt file.
    The function assumes that the text file has been read entirely into a variable
    so the file contents are passed into this function as a string.

    Parameters
    ----------
    cmake_content: str
                   Full contents of the CMakeLists.txt file

    Returns
    -------
    A list of all include directories 
    """
    # Find definition of ${BspRootDirPath} (if any)
    regex = "SET\\(BspRootDirPath \\${CMAKE_CURRENT_SOURCE_DIR}([^\\)]*)\\)"
    bsp_root_dir = re.findall(regex, cmake_content)

    # list all lines which start with "include_directories(${ProjDirPath}/"
    regex = "include_directories|INCLUDE_DIRECTORIES\\(([^\\)]*)\\)"
    inc_dirs = re.findall(regex, cmake_content)

    # replace ${BspRootDirPath} with a description based on ${ProjDirPath}
    if (len(bsp_root_dir) == 1):
        for i, inc_dir in enumerate(inc_dirs):
            inc_dirs[i] = inc_dir.replace("${BspRootDirPath}",
                                          "${ProjDirPath}" + bsp_root_dir[0])

    # remove duplicated include directories
    inc_dirs = list(dict.fromkeys(inc_dirs))
    inc_dirs.sort()

    return inc_dirs


def cmake_get_configurations(cmake_content: str, flag_prefix: str) -> list:
    """ extract a list of configurations from a CMakeLists.txt file.  
    This is done by parsing statements where flags are set, such as 
      `SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -D__DEBUG")`
    The function assumes that the text file has been read entirely into a variable
    so the file contents are passed into this function as a string.

    Parameters
    ----------
    cmake_content: str
            Full contents of the CMakeLists.txt file

    flag_prefix: str
            Filter, which flags should be looked at  
            (`CMAKE_C_FLAGS_` in the example above).

    Returns
    -------
    A list of all configurations matching the given flag_prefix, 
    converted to Cmake standard camel case.
	Duplicates are removed.
    """
    # find all occurence of SET(...FLAGS...) statements
    regex = "SET\\(" + flag_prefix + "(\\w*)[^\\)]*\\)"
    flag_keywords = re.findall(regex, cmake_content)

    # remove duplicates
    flag_keywords = list(dict.fromkeys(flag_keywords))

    for i, f in enumerate(flag_keywords):
        if f.upper() == "DEBUG":
            flag_keywords[i] = "Debug"
        elif f.upper() == "RELEASE":
            flag_keywords[i] = "Release"
        elif f.upper() == "RELWITHDEBINFO":
            flag_keywords[i] = "RelWithDebInfo"
        elif f.upper() == "MINSIZEREL":
            flag_keywords[i] = "MinSizeRel"
        else:
            assert False

    return flag_keywords


def cmake_get_defines(cmake_content: str, flags_keyword: str) -> list:
    """ extract a list of preprocessor defines from a CMakeLists.txt file.  
    This is done by parsing statements where flags are set, such as 
      `SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -D__DEBUG")`
    The function assumes that the text file has been read entirely into a 
    variable so the file contents are passed into this function as a string.

    Parameters
    ----------
    cmake_content: str
            Full contents of the CMakeLists.txt file

    flags_keyword: str
            Filter, which flags should be looked at  
            (`CMAKE_C_FLAGS_DEBUG_` in the example above).

    Returns
    -------
    A list of all preprocessor defines matching the given flags_keyword. 
    Duplicates are removed.
    """
    regex = "SET\\(" + flags_keyword + \
        "\\s*\"\\${" + flags_keyword + "}\\s*-D([^\"]*)\"\\)"

    defs = re.findall(regex, cmake_content)

    # remove duplicates
    defs = list(dict.fromkeys(defs))
    print(len(defs), "compiler defines")
    return defs

def cmake_get_projectname(cmake_content: str) -> str:
    """ extract the project name from a CMakeLists.txt file 
    
    Parameters
    ----------
    cmake_content: str
            Full contents of the CMakeLists.txt file

    Returns
    -------
    project name (which is typically used as the name of the .elf output file)
    """

    regex = "SET\(ProjectName (\w*)"
    projectname = re.findall(regex, cmake_content)
    if len(projectname) == 1:
        return projectname[0]
    else:
        return ""


def cmake_get_jlink_cpu(cmake_content: str) -> str:
    """ extract the CPU name from a CMakeLists.txt file in the form as it is 
    used by the J-Link debugger.  
    
    Parameters
    ----------
    cmake_content: str
            Full contents of the CMakeLists.txt file

    Returns
    -------
    CPU name as it is uesd in the J-Link debugger
    """

    regex = "-DCPU_(\\w+)"
    cpus = re.findall(regex, cmake_content)

    # remove duplicates
    cpus = list(dict.fromkeys(cpus))
    assert(len(cpus) == 1)
    
    if cpus[0] == "MCIMX7D_M4":
        jlink_cpu = "MCIMX7D5_M4"
    
    elif cpus[0] == "MIMX8QM6AVUFF_cm4_core0":
        jlink_cpu = "MIMX8QM6_M4_0"

    elif cpus[0] == "MIMX8QM6AVUFF_cm4_core1":
        jlink_cpu = "MIMX8QM6_M4_1"

    elif cpus[0] == "MIMX8QX6AVLFZ":
        jlink_cpu = "MIMX8QX6_M4"

    else:
        assert(False)
        jlink_cpu = "[UNKNOWN CPU]"

    return jlink_cpu
