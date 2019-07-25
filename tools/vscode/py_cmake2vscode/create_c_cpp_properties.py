"""Convert Cmake project files to VSCode workspace for C_CPP extension

This script walks throug all subfolders and searches for Cmake project files.

Each project file gets converted into a VSCode settings file.


This file can also be imported as a module and contains the following
functions:

    * cmake_get_CMakeLists_recursive - Recursively get a list of all 
                                       CMakeLists.txt fiels
    * cmake_to_vscode - Create VSCode configuration files for one 
                        CMakeLists.txt file 

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
from cmakelists_parser import cmake_get_includes, cmake_get_defines, cmake_get_configurations
import os


def get_c_cpp_properties(paths: dict, dirs: dict) -> str:
    """ Read one file CMakeLists.txt and create a VSCode settings file out of it.  
    It is assumed that the CMakeLists.txt is located inside a folder armcc.  
    The vscode folder is copied to the same base folder, so hat it stands in 
    parallel to armcc.

    Parameters
    ----------
    paths: dict
        a list of file names (all are absolute paths)
    dirs: dict
        a list of folders names (all are absolute)

    Returns
    -------
    The contents of the 'c_cpp_properties.json' file as a string
    """
    # read the file contents
    f = open(paths["CMakeLists.txt"], 'r')
    file_content = f.read()
    f.close()

    # the whole json file is represented by a dict:
    json_data = dict()
    json_data['version'] = 4
    json_data['configurations'] = list()

    # add include folders to the 'env' section.
    cmake_includes = cmake_get_includes(file_content)
    #json_data['env']['global_inc_paths'] = cmake_get_includes(file_content)

    # include folders need to be adjusted:
    # - in 'CMakeLists.txt' folders are relative to the <project>/armgcc/ folder
    # - in 'c_cpp_properties.json', folders are relative to the <project> folder
    # - Include folders in c_cpp_properties.json should end with a slash "/",
    #   otherwise all subfolders are part of the search path, too.
    all_include_dirs = []
    for include_dir in cmake_includes:
        include_dir = include_dir.replace("${ProjDirPath}", dirs["armgcc"])
        include_dir_rel = os.path.relpath(include_dir, dirs["${workspaceFolder}"])
        all_include_dirs.append(include_dir_rel + "/")

    print(len(all_include_dirs), " unique include directories")

    # evaluate all C configurations that appear in the CMakeLists.txt file
    c_configs = cmake_get_configurations(file_content, "CMAKE_C_FLAGS_")
    print(len(c_configs), " unique configurations")

    # add all configurations to the json structure
    json_data['configurations'] = list()
    for config_name in c_configs:
        configuration = dict()
        configuration['name'] = config_name
        configuration['compilerPath'] = '\"${env:ARMGCC_DIR}/bin/arm-none-eabi-gcc\"'
        configuration['cStandard'] = 'c99'
        configuration['cppStandard'] = 'c++14'
        configuration['intelliSenseMode'] = 'gcc-x64'
        configuration['includePath'] = all_include_dirs
        configuration['defines'] = cmake_get_defines(
            file_content, "CMAKE_C_FLAGS_" + config_name.upper())
        json_data['configurations'].append(configuration)

    # print the full json file content (for debug, instead of writing it to a file)
    cpp_props = json.dumps(json_data, indent=4)

    return cpp_props
