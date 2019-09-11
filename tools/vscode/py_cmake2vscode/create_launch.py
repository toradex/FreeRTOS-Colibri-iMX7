"""Create launch.json file for VSCode

This file can also be imported as a module and contains the following
functions:

    * create_launch()

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
import os
from pathlib import Path, PosixPath
from cmakelists_parser import cmake_get_configurations, cmake_get_jlink_cpu


def create_launch(project_name: str, paths: dict, dirs: dict) -> str:
    """
    return the contents of a 'launch.json' file, generated from a template 
    file and the 'CMakeLists.txt' file

    Parameters
    ----------
    project_name: str
        name of the current project, equal to folder name
    paths: dict
        a list of file names (all are absolute paths)
    dirs: dict
        a list of folders names (all are absolute)

    Returns
    -------
    The contents of the full tasks.json file
    """

    # read contents of launch_template.json and CmakeLists.txt
    f = open(paths["launch_template.json"], 'r')
    launch_template_txt = f.read()
    f.close()

    f = open(paths["CMakeLists.txt"], 'r')
    cmakelists_txt = f.read()
    f.close()

    # parse json template, remove "configurations" section
    # (we will later add it again)
    launch_json = json.loads(launch_template_txt)
    
    cfg_jlink = launch_json["configurations"][0]
    assert(cfg_jlink["servertype"] == "jlink")
    cfg_openocd = launch_json["configurations"][1]
    assert(cfg_openocd["servertype"] == "openocd")
    
    launch_json["configurations"] = []

    # ==== J-Link specific settings: ====
    # parse Segger J-Link CPU name from CmakeLists.txt
    jlink_cpu = cmake_get_jlink_cpu(cmakelists_txt)

    cfg_jlink["device"] = jlink_cpu
    cfg_jlink["name"] = f"J-Link Debug {project_name}.elf"
    cfg_jlink["executable"] = \
        os.path.relpath(paths["debug.elf"], dirs["${workspaceFolder}"])
 
    # As an option the precise J-Link Pro adapter can be specified
    # cfg_jlink["ipAddress"] = "null"
    # cfg_jlink["serialNumber"] = "null"

    # ==== OpenOCD specific settings: ====
    cfg_openocd["name"] = f"OpenOcd Debug {project_name}.elf"
    cfg_openocd["executable"] = \
        os.path.relpath(paths["debug.elf"], dirs["${workspaceFolder}"])
  
    # add the J-Link and openocd configurations to launch.json
    launch_json["configurations"].append(cfg_jlink)
    launch_json["configurations"].append(cfg_openocd)
    
    return json.dumps(launch_json, indent=4, sort_keys=False)
    