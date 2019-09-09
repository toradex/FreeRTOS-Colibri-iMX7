#!/usr/bin/env python3
"""main.py: Create VSCode support files for all FreeRTOS demos.

Usage:
    python3 main.py <base_path>

The routine walks through all projects in <base_path> and all subfolders.
If <base_path> is not provided, "../../.." is taken as a default path,
which should be the main folder for the current SoC's M4 code.

The library does *not* create the c_cpp_properties.json files for the common
subprojects 
 - platform/devices, 
 - platform/drivers, 
 - platform/utilities and
 - rtos/FreeRTOS.

__author__     = "Andy Kiser"
__copyright__  = "Copyright 2019, Toradex AG"
__license__    = "Exclusively for use in projects based on Toradex products"
__version__    = "1.1.0"
__maintainer__ = "Andy Kiser"
__email__      = "andy.kiser@toradex.com"
__status__     = "dev"
"""

from cmakelists_parser import cmake_get_projectname
from create_c_cpp_properties import get_c_cpp_properties
from create_tasks import cmakelists_to_tasks
from create_workspace import create_workspace
from create_launch import create_launch
import shutil
from pathlib import Path, PosixPath
import sys
import os
import json

if len(sys.argv) > 1:
    walk_root_dir = os.path.abspath("../../..")
else:
    walk_root_dir = os.path.abspath(sys.argv[1])

print()
print("Converting armgcc to vscode in subfolders of" , walk_root_dir)
print()

# get a list of all 'CMakeLists.txt' files
cmakelists_files = []
for f in Path(walk_root_dir).glob('**/CMakeLists.txt'):
    cmakelists_files.append(os.path.abspath(f))

# walk through the list of all 'CMakeLists.txt' files
for cmakelists in cmakelists_files:
    # read the CMakeLists.txt file
    f = open(cmakelists, 'r')
    cmakelists_txt = f.read()
    f.close()

    # evaluate and store required directories (all are absolute)
    dirs = {}
    dirs["python"] = os.path.dirname(os.path.abspath(sys.argv[0]))
    dirs["armgcc"] = os.path.abspath(os.path.dirname(cmakelists))
    dirs["${workspaceFolder}"] = os.path.abspath(dirs["armgcc"] + "/..")
    dirs[".vscode"] = os.path.abspath(dirs["${workspaceFolder}"] + "/.vscode")

    r = dirs["armgcc"]
    while not os.path.exists(r + "/rtos"):
        r += "/.."
    dirs["repository_root"] = os.path.abspath(r)

    # evaluate the project name
    project_name = cmake_get_projectname(cmakelists_txt)
    if project_name == "":
        project_name = os.path.basename(dirs["${workspaceFolder}"])
 
   # evaluate and store full paths of required filenames (all are absolute)
    paths = {}
    paths["CMakeLists.txt"] = \
        cmakelists

    paths["template.code-workspace"] = \
        dirs["python"] + "/workspace_template.code-workspace"

    paths["*.code-workspace"] = \
        dirs[".vscode"] + "/" + project_name + ".code-workspace"

    paths["c_cpp_properties.json"] = \
        dirs[".vscode"] + "/c_cpp_properties.json"

    paths["tasks.json"] = \
        dirs[".vscode"] + "/tasks.json"

    paths["launch.json"] = \
        dirs[".vscode"] + "/launch.json"

    paths["armgcc.cmake"] = \
        dirs["repository_root"] + "/tools/cmake_toolchain_files/armgcc.cmake"

    paths["tasks_template.json"] = \
        dirs["python"] + "/tasks_template.json"

    paths["launch_template.json"] = \
        dirs["python"] + "/launch_template.json"

    paths["debug.elf"] = \
        dirs["armgcc"] + "/debug/" + project_name + ".elf"

    # print on console, which project is being processed
    print("\033[95m", dirs["${workspaceFolder}"], "\033[0m")

    # delete existing 'vscode' folder, and create a new empty vscode folder
    if os.path.exists(dirs[".vscode"]):
        shutil.rmtree(dirs[".vscode"])
    os.mkdir(dirs[".vscode"])

    # create workspace file
    ws_txt = create_workspace(project_name, paths, dirs)
    f = open(paths["*.code-workspace"], "w+")
    f.write(ws_txt)
    f.close()

    # create 'c_cpp_properties.json' file for intellisense
    c_cpp_properties_txt = get_c_cpp_properties(paths, dirs)
    f = open(paths["c_cpp_properties.json"], 'w')
    f.write(c_cpp_properties_txt)
    f.close()

    # create 'tasks.json' file with build commands
    tasks_json_txt = cmakelists_to_tasks(paths, dirs)
    f = open(paths["tasks.json"], 'w')
    f.write(tasks_json_txt)
    f.close()

    # create 'launch.json' file
    launch_txt = create_launch(project_name, paths, dirs)
    f = open(paths["launch.json"], "w+")
    f.write(launch_txt)
    f.close()
    