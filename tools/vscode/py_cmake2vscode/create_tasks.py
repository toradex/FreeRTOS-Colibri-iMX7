"""Create tasks.json file for VSCode



This file can also be imported as a module and contains the following
functions:

    * create_tasks()

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
from cmakelists_parser import cmake_get_configurations


def cmakelists_to_tasks(paths: dict, dirs: dict) -> str:
    """
    return the contents of a 'tasks.json' file, generated from a template file and the
    'CMakeLists.txt' file

    Parameters
    ----------
    paths: dict
        a list of file names (all are absolute paths)
    dirs: dict
        a list of folders names (all are absolute)

    Returns
    -------
    The contents of the full tasks.json file
    """

    # read contents of tasks_template.json and CmakeLists.txt
    f = open(paths["tasks_template.json"], 'r')
    tasks_template_txt = f.read()
    f.close()

    f = open(paths["CMakeLists.txt"], 'r')
    cmakelists_txt = f.read()
    f.close()

    configs = cmake_get_configurations(cmakelists_txt, "CMAKE_C_FLAGS_")

    if not os.path.exists(paths["armgcc.cmake"]):
        raise Exception(
            "armgcc.cmake not found in the expected location. Build will not work")

    rel = os.path.relpath(paths["armgcc.cmake"], dirs["armgcc"])
    t = tasks_template_txt.replace(
        "${config:toradex_freertos.cmake_toolchain_file}", rel)

    # parse json template, extract tasks from template
    tasks_json = json.loads(t)
    str_cmake_task_template = json.dumps(tasks_json['tasks'][0])
    str_build_task_template = json.dumps(tasks_json['tasks'][1])

    # remove template tasks from the json data (we will add them again later)
    tasks_json['tasks'] = []

    # Add CMake and build tasks for each configuration
    for config_name in configs:
        # prepare CMake task
        str_cmake_task = str_cmake_task_template.replace(
            "<configuration>", config_name)
        str_cmake_task = str_cmake_task.replace(
            "<configuration2>", config_name.lower()
        )
        cmake_task = json.loads(str_cmake_task)

        # prepare build task, remove "default" for any but the first
        # configuration
        str_build_task = str_build_task_template.replace(
            "<configuration>", config_name)
        str_build_task = str_build_task.replace(
            "<configuration2>", config_name.lower()
        )
        build_task = json.loads(str_build_task)
        if (config_name != configs[0]):
            build_task['group'] = cmake_task['group']

        # insert tasks into the json data structure
        tasks_json['tasks'].append(cmake_task)
        tasks_json['tasks'].append(build_task)

    return json.dumps(tasks_json, indent=4, sort_keys=False)
