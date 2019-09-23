"""Create <projectname>.code-workspace file for VSCode

This file can also be imported as a module and contains the following
functions:

    * create_workspace()

__author__     = "Andy Kiser"
__copyright__  = "Copyright 2019, Toradex AG"
__license__    = "Exclusively for use in projects based on Toradex products"
__version__    = "1.1.0"
__maintainer__ = "Andy Kiser"
__email__      = "andy.kiser@toradex.com"
__status__     = "dev"
"""

import os
import json
from pathlib import Path, PosixPath


def create_workspace(project_name: str, paths: dict, dirs: dict) -> str:
    """
    return the contents of a '<projectname>.code-workspace' file, generated
    from a template file and the 'CMakeLists.txt' file

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
    The contents of the full *.code-workspace file
    """

    # read the template
    f = open(paths["template.code-workspace"], 'r')
    workspace_template_txt = f.read()
    f.close()

    # replace the path in the template string
    rel = os.path.relpath(dirs["repository_root"], dirs[".vscode"])
    workspace_template_txt = workspace_template_txt.replace(
        "<path-to-socroot>", rel)

    # Parse json file
    # Actually not required. I leave the code in for potential future
    # extensions, where modifying the json content is easier.
    workspace_json = json.loads(workspace_template_txt)

    # for projects based on the rpmsg library, add the additional workspace
    # folder 'open-amp to the multi-folder workspace.
    RPMSG_PROJECTS = ["gpio_freertos", "pingpong_bm",
                      "pingpong_freertos", "str_echo_bm", "str_echo_freertos"]

    if project_name in RPMSG_PROJECTS:
        s = workspace_json["folders"][1]
        p = s["path"].replace("/platform/devices",
                              "/middleware/multicore/open-amp")
        s["path"] = s["name"] = p
        workspace_json["folders"].append(s)

    return json.dumps(workspace_json, indent=4, sort_keys=False)
