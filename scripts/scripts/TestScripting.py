import os
import sys
current_file_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_file_dir + r"\.."))
from experiment import ExperimentInstance

## Do your config here
## config[name -> str] = str list
output_folder_name = r"TestScripting"
configs = {}
configs["simulator"] = ["trax", "dual-streaming"]
configs["scene_name"] = ["sponza", "san-miguel"]
configs["size"] = ["128", "512"]


last_level_folder = os.path.basename(current_file_dir)
if last_level_folder != "scripts":
    raise Exception("This file should be defined in 'experiment/scripts/' folder")

abs_exe_path = os.path.join(current_file_dir, r"..\..\build\src\arches-v2\Release\arches-v2.exe")
abs_output_dir = os.path.join(current_file_dir, r"..\results", output_folder_name)
exp_instance = ExperimentInstance(abs_output_dir, abs_exe_path, clear_folder = True)

# Let's run simulations and generate results
exp_instance.run(configs=configs)

