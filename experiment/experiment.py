import utils
import itertools
import os
import re
from subprocess import PIPE, run
import shutil

class ExperimentInstance:
    # Specify the output folder and the .exe path
    def __init__(self, output_folder: str, exe_path: str, clear_folder = False):
        if os.path.exists(output_folder) and clear_folder:
            shutil.rmtree(output_folder)
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
        # validate the exe path
        if not os.path.exists(exe_path):
            raise Exception("Exe {} does not exist".format(exe_path))
        self.output_folder = output_folder
        self.exe_path = exe_path

    def get_command(self, config: dict):
        cmd = self.exe_path
        for key, value in config.items():
            if key == "size":
                cmd += " -Dframebuffer_width=" + str(value) + " -Dframebuffer_height=" + str(value)
            else:
                cmd += " -D" + key + "=" + str(value)
        return cmd

    def validate_configs(self, configs: dict):
        for key, value in configs.items():
            if not isinstance(key, str):
                print("Error: Wrong key type for: ", key)
                raise TypeError("Config keys should be string")
            if not isinstance(value, list):
                print("Error: Wrong key type for: ", value)
                raise TypeError("Config values should be a list containing strings")
            for e in value:
                if not isinstance(e, str):
                    raise TypeError("Elements in config values should be strings")      
      
    # run simulation with the config combinations
    # The dict should be configs[key: str] = value -> list
    # e.g., configs["scenes"] = ["sponza", "san-miguel"]
    def run(self, configs: dict):
        # validate the configs first
        self.validate_configs(configs)

        config_names = [key for key, value in configs.items()]

        # Get the lists from the dictionary
        lists = configs.values()

        # Generate all combinations
        combinations = itertools.product(*lists)

        # enumerate all combinations
        for combo in combinations:
            if len(combo) != len(config_names):
                raise Exception("Really Stupid Bug")
            config = dict(zip(config_names, combo))
            
            os_cmd = self.get_command(config)

            hashed_name = utils.HashFilename(config)
            output_file = self.output_folder + "/" + hashed_name + ".txt"
            os_run = False

            print("Running command: ", os_cmd)
            if os_run:
                os.system(os_cmd)
            else:
                result = run(os_cmd, stdout=PIPE, stderr=PIPE,
                        universal_newlines=True, shell=True)
                log_str = result.stdout
                with open(output_file, 'w') as file:
                    file.write(log_str)
                    file.close()
            print("Finished.")