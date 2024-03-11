import os
from subprocess import PIPE, run
EXE_PATH = R"..\..\build\src\arches-v2\Arches-v2.exe"

default_config = {}
default_config["scene_name"] = "sponza"
default_config["framebuffer_width"] = 1024
default_config["framebuffer_height"] = 1024
default_config["traversal_scheme"] = 1
default_config["use_early"] = 1
default_config["hit_delay"] = 0
# 1
scene_list = ["sponza", "san-miguel", "hairball"]
# scene_list = ["sponza"]
size_list = [1024]
weight_schemes = [0, 1, 2]

# size_list = [256, 1024]
# scheme_list = [0, 1]

def get_command(config: dict):
    cmd = EXE_PATH
    for key, value in config.items():
        cmd += " -D" + key + "=" + str(value)
    print(cmd)
    return cmd

def run_config(config: dict, os_run: bool = True):
    if os.path.exists(EXE_PATH):
        pass
    else:
        print("Can not find the exe file")
    cmd = get_command(config)
    test_name = config["scene_name"] + "_" + str(config["weight_scheme"]) + "_" + str(config["framebuffer_width"])
    if os_run:
        os.system(cmd)
    else:
        result = run(cmd, stdout=PIPE, stderr=PIPE,
                 universal_newlines=True, shell=True)
        log_str = result.stdout
        err_str = result.stderr
        
        with open('{}_log.txt'.format(test_name), 'w') as file:
            file.write(log_str)
            file.close()
        with open('{}_err.txt'.format(test_name), 'w') as file:
            file.write(err_str)
            file.close()
    print("{} Finished".format(test_name))

if __name__ == "__main__":
    for size in size_list:
        for scene in scene_list:
            for weight_scheme in weight_schemes:
                config = default_config 
                config["scene_name"] = scene
                config["weight_scheme"] = weight_scheme
                run_config(config, os_run=False)