import os
from subprocess import PIPE, run
EXE_PATH = R"..\..\..\build\src\arches-v2\Release\Arches-v2.exe"

default_config = {}
default_config["simulator"] = 0
default_config["logging_interval"] = 1 * 1024

default_config["scene_name"] = "sponza"
default_config["framebuffer_width"] = 512
default_config["framebuffer_height"] = 512
default_config["pregen_rays"] = 0
default_config["pregen_bounce"] = 0

default_config["use_scene_buffer"] = 1
default_config["rays_on_chip"] = 0
default_config["use_early"] = 0
default_config["hit_delay"] = 0
default_config["hit_buffer_size"] = 1024 * 1024
default_config["traversal_scheme"] = 0
default_config["weight_scheme"] = 1
default_config["dynamic_prefetch"] = 0

# 1
scenes = ["sponza", "san-miguel"]
# scenes = ["san-miguel"]
simulators = [1]
dynamic_prefetchs = [0, 1]

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
    test_name = config["scene_name"] + "-" + str(config["framebuffer_height"]) + "-" + str(config["dynamic_prefetch"]) + "-" + ("trax" if config["simulator"] == 0 else "ds")
    if os_run:
        os.system(cmd)
    else:
        result = run(cmd, stdout=PIPE, stderr=PIPE,
                 universal_newlines=True, shell=True)
        log_str = result.stdout     
        with open('{}.log'.format(test_name), 'w') as file:
            file.write(log_str)
            file.close()
        #with open('{}_err.txt'.format(test_name), 'w') as file:
        #    file.write(err_str)
        #    file.close()
    print("{} Finished".format(test_name))

if __name__ == "__main__":
    for scene in scenes:
        for dynamic_prefetch in dynamic_prefetchs:
             for simulator in simulators:
                config = default_config
                config["scene_name"] = scene
                config["dynamic_prefetch"] = dynamic_prefetch
                config["simulator"] = simulator
                run_config(config, os_run=False)