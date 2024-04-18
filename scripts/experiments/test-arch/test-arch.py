import os, sys
from subprocess import PIPE, run
EXE_PATH = R"..\..\..\build\src\arches-v2\Release\Arches-v2.exe"

default_config = {}
default_config["simulator"] = 0
default_config["logging_interval"] = 1 * 1024

default_config["scene_name"] = "sponza"
default_config["framebuffer_width"] = 512
default_config["framebuffer_height"] = 512
default_config["pregen_rays"] = 1
default_config["pregen_bounce"] = 0

arch = sys.argv[1]

if arch == 'trax':
    default_config["simulator"] = 0
elif arch == 'ds':
    default_config["simulator"] = 1
    default_config["use_scene_buffer"] = 1
    default_config["rays_on_chip"] = 0
    default_config["use_early"] = 0
    default_config["hit_delay"] = 0
    default_config["hit_buffer_size"] = 512 * 512
    default_config["traversal_scheme"] = 0
    default_config["weight_scheme"] = 1
elif arch == 'ocds':
    default_config["simulator"] = 1
    default_config["use_scene_buffer"] = 1
    default_config["rays_on_chip"] = 1
    default_config["use_early"] = 0
    default_config["hit_delay"] = 0
    default_config["hit_buffer_size"] = 512 * 512
    default_config["traversal_scheme"] = 0
    default_config["weight_scheme"] = 1
elif arch == 'ocds2':
    default_config["simulator"] = 1
    default_config["use_scene_buffer"] = 0
    default_config["rays_on_chip"] = 1
    default_config["use_early"] = 0
    default_config["hit_delay"] = 0
    default_config["hit_buffer_size"] = 512 * 512
    default_config["traversal_scheme"] = 0
    default_config["weight_scheme"] = 1

# 1
#scenes = ["sponza", "san-miguel", "hairball"]
scenes = ["san-miguel"]
bounces = [2]

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
    test_name = config["scene_name"] + "-" + str(config["framebuffer_height"]) + "-" + str(config["pregen_bounce"]) + "-" + arch;
    if os_run:
        os.system(cmd)
    else:
        result = run(cmd, stdout=PIPE, stderr=PIPE,
                 universal_newlines=True, shell=True)
        log_str = result.stdout
        err_str = result.stderr
        
        with open('{}.log'.format(test_name), 'w') as file:
            file.write(log_str)
            file.close()
        #with open('{}_err.txt'.format(test_name), 'w') as file:
        #    file.write(err_str)
        #    file.close()
    print("{} Finished".format(test_name))

if __name__ == "__main__":
    for scene in scenes:
        for bounce in bounces:
            config = default_config
            config["scene_name"] = scene
            config["pregen_bounce"] = bounce
            run_config(config, os_run=False)