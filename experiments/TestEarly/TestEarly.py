import os
from subprocess import PIPE, run
EXE_PATH = R"..\..\build\src\arches-v2\Arches-v2.exe"

default_config = {}
default_config["scene_name"] = "sponza"
default_config["framebuffer_width"] = 256
default_config["framebuffer_height"] = 256
default_config["traversal_scheme"] = 0

# scene_list = ["sponza", "san-miguel", "hairball"]
scene_list = ["sponza"]
size_list = [256]
scheme_list = [1]
early_list = [0, 1]
delay_list = [0, 1]
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
    test_name = ("Early" if config["use_early"] else "NoEarly") + "_" + ("Delay" if config["hit_delay"] else "NoDelay") + "_" + config["scene_name"] + "_" + ("Breadth" if config["traversal_scheme"] == 0 else "Depth") + str(config["framebuffer_width"])
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
            for scheme in scheme_list:
                for early in early_list:
                    for delay in delay_list:
                        config = default_config
                        if early == 0 and delay == 1:
                            continue
                        config["framebuffer_width"] = size
                        config["framebuffer_height"] = size
                        config["scene_name"] = scene
                        config["traversal_scheme"] = scheme
                        config["use_early"] = early
                        config["hit_delay"] = delay
                        run_config(config, os_run=False)