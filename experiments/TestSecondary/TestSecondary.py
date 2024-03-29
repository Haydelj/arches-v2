import os
from subprocess import PIPE, run
EXE_PATH = R"..\..\build\src\arches-v2\Arches-v2.exe"

default_config = {}
default_config["scene_name"] = "sponza"
default_config["framebuffer_width"] = 256
default_config["framebuffer_height"] = 256
default_config["traversal_scheme"] = 0
default_config["simulator"] = 0 # 0 - trax, 1 - dual-streaming
default_config["use_secondary_rays"] = 1

scene_list = ["san-miguel", "hairball"]
# scene_list = ["sponza"]
size_list = [1024]
scheme_list = [0, 1]
early_list = [0, 1] 
delay_list = [0]
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
    simulator = "TRaX" if config["simulator"] == 0 else "DualStreaming"
    scene = config["scene_name"]
    size = str(config["framebuffer_width"])
    test_name = simulator + "_" + scene + "_" + size
    if simulator == "DualStreaming":
        early = "Early" if config["use_early"] == 1 else "NoEarly"
        scheme = "BFS" if config["traversal_scheme"] == 0 else "DFS"
        test_name = test_name + "_" + early + "_" + scheme


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
    # trax
    for scene in scene_list:
        for size in size_list:
            config = default_config
            config["simulator"] = 0
            config["framebuffer_width"] = config["framebuffer_height"] = size
            config["scene_name"] = scene
            run_config(config, os_run=False)

    quit()
    for size in size_list:
        for scene in scene_list:
            for scheme in scheme_list:
                for early in early_list:
                    for delay in delay_list:
                        config = default_config
                        if early == 0 and delay == 1:
                            continue
                        config["simulator"] = 1
                        config["framebuffer_width"] = size
                        config["framebuffer_height"] = size
                        config["scene_name"] = scene
                        config["traversal_scheme"] = scheme
                        config["use_early"] = early
                        config["hit_delay"] = delay
                        run_config(config, os_run=False)