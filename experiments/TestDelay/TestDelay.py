import os
from subprocess import PIPE, run
EXE_PATH = R"..\..\build\src\arches-v2\Arches-v2.exe"

default_config = {}
default_config["scene_name"] = "sponza"
default_config["framebuffer_width"] = 1024
default_config["framebuffer_height"] = 1024
default_config["traversal_scheme"] = 1
default_config["simulator"] = 1 # 0 - trax, 1 - dual-streaming
default_config["secondary_rays"] = 0
default_config["hit_buffer_size"] = 128
default_config["hit_delay"] = 0
default_config["use_early"] = 1

scene_list = ["sponza", "san-miguel", "hairball"]
delay_list = [0, 1]
hit_buffer_size = [128, 128 * 1024]

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
    test_name = scene + "_" + size
    hit_buffer_size = str(config["hit_buffer_size"])
    delay = "Delay" if config["hit_delay"] == 1 else "NoDelay"
    test_name = test_name + "_" + delay + "_" + hit_buffer_size
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
    for scene in scene_list:
        for delay in delay_list:
            for size in hit_buffer_size:
                config = default_config
                config["simulator"] = 1
                config["scene_name"] = scene
                config["hit_delay"] = delay
                config["hit_buffer_size"] = size
                run_config(config, os_run=False)