import os
import json
import subprocess
import shutil

def get_test_configs():
    framebuffer_dim = 1024

    scene_id_map = {"sponza": 0, "intel-sponza" : 1 , "san-miguel" : 2, "hairball" : 3, "cornellbox" : 4 , "living_room" : 5}
    

    base_config = {
        "simulator": 0,
        "logging_interval": 10000,
        "scene_id": 0,
        "scene_name": "sponza",
        "framebuffer_width": framebuffer_dim,
        "framebuffer_height": framebuffer_dim,
        "total_threads": framebuffer_dim << 10,
        "warm_l2": 1,
        "pregen_rays": 1,
        "pregen_bounce": 2,

        #Dual Streaming
        "use_scene_buffer": 0,
        "rays_on_chip": 1,
        "hits_on_chip": 1,
        "use_early": 0,
        "hit_delay": 0,
        "hit_buffer_size": 64 * framebuffer_dim,
        "traversal_scheme": 1,
        "weight_scheme": 1,
    }
    
    test_sim_types = [0,1]
    test_scenes = ["sponza", "intel-sponza" , "san-miguel" , "hairball" , "cornellbox"]
    test_bounce_types = [0,1,2]

    configs = []
    for sim_type in test_sim_types:
        for scene in test_scenes:
            for bounce_type in test_bounce_types:
                config = base_config.copy()
                config["simulator"] = sim_type
                config["scene_id"] = scene_id_map.get(scene, f"unknown_{scene}")
                config["scene_name"] = scene
                config["pregen_bounce"] = bounce_type
                configs.append(config)

    return configs


def format_config_as_args(config):
    args = []
    for key, value in config.items():
        if isinstance(value, dict):
            value = json.dumps(value)
        args.append(f"--{key}={value}")
    return args


def generate_log_filename(config, log_dir="logs"):
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    sim_type_map = {0: "trax", 1: "dualstreaming"}
    sim_type_name = sim_type_map.get(config["simulator"], f"unknown_{config['simulator']}")

    filename = f"{sim_type_name}_scene_{config['scene_name']}_bounce_{config['pregen_bounce']}.log"
    return os.path.join(log_dir, filename)


def generate_image_filename(config, output_dir="images"):
    """Generate a unique image filename based on the configuration."""
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    sim_type_map = {0: "trax", 1: "dualstreaming"}
    sim_type_name = sim_type_map.get(config["simulator"], f"unknown_{config['simulator']}")

    filename = f"{sim_type_name}_scene_{config['scene_name']}_bounce_{config['pregen_bounce']}.png"
    return os.path.join(output_dir, filename)


def move_output_image(config, output_path):
    """Move or rename the output image to a named file based on the configuration."""
    target_image_path = generate_image_filename(config)
    if os.path.exists(output_path):
        shutil.move(output_path, target_image_path)
        print(f"Image saved as: {target_image_path}")
    else:
        print(f"Output image not found at: {output_path}")


def execute_program(program_path, configs, working_directory, output_image="out.png"):
    original_directory = os.getcwd()
    os.chdir(working_directory)

    try:
        for idx, config in enumerate(configs):
            args = format_config_as_args(config)
            cmd = [program_path] + args
            log_file = generate_log_filename(config)

            print(f"[{idx + 1}/{len(configs)}] Executing: {' '.join(cmd)}")
            print(f"Logs will be saved to: {log_file}")

            with open(log_file, "w") as log:
                try:
                    subprocess.run(cmd, stdout=log, stderr=log, check=True, shell=False)
                    # Move or rename the output image
                    move_output_image(config, output_image)
                except subprocess.CalledProcessError as e:
                    log.write(f"\nError: Command failed with exit code {e.returncode}\n")
                except Exception as ex:
                    log.write(f"\nUnexpected error occurred: {str(ex)}\n")
    finally:
        os.chdir(original_directory)


def main():
    program_path = os.path.abspath(r"..\..\build\src\arches-v2\Release\Arches-v2.exe")
    working_directory = os.path.abspath(r"..\..\build\src\arches-v2\Release")

    if not os.path.exists(program_path):
        raise FileNotFoundError(f"Executable not found at: {program_path}")

    if not os.path.exists(working_directory):
        raise FileNotFoundError(f"Working directory not found: {working_directory}")

    configs = get_test_configs()
    execute_program(program_path, configs, working_directory)


if __name__ == "__main__":
    main()
