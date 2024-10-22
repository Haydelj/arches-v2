import os
import sys
current_file_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_file_dir + r"\.."))
from experiment import ExperimentInstance
import utils
from matplotlib import pyplot as plt, ticker as mticker, colors as mcolors
from math import pi, sqrt, exp
import numpy as np
import itertools

Run = False
clear_folder = False
## Do your config here
## config[name -> str] = str list
output_folder_name = r"TestScripting"
configs = {}
configs["simulator"] = ["trax", "dual-streaming"]
configs["scene_name"] = ["sponza", "san-miguel"]
configs["size"] = ["1024"]


last_level_folder = os.path.basename(current_file_dir)
if last_level_folder != "scripts":
    raise Exception("This file should be defined in 'experiment/scripts/' folder")

abs_exe_path = os.path.join(current_file_dir, r"..\..\build\src\arches-v2\Release\arches-v2.exe")
abs_output_dir = os.path.join(current_file_dir, r"..\results", output_folder_name)

if Run:
    exp_instance = ExperimentInstance(abs_output_dir, abs_exe_path, clear_folder = clear_folder)
    # Let's run simulations and generate some results
    exp_instance.run(configs=configs)

# quit()
# Let's try to visualize the logs

# Example 1: Comparing Mrays/s for trax and dual-streaming on 2 scenes
# This will be a histogram
# data_name = "MRays/s"
# vis_configs = configs
# vis_configs["size"] = ["512"]
# config_names = [key for key, value in vis_configs.items()]
# lists = vis_configs.values()
# combinations = itertools.product(*lists)

# for combo in combinations:
#     config = dict(zip(config_names, combo))
#     filename = utils.HashFilename(config) + ".txt"
#     file_path = os.path.join(abs_output_dir, filename)
#     data = utils.ExtractLogInfo(file_path, data_name)
#     print(config, data)
# quit()
# Example 2: Visualize the bandwidth of two simulators on san-miguel
# This will be some charts containing the time-varying curves
config = {}
config["scene_name"] = "sponza"
config["size"] = "1024"
config["simulator"] = "trax"
memory_unit_name = "L2 Cache"
filename = utils.HashFilename(config) + ".txt"
file_path = os.path.join(abs_output_dir, filename)
trax_bandwidths = utils.GetBandwidth(file_path, memory_unit_name)
trax_cycles = trax_bandwidths[0]
trax_total_bandwidth = trax_bandwidths[1]
trax_detailed_bandwidth = trax_bandwidths[2]

config["simulator"] = "dual-streaming"
filename = utils.HashFilename(config) + ".txt"
file_path = os.path.join(abs_output_dir, filename)
ds_bandwidths = utils.GetBandwidth(file_path, memory_unit_name)
ds_cycles = ds_bandwidths[0]
ds_total_bandwidth = ds_bandwidths[1]
ds_detailed_bandwidth = ds_bandwidths[2]
smoothing = True
trax_kernel_size = min(31, len(trax_total_bandwidth) - 1)
trax_kernel = np.array(utils.gauss(trax_kernel_size, 5))
trax_kernel = trax_kernel / trax_kernel.sum()

ds_kernel_size = min(31, len(ds_total_bandwidth) - 1)
ds_kernel = np.array(utils.gauss(ds_kernel_size, 5))
ds_kernel = ds_kernel / ds_kernel.sum()

# Let's use 3 plots (1 for comparison, 1 for trax, 1 for ds)
plt.figure(figsize=(24, 8))
plt.subplot(1, 3, 1)
y = trax_total_bandwidth
if smoothing:
    y = np.convolve(y, trax_kernel, mode='same')
plt.plot(trax_cycles, y, label = "trax")
y = ds_total_bandwidth
if smoothing:
    y = np.convolve(y, ds_kernel, mode='same')
plt.plot(ds_cycles, y, label = "dual-streaming")
plt.title("Trax/Dual-Streaming Bandwidth Comparison")
plt.legend()

plt.subplot(1, 3, 2)
plt.title("Trax {} Bandwidth Utilization on {}".format(memory_unit_name, config["scene_name"]))
for (unit_name, request_label), bandwidths in trax_detailed_bandwidth.items():
    if len(bandwidths) == len(trax_cycles):
        smoothed = np.convolve(bandwidths, trax_kernel, mode='same')
        plt.plot(trax_cycles, smoothed, label=f"{unit_name} - {request_label}")
plt.legend()

plt.subplot(1, 3, 3)
plt.title("Dual-Streaming {} Bandwidth Utilization on {}".format(memory_unit_name, config["scene_name"]))
for (unit_name, request_label), bandwidths in ds_detailed_bandwidth.items():
    if len(bandwidths) == len(ds_cycles):
        smoothed = np.convolve(bandwidths, ds_kernel, mode='same')
        plt.plot(ds_cycles, smoothed, label=f"{unit_name} - {request_label}")
plt.legend()
plt.show()