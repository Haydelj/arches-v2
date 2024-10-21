import os
import sys
current_file_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_file_dir + r"\.."))
from experiment import ExperimentInstance
import utils
from matplotlib import pyplot as plt, ticker as mticker, colors as mcolors
from math import pi, sqrt, exp
import numpy as np

Run = True
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

if Run:
    exp_instance = ExperimentInstance(abs_output_dir, abs_exe_path, clear_folder = True)
    # Let's run simulations and generate some results
    exp_instance.run(configs=configs)

quit()
# Let's try to visualize the logs

# Example 1: Comparing Mrays/s for trax and dual-streaming on 2 scenes




# Example 2: Visualize the bandwidth of two simulator on san-miguel
config = {}
config["scene_name"] = "san-miguel"
config["size"] = "512"

config["simulator"] = "trax"
filename = utils.HashFilename(config) + ".txt"
file_path = os.path.join(abs_output_dir, filename)
trax_cycles, trax_bandwidth_total, trax_bandwidth_details = utils.GetBandwidthList(file_path, "DRAM")

config["simulator"] = "dual-streaming"
filename = utils.HashFilename(config) + ".txt"
file_path = os.path.join(abs_output_dir, filename)
ds_cycles, ds_bandwidth_total, ds_bandwidth_details = utils.GetBandwidthList(file_path, "DRAM")

smoothing = True
def gauss(n=11,sigma=1):
    r = range(-int(n/2),int(n/2)+1)
    return [1 / (sigma * sqrt(2*pi)) * exp(-float(x)**2/(2*sigma**2)) for x in r]
kernel_size = 61
kernel = np.array(gauss(kernel_size, 5))
kernel = kernel / kernel.sum()

print(trax_cycles)
print(trax_bandwidth_total)
quit()
# Let's use 3 plots (1 for comparison, 1 for trax, 1 for ds)
plt.figure(figsize=(12, 4))

plt.subplot(1, 3, 1)
y = trax_bandwidth_total
if smoothing:
    y = np.convolve(y, kernel, mode='same')
plt.plot(trax_cycles, y, label = "trax")

y = ds_bandwidth_total
if smoothing:
    y = np.convolve(y, kernel, mode='same')
plt.plot(ds_cycles, y, label = "dual-streaming")



plt.subplot(1, 3, 2)

plt.subplot(1, 3, 3)
plt.legend()
plt.show()