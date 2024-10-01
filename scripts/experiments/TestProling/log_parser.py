from matplotlib import pyplot as plt, ticker as mticker, colors as mcolors
import numpy as np
from math import pi, sqrt, exp
import sys, re

def gauss(n=11,sigma=1):
    r = range(-int(n/2),int(n/2)+1)
    return [1 / (sigma * sqrt(2*pi)) * exp(-float(x)**2/(2*sigma**2)) for x in r]

log_file_name = "DualStreaming_san-miguel_1024_log.txt"
print("Parsing: " + log_file_name)

dram_header_regex = r'-+DRAM-+'
bandwidth_utilization_regex = r'={2,}\s*Detailed Bandwidth Utilization:\s*={2,}'
end_line_regex = r'^=+\s*$'
detail_pattern = r'Unit name:\s*(.*?),\s*Request label:\s*(.*?),\s*Bandwidth Utilization:\s*([\d.]+)\s*bytes/cycle'
all_pairs = {}
cycles = []

with open(log_file_name) as f:
    lines = f.readlines()  # List containing lines of file
    
    # Find all pairs for the first iteration
    under_DRAM = False
    under_detailed = False
    length = 0
    for line in lines:

        if 'Cycle:' in line:
            cycles.append(int(re.findall(r'[\d]+', line)[0]))

        if re.search(dram_header_regex, line):
            under_DRAM = True
            length += 1
        
        if under_DRAM and re.search(bandwidth_utilization_regex, line):
            under_detailed = True

        match = re.search(detail_pattern, line)
        if under_detailed and match:
            unit_name = match.group(1)
            request_label = match.group(2)
            all_pairs[(unit_name, request_label)] = []

        if under_DRAM and under_detailed and re.search(end_line_regex, line):
            under_DRAM = False
            under_detailed = False

    # Fill out the lists
    under_DRAM = False
    under_detailed = False
    t = -1
    for line in lines:
        if re.search(dram_header_regex, line):
            under_DRAM = True
            t += 1
            for key in all_pairs:
                all_pairs[key].append(float(0))
        
        if under_DRAM and re.search(bandwidth_utilization_regex, line):
            under_detailed = True

        match = re.search(detail_pattern, line)
        if under_detailed and match:
            unit_name = match.group(1)
            request_label = match.group(2)
            bandwidth = float(match.group(3))
            all_pairs[(unit_name, request_label)][t] = bandwidth
            
        if under_DRAM and under_detailed and re.search(end_line_regex, line):
            under_DRAM = False
            under_detailed = False

# Plotting the data
plt.figure(figsize=(12, 6))  # Set the figure size

kernel_size = 61
kernel = np.array(gauss(kernel_size, 5))
kernel = kernel / kernel.sum()

print(len(cycles))
print(cycles)
print(length)
# exit()

for (unit_name, request_label), bandwidths in all_pairs.items():
    if len(bandwidths) == len(cycles):
        smoothed = np.convolve(bandwidths, kernel, mode='same')
        plt.plot(cycles, smoothed, label=f"{unit_name} - {request_label}")

# exit()
plt.xticks(np.arange(0, cycles[-1], cycles[-1] / 10))
plt.yticks(np.arange(0, 200, 40))
plt.grid()

plt.title("DRAM Bandwidth Utilization Over Time")
plt.xlabel("Cycles")
plt.ylabel("Bandwidth (bytes/cycle)")
plt.legend()
plt.show()
