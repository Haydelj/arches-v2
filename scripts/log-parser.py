import sys
import re

from matplotlib import pyplot as plt, ticker as mticker
import numpy as np

from math import pi, sqrt, exp

def gauss(n=11,sigma=1):
    r = range(-int(n/2),int(n/2)+1)
    return [1 / (sigma * sqrt(2*pi)) * exp(-float(x)**2/(2*sigma**2)) for x in r]

#keys = ['L1d$ Read', 'L2$ Read', 'DRAM Read']
keys = ['DRAM Total', 'DRAM Write', 'DRAM Read']
colors = ['-r', '-g', '-b']
cycles = []
data = []
for key in keys:
    data.append([])

# log_file_name = sys.argv[1]
log_file_name = "C:\\ExperimentEnvs\\arches-main\\scripts\\experiments\\TestBounces\\san-miguel-512-0-ds.log"
# log_file_name = "C:\\ExperimentEnvs\\arches-main\\scripts\\experiments\\TestBounces\\sponza-512-0-ds.log"
# log_file_name = "C:\\ExperimentEnvs\\arches-main\\scripts\\experiments\\TestBounces\\san-miguel-512-0-trax.log"
print("Parsing: " + log_file_name)
with open(log_file_name) as f:
    lines = f.readlines() #list containing lines of file
    for line in lines:
        if 'Cycle:' in line:
            cycles.append(int(re.findall(r'[\d]+', line)[0]))

        for i in range(len(keys)):
            if(keys[i] in line):
                data[i].append(float(re.findall(r'[\d]*[.][\d]+', line)[0]))


show_unsmoothed = True
if show_unsmoothed:
    for i in range(len(keys)):
        if len(data[i]) == len(cycles):
            plt.plot(cycles, data[i], colors[i], alpha=0.125)

kernel_size = len(cycles) / 64
kernel = gauss(kernel_size, kernel_size / 4)

for i in range(len(keys)):
    if len(data[i]) == len(cycles):
        plt.plot(cycles, np.convolve(data[i], kernel, mode='same'), colors[i], label=keys[i])

plt.legend(loc="upper right")
plt.ylabel('Bandwidth (bytes/cycle)')
plt.xlabel('Time (cycles)')
#plt.yscale('log')
plt.grid()
plt.show()