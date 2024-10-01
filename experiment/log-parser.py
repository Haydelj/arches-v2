from matplotlib import pyplot as plt, ticker as mticker, colors as mcolors
import numpy as np
from math import pi, sqrt, exp
import sys, re

def gauss(n=11,sigma=1):
    r = range(-int(n/2),int(n/2)+1)
    return [1 / (sigma * sqrt(2*pi)) * exp(-float(x)**2/(2*sigma**2)) for x in r]


ylabel = 'Bandwidth (bytes/cycle)'
keys = []
if len(sys.argv) > 2:
    if sys.argv[2] == 'read':
        keys += ['L1d$ Read', 'L2$ Read', 'DRAM Read', 'Scene Read']
    elif sys.argv[2] == 'dram':
        keys += ['DRAM Total', 'DRAM Write', 'DRAM Read']
    elif sys.argv[2] == 'streams':
        keys += ['Scene Fill', 'Ray Write', 'Ray Read']
else:
    keys +=  ['L2$ Hit Rate']

colors = list(mcolors.TABLEAU_COLORS)
cycles = []
data = []
for key in keys:
    data.append([])

log_file_name = sys.argv[1]
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

kernel_size = 31
kernel = np.array(gauss(kernel_size, 5))
kernel = kernel / kernel.sum()

for i in range(len(keys)):
    if len(data[i]) == len(cycles):
        plt.plot(cycles, np.convolve(data[i], kernel, mode='same'), colors[i], label=keys[i])

plt.legend(loc="upper right")
plt.ylabel(ylabel)
plt.xlabel('Time (cycles)')
#plt.yscale('log')
plt.grid()
plt.show()