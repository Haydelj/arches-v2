import sys
import re

import matplotlib.pyplot as plt
import numpy as np

cycles = []
dram_data = []
l2_data = []
l1_data = []

log_file_name = sys.argv[1]
print("Parsing: " + log_file_name)
with open(log_file_name) as f:
    lines = f.readlines() #list containing lines of file
    for line in lines:
        if('Cycle:' in line):
            cycles.append(int(re.findall(r'[\d]+', line)[0]))
        if('DRAM Read:' in line):
            dram_data.append(float(re.findall(r'[\d]*[.][\d]+', line)[0]))
        if('L2$ Read:' in line):
            l2_data.append(float(re.findall(r'[\d]*[.][\d]+', line)[0]))
        if('L1d$ Read:' in line):
            l1_data.append(float(re.findall(r'[\d]*[.][\d]+', line)[0]))

plt.plot(cycles, dram_data, "-r", label="DRAM")
plt.plot(cycles, l2_data, "-g", label="L2$")
plt.plot(cycles, l1_data, "-b", label="L1d$")

plt.yscale('symlog')
plt.legend(loc="upper right")
plt.show()