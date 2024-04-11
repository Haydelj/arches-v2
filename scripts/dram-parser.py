import sys
import re

import matplotlib.pyplot as plt
import numpy as np

cycles = []
dram_total_data = []
dram_read_data = []
dram_write_data = []

log_file_name = sys.argv[1]
print("Parsing: " + log_file_name)
with open(log_file_name) as f:
    lines = f.readlines() #list containing lines of file
    for line in lines:
        if('Cycle:' in line):
            cycles.append(int(re.findall(r'[\d]+', line)[0]))
        if('DRAM Total:' in line):
            dram_total_data.append(float(re.findall(r'[\d]*[.][\d]+', line)[0]))
        if('DRAM Read:' in line):
            dram_read_data.append(float(re.findall(r'[\d]*[.][\d]+', line)[0]))
        if('DRAM Write:' in line):
            dram_write_data.append(float(re.findall(r'[\d]*[.][\d]+', line)[0]))

if(len(dram_total_data)):
    plt.plot(cycles, dram_total_data, "-r", label="total")
if(len(dram_read_data)):
    plt.plot(cycles, dram_read_data, "-g", label="read")
if(len(dram_write_data)):
    plt.plot(cycles, dram_write_data, "-b", label="write")

plt.legend(loc="upper right")
plt.show()