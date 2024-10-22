import json
import hashlib
import os
import sys, re
from math import pi, sqrt, exp

# return a hashed string given a dict
def HashFilename(config: dict) -> str:
    json_string = json.dumps(config, sort_keys=True)

    # Hash the JSON string
    hash_object = hashlib.sha256(json_string.encode())
    hash_string = hash_object.hexdigest()
    return hash_string

# Here we extract the scaler data (e.g., Mrays/s, Power, etc.) using regular expressions
def ExtractLogInfo(log_path: str, data_name: str):
    pattern = r"{}:\s*(\d+)".format(data_name)
    find = False
    value = 0
    with open(log_path) as f:
        lines = f.readlines()  # List containing lines of file
        for line in lines:
            match = re.search(pattern, line, re.IGNORECASE)
            if match:
                find = True
                value = float(match.group(1))
    if not find:
        raise Exception("Data {} does not exist in log {}.".format(data_name, log_path))
    return value
        
def GetBandwidth(log_path: str, unit_name: str) -> list:
    cycles = []
    total_bandwidth_timeline = []
    total_bandwidth_average = 0
    with open(log_path) as f:
        all_pairs_timeline = {} # value is a list
        all_pairs_average = {} # value is a float

        lines = f.readlines()  # List containing lines of file
        # Find all pairs at the first iteration
        header_regex = r'-+{}-+'.format(unit_name)
        detail_pattern = r'Unit name:\s*(.*?),\s*Request label:\s*(.*?),\s*Bandwidth Utilization:\s*([\d.]+)\s*bytes/cycle'
        under_label = False
        under_detailed = False
        length = 0
        tot = 0
        for line in lines:
            # print(line)
            if 'Cycle:' in line:
                cycles.append(int(re.findall(r'[\d]+', line)[0]))
            if re.search(header_regex, line):
                under_label = True
                length += 1
            if under_label and line == "Bandwidth Utilization Starts:\n":
                under_detailed = True
            match = re.search(detail_pattern, line)
            if under_detailed and match:
                unit_name = match.group(1)
                request_label = match.group(2)
                bandwidth = float(match.group(3))
                tot += bandwidth
                all_pairs_timeline[(unit_name, request_label)] = []

            if under_label and under_detailed and line == "Bandwidth Utilization Ends.\n":
                under_label = False
                under_detailed = False
                
                if len(total_bandwidth_timeline) < len(cycles):
                    total_bandwidth_timeline.append(tot)
                else:
                    total_bandwidth_average = tot
                tot = 0

        # Fill out the lists
        under_label = False
        under_detailed = False
        t = -1
        for line in lines:
            if re.search(header_regex, line):
                under_label = True
                t += 1
                if t < len(cycles):
                    for key in all_pairs_timeline:
                        all_pairs_timeline[key].append(float(0))
            if under_label and line == "Bandwidth Utilization Starts:\n":
                under_detailed = True

            match = re.search(detail_pattern, line)
            if under_detailed and match:
                unit_name = match.group(1)
                request_label = match.group(2)
                bandwidth = float(match.group(3))
                if t < len(cycles):
                    all_pairs_timeline[(unit_name, request_label)][t] = bandwidth
                else:
                    all_pairs_average[(unit_name, request_label)] = bandwidth
                
            if under_label and under_detailed and line == "Bandwidth Utilization Ends.\n":
                under_label = False
                under_detailed = False
        return [cycles, total_bandwidth_timeline, all_pairs_timeline, total_bandwidth_average, all_pairs_average]
    raise Exception("File {} doesn't exist.".format(log_path))

def gauss(n=11,sigma=1):
    r = range(-int(n/2),int(n/2)+1)
    return [1 / (sigma * sqrt(2*pi)) * exp(-float(x)**2/(2*sigma**2)) for x in r]