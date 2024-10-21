import json
import hashlib
import os
import sys, re

# return a hashed string given a dict
def HashFilename(config: dict) -> str:
    json_string = json.dumps(config, sort_keys=True)

    # Hash the JSON string
    hash_object = hashlib.sha256(json_string.encode())
    hash_string = hash_object.hexdigest()
    return hash_string

# Here we extract the scaler data (e.g., Mrays/s, Power, etc.) using regular expression
def ExtractLogInfo(log_path: str, data_name: str):
    if not os.path.exists(log_path):
        raise Exception("Exe {} does not exist".format(log_path))
    pass

def GetBandwidthList(log_path: str, unit_name: str) -> dict:
    cycles = []
    total_bandwidth = []
    with open(log_path) as f:
        all_pairs = {}
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
                # print("Start!")
            match = re.search(detail_pattern, line)
            if under_detailed and match:
                unit_name = match.group(1)
                request_label = match.group(2)
                bandwidth = float(match.group(3))
                tot += bandwidth
                all_pairs[(unit_name, request_label)] = []

            if under_label and under_detailed and line == "Bandwidth Utilization Ends.\n":
                under_label = False
                under_detailed = False
                total_bandwidth.append(tot)
                print(total_bandwidth)
                # print("tot = ", tot)
                tot = 0

        # Fill out the lists
        under_label = False
        under_detailed = False
        t = -1
        for line in lines:
            if re.search(header_regex, line):
                under_label = True
                t += 1
                for key in all_pairs:
                    all_pairs[key].append(float(0))
            if under_label and line == "Bandwidth Utilization Starts:":
                under_detailed = True

            match = re.search(detail_pattern, line)
            if under_detailed and match:
                unit_name = match.group(1)
                request_label = match.group(2)
                bandwidth = float(match.group(3))
                all_pairs[(unit_name, request_label)][t] = bandwidth
                
            if under_label and under_detailed and line == "Bandwidth Utilization Ends.":
                under_label = False
                under_detailed = False
        return cycles, total_bandwidth, all_pairs
    raise Exception("File {} doesn't exist.".format(log_path))