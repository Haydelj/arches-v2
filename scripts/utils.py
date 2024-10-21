import json
import hashlib
import os

# return a hashed string given a dict
def HashFilename(config: dict) -> str:
    json_string = json.dumps(config, sort_keys=True)

    # Hash the JSON string
    hash_object = hashlib.sha256(json_string.encode())
    hash_string = hash_object.hexdigest()
    return hash_string

def AnalyzeLog(log_path: str):
    if not os.path.exists(log_path):
        raise Exception("Exe {} does not exist".format(log_path))
    pass