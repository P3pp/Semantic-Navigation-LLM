import json
import os

DEST_FILE = os.path.expanduser('~/turtlebot4_ws/config/dynamic_destinations.json')

def load_destinations():
    if os.path.exists(DEST_FILE):
        with open(DEST_FILE, 'r') as f:
            return json.load(f)
    return {}

def save_destination(name, coords):
    data = load_destinations()
    data[name] = coords
    with open(DEST_FILE, 'w') as f:
        json.dump(data, f, indent=2)

def remove_destination(name):
    data = load_destinations()
    if name in data:
        del data[name]
        with open(DEST_FILE, 'w') as f:
            json.dump(data, f, indent=2)
