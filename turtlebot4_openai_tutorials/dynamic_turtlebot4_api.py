import json
import os

API_FILE = os.path.expanduser('~/turtlebot4_ws/config/dynamic_turtlebot4_api.json')

def load_api_commands():
    if os.path.exists(API_FILE):
        with open(API_FILE, 'r') as f:
            return json.load(f)
    return {}

def save_api_command(name, command_block):
    data = load_api_commands()
    data[name] = f"# {name.replace('_', ' ')}\n{command_block}"
    with open(API_FILE, 'w') as f:
        json.dump(data, f, indent=2)

def remove_api_command(name):
    data = load_api_commands()
    if name in data:
        del data[name]
        with open(API_FILE, 'w') as f:
            json.dump(data, f, indent=2)
