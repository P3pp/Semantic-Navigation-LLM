# Configuration Files Overview – `config/`

This folder contains dynamic JSON files that store the robot's evolving knowledge about its environment and executable commands.



## dynamic_destinations.json`

This file stores **symbolic positions** of detected objects or locations in the environment.

## Format

```json
{
  "object_name": [x, y, yaw],
  "chair": [0.77, -3.10, -14.0],
  ...
}


object_name: symbolic name used in prompts and APIs

x, y: global position of the object

yaw: orientation in degrees (robot's heading when detected)


How it's updated
Automatically updated by:

natural_language_nav.py

via the function: save_destination(object_name, [x, y, yaw])

Used for:

Referencing detected objects in future tasks (e.g., "go to the chair")


dynamic_turtlebot4_api.json
This file stores callable action templates associated with symbolic destinations or functions.

{
  "go_to_chair": "# go to chair\n dest = destinations['chair']\n ...",
  "dock": "# dock\n navigator.dock()",
  ...
}
Keys are action names (e.g., go_to_chair)

Values are the Python code blocks executed by the GPT agent

How it's updated
Automatically updated by:

natural_language_nav.py

via the function: save_api_command(name, code)

Used by:

GPT-based agent to re-use commands based on past experience or known object locations

Manual Edit (Advanced)
You can manually add or remove entries using a text editor or Python. However, it's safer to use the provided API functions:

load_destinations(), save_destination(...), remove_destination(...)

load_api_commands(), save_api_command(...), remove_api_command(...)


Example Usage
When the robot finds a mug:

BLIP confirms "yes"

YOLO estimates its position

dynamic_destinations.json is updated with the mug’s coordinates

dynamic_turtlebot4_api.json is updated with a new go_to_mug() command

Future commands like "go to the mug" will now work
