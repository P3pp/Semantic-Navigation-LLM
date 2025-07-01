#  Script Overview – turtlebot4_openai_tutorials

This document describes the purpose and functionality of each Python script in the `turtlebot4_openai_tutorials` module.

---

##  `natural_language_nav.py`

**Main GPT-based controller node.**

- Listens for user instructions on `/user_input`
- Sends prompts to the OpenAI API (e.g., GPT-3.5 Turbo)
- Generates Python code and executes it securely (AST-based fallback)
- Triggers navigation via the TurtleBot4Navigator
- Captures RGB and depth images upon goal arrival
- Launches BLIP and YOLOv8 for visual task validation
- Updates symbolic memory with object positions and callable actions

 **Launch with:**  
```bash
ros2 launch turtlebot4_openai_tutorials natural_language_nav_launch.py openai_api_key:=your_api_key



check_depth_proximity_yolo.py
Performs YOLOv8 object detection + depth estimation.

Loads the latest RGB image and depth map

Detects objects using YOLOv8 (anchor-free)

Computes mean depth from the bounding box center

If object is closer than 1.5m, transforms its position to global map frame

Saves debug image with bounding boxes

 Used internally by natural_language_nav.py
 Requires: YOLOv8 weights (yolov8n.pt) in working directory



vision_agent_vqa.py
Answers natural language questions using BLIP-VQA.

Loads captured_image.png

Uses Salesforce/blip-vqa-base model

Answers questions like “Is there a chair?” with “yes” / “no” / object description

 Called by natural_language_nav.py after navigation


dynamic_destinations.py
Manages symbolic locations of detected objects.

Functions:

load_destinations()

save_destination(name, [x, y, yaw])

remove_destination(name)

JSON file:
~/turtlebot4_ws/config/dynamic_destinations.json

Used to store spatial memory of verified objects



dynamic_turtlebot4_api.py
Manages callable API commands linked to symbolic destinations.

Functions:

load_api_commands()

save_api_command(name, code)

remove_api_command(name)

JSON file:
~/turtlebot4_ws/config/dynamic_turtlebot4_api.json

 Enables the agent to use reusable commands (e.g., go_to_bathroom())
