# Semantic Navigation and Perception using LLMs for Mobile Robots

This repository contains the **custom modules** added to the official `turtlebot4_ws` ROS 2 workspace to enable **natural-language-driven navigation and visual perception** using GPT, BLIP and YOLOv8.

> This repository **does not include the entire TurtleBot4 workspace**. It includes only the files you need to **extend an existing workspace**.

---

## Features

- Natural Language understanding via GPT (OpenAI API)
- Secure Python code execution via AST parsing
- ROS 2 Nav2 autonomous navigation
- Visual verification:
  - **BLIP** for Visual Question Answering (VQA)
  - **YOLOv8 + Depth** for object localization
- Runtime symbolic updates via `*.json` files

---
📘 See [turtlebot4_openai_tutorials/README_SCRIPTS.md](./turtlebot4_openai_tutorials/README_SCRIPTS.md) for detailed script explanation.
📘 See [config/README_CONFIG.md](./config/README_CONFIG.md) for details on dynamic configuration files.

## Project Structure
Semantic-Navigation-LLM/
├── turtlebot4_openai_tutorials/
│ ├── natural_language_nav.py
│ ├── check_depth_proximity_yolo.py
│ ├── vision_agent_vqa.py
│ ├── dynamic_turtlebot4_api.py
│ └── init.py
├── config/
│ ├── dynamic_destinations.json
│ └── dynamic_turtlebot4_api.json
├── captured_images/
│ ├── captured_image.png
│ └── depth.npy
├── docs/
│ └── architecture_diagram.png
├── requirements.txt
├── run_test_pipeline.py
├── run_all.sh
└── README.md


---

## How to Use

### 1. Download and prepare the official TurtleBot4 ROS 2 workspace

First, download the **official TurtleBot4 workspace** by following the instructions at:

 https://turtlebot.github.io/turtlebot4-user-manual/software/ros2/installation/

Then, make sure your workspace is cloned or installed at:


> If the workspace is not installed, follow the instructions under "Install from Source" in the user manual.

---

### 2. Add the custom modules from this repository

Clone this repository anywhere (e.g., your Desktop), then **copy or move the files** as follows:

#### Replace/Add Python files

Copy the content of the `turtlebot4_openai_tutorials` folder from this repo into:
~/turtlebot4_ws/src/turtlebot4_tutorials/turtlebot4_openai_tutorials/turtlebot4_openai_tutorials/

Overwrite any existing files if prompted.

#### Copy dynamic config files

Copy the `*.json` files from `config/` into your workspace's `config/` folder:
~/turtlebot4_ws/config/

---

### 3. Build and set up environment

```bash
cd ~/turtlebot4_ws
colcon build
source install/setup.bash

# Terminal 1
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  nav2:=true slam:=false localization:=true rviz:=true

# Terminal 2
ros2 launch turtlebot4_openai_tutorials natural_language_nav_launch.py \
  openai_api_key:=your_api_key parking_brake:=true

