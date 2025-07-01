import os
import json
import subprocess
import rclpy
import ament_index_python
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from tf2_ros import Buffer, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from openai import OpenAI
from turtlebot4_python_tutorials.nav_through_poses import execute_nav_through_poses
from turtlebot4_python_tutorials.mail_delivery import execute_mail_delivery
from turtlebot4_openai_tutorials.dynamic_destinations import save_destination, load_destinations
from turtlebot4_openai_tutorials.dynamic_turtlebot4_api import save_api_command
import re
import ast

DYNAMIC_DESTINATIONS_PATH = os.path.expanduser('~/turtlebot4_ws/config/dynamic_destinations.json')
DYNAMIC_PROMPT_INJECTOR_PATH = os.path.expanduser('~/turtlebot4_ws/config/dynamic_turtlebot4_api.json')

class GPTNode(Node):
    def __init__(self, navigator):
        super().__init__('gpt_node')
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('model_name', 'gpt-3.5-turbo')
        self.declare_parameter('parking_brake', True)

        self.api_key = self.get_parameter('openai_api_key').value
        self.client = OpenAI(api_key=self.api_key)
        self.model_name = self.get_parameter('model_name').value
        self.prompts = []
        self.full_prompt = ""

        self.navigator = navigator
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_input = self.create_subscription(String, 'user_input', self.user_input, 10)
        self.pub_ready = self.create_publisher(Bool, 'ready_for_input', 10)
        self.publish_status(False)

        self.bridge = CvBridge()
        self.vision_requested = False
        self.image_saved = True
        self.target_object = None
        self.latest_depth = None
        self.depth_ready = False
        self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/oakd/rgb/preview/depth', self.depth_callback, 10)

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_ready = True
        except Exception as e:
            self.error(f'Errore durante il salvataggio della depth: {e}')

    def image_callback(self, msg):
        if not self.vision_requested:
            return 
            
        try:
            self.vision_requested = False 
            image_dir = os.path.expanduser('~/turtlebot4_ws/captured_images')
            os.makedirs(image_dir, exist_ok=True)
            image_path = os.path.join(image_dir, 'captured_image.png')
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(image_path, cv_image)
            self.info(f'Image saved at {image_path}')

            if self.latest_depth is not None:
                np.save(os.path.join(image_dir, 'depth.npy'), self.latest_depth)
                self.info(f'Depth image saved at {image_dir}/depth.npy')
            else:
                self.warn('⚠️ Depth non ancora disponibile al momento della cattura RGB')
            self.run_vision_pipeline()
            
        except Exception as e:
            self.error(f'Error saving image: {e}')

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def estimate_object_position(self, robot_x, robot_y, robot_yaw_rad, bbox_center_x, image_width, distance_m, hfov_deg=80.0):
        angle_offset_deg = (bbox_center_x - (image_width / 2)) * (hfov_deg / image_width)
        angle_offset_rad = math.radians(angle_offset_deg)
        global_angle = robot_yaw_rad + angle_offset_rad
        object_x = robot_x + distance_m * math.cos(global_angle)
        object_y = robot_y + distance_m * math.sin(global_angle)
        return object_x, object_y

    def run_vision_pipeline(self):
        if not self.target_object:
            self.warn("[Vision] Nessun oggetto target definito. Skipping.")
            return

        question = f"Is there a {self.target_object} in front of the robot?"

        try:
            self.info("[Vision] Running BLIP-VQA...")
            script_path = os.path.join(os.path.dirname(__file__), 'vision_agent_vqa.py')
            result_vqa = subprocess.check_output(['python3', script_path, question], text=True)
            self.info(result_vqa.strip())

            self.info("[Vision] Checking YOLO depth proximity...")
            yolo_script = os.path.join(os.path.dirname(__file__), 'check_depth_proximity_yolo.py')
            result_depth = subprocess.check_output(['python3', yolo_script, self.target_object], text=True)
            self.info(result_depth.strip())

            match_x = re.search(r'bbox_center_x\s*=\s*(\d+)', result_depth)
            match_width = re.search(r'image_width\s*=\s*(\d+)', result_depth)
            match_depth = re.search(r'mean_depth\s*=\s*([0-9.]+)', result_depth)
            depth_ok = False
            if match_depth:
                depth_val = float(match_depth.group(1))
                depth_ok = depth_val <= 1.5

            if 'yes' in result_vqa.lower() and depth_ok:
                if match_x and match_width and match_depth:
                    bbox_center_x = int(match_x.group(1))
                    image_width = int(match_width.group(1))
                    mean_depth = float(match_depth.group(1))

                    tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
                    robot_x = tf.transform.translation.x
                    robot_y = tf.transform.translation.y
                    yaw = self.quaternion_to_yaw(tf.transform.rotation)

                    obj_x, obj_y = self.estimate_object_position(robot_x, robot_y, yaw, bbox_center_x, image_width, mean_depth)
                    yaw_deg = math.degrees(yaw)
                    save_destination(self.target_object, [obj_x, obj_y, yaw_deg])

                    save_api_command(f"go_to_{self.target_object}", f"dest = destinations['{self.target_object}']\nnavigator.info('Moving to {self.target_object}')\ngoal = navigator.getPoseStamped(dest[0:2], dest[2])\nnavigator.startToPose(goal)")
                    self.info(f"✅ Oggetto '{self.target_object}' rilevato e registrato come destinazione.")
                else:
                    self.warn("[Vision] Informazioni incomplete da YOLO per stimare la posizione dell'oggetto.")
            else:
                self.info(f"❌ Oggetto '{self.target_object}' non rilevato o troppo lontano.")

        except Exception as e:
            self.error(f'Errore nel vision pipeline: {e}')

    def query(self, base_prompt, query, stop_tokens=None, query_kwargs=None, log=True):
        new_prompt = f'{base_prompt}\n{query}'
        use_query_kwargs = {'max_tokens': 512, 'temperature': 0}
        if query_kwargs is not None:
            use_query_kwargs.update(query_kwargs)

        messages = [{"role": "user", "content": new_prompt}]
        response = self.client.chat.completions.create(
            model=self.model_name,
            messages=messages,
            **use_query_kwargs
        ).choices[0].message.content.strip()

        if log:
            self.info(f"GPT Query: {query}")
            self.info(f"GPT Response: {response}")

        return response

    def extract_target_object(self, instruction):
        patterns = [r"check (?:if )?(?:there (?:is|are) )?(?:a |an |the )?([\w\s]+)",
        r"look for (?:a |an |the )?([\w\s]+)",
        r"see (?:if )?(?:there (?:is|are) )?(?:a |an |the )?([\w\s]+)",
        r"is (?:there )?(?:a |an |the )?([\w\s]+)",
        r"(?:there is|there are) (?:a |an |the )?([\w\s]+)",
        r"find (?:a |an |the )?([\w\s]+)"]
        for pattern in patterns:
            match = re.search(pattern, instruction.lower())
            if match:
                return match.group(1).strip()
        return None


    def safe_exec_with_fallback(self, code_str, context):
        try:
            tree = ast.parse(code_str)
            called_funcs = {node.func.id for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
            available_funcs = set(context.keys()) | set(dir(__builtins__))
            undefined_funcs = called_funcs - available_funcs

            if undefined_funcs:
                self.warn(f"⚠️ Funzioni non definite trovate: {undefined_funcs}")
                for func in undefined_funcs:
                    pattern = rf"{func}\\(.*?\\)"
                    code_str = re.sub(pattern, f"# TODO: define function `{func}`", code_str)

            exec(code_str, context)
        except Exception as e:
            self.error(f'[SAFE EXEC] Errore durante esecuzione codice GPT: {e}')

    def publish_status(self, status):
        msg = Bool()
        msg.data = status
        self.ready_for_input = status
        self.pub_ready.publish(msg)

    def user_input(self, msg):
        if not self.ready_for_input:
            self.info(f"Received input <{msg.data}> when not ready, skipping")
            return

        self.publish_status(False)
        self.info(f"Received input <{msg.data}>")
        query = '# ' + msg.data
        self.target_object = self.extract_target_object(msg.data)
        result = self.query(f'{self.full_prompt}', query, ['#', 'objects = ['])

        self.safe_exec_with_fallback(result, {
            "navigator": self.navigator,
            "TurtleBot4Directions": TurtleBot4Directions,
            "destinations": load_destinations(),
            "execute_nav_through_poses": execute_nav_through_poses,
            "execute_mail_delivery": execute_mail_delivery
        })

        self.image_saved = False
        self.vision_requested = self.target_object is not None
        self.info('Task completed, ready to capture image.')
        self.publish_status(True)

    def info(self, msg):
        self.get_logger().info(msg)

    def warn(self, msg):
        self.get_logger().warn(msg)

    def error(self, msg):
        self.get_logger().error(msg)

def read_prompt_file(prompt_file):
    if os.path.exists(prompt_file):
        with open(prompt_file, 'r') as file:
            return json.load(file)
    return {}

def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()
    gpt = GPTNode(navigator)

    api_data = read_prompt_file(DYNAMIC_PROMPT_INJECTOR_PATH)
    for key, code in api_data.items():
        gpt.prompts.append(code)

    for p in gpt.prompts:
        gpt.full_prompt = gpt.full_prompt + '\n' + p

    if not gpt.get_parameter('parking_brake').value:
        gpt.warn("Parking brake not set, robot will execute commands!")
        if not navigator.getDockedStatus():
            navigator.info('Docking before initializing pose')
            navigator.dock()

        initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        navigator.setInitialPose(initial_pose)
        navigator.waitUntilNav2Active()
        navigator.undock()
    else:
        gpt.warn("Parking brake set, robot will not execute commands!")

    gpt.info("Entering input parsing loop with destination map")
    gpt.info(str(load_destinations()))
    gpt.publish_status(True)
    try:
        rclpy.spin(gpt)
    except KeyboardInterrupt:
        pass

    gpt.destroy_node()
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

