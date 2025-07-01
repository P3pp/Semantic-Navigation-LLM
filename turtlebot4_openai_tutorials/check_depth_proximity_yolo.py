
import cv2
import numpy as np
import os
import sys
import math
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

def quaternion_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def estimate_object_global_position(robot_x, robot_y, robot_yaw_rad, bbox_center_x, image_width, distance_m, hfov_deg=80.0):
    angle_offset_deg = (bbox_center_x - (image_width / 2)) * (hfov_deg / image_width)
    angle_offset_rad = math.radians(angle_offset_deg)
    global_angle = robot_yaw_rad + angle_offset_rad
    object_x = robot_x + distance_m * math.cos(global_angle)
    object_y = robot_y + distance_m * math.sin(global_angle)
    return object_x, object_y

# === CONFIG ===
CAPTURE_DIR = os.path.expanduser('~/turtlebot4_ws/captured_images')
RGB_PATH = os.path.join(CAPTURE_DIR, 'captured_image.png')
DEPTH_PATH = os.path.join(CAPTURE_DIR, 'depth_debug.png')
DEPTH_RAW_PATH = os.path.join(CAPTURE_DIR, 'depth.npy')
THRESHOLD_METERS = 1.5

class ObjectPositionEstimator(Node):
    def __init__(self, object_class):
        super().__init__('object_position_estimator')
        self.object_class = object_class.lower()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.run_detection()

    def load_depth_map(self):
        if os.path.exists(DEPTH_RAW_PATH):
            depth = np.load(DEPTH_RAW_PATH)
            depth = np.where(np.isinf(depth), np.nan, depth)
            return depth
        print('[WARN] Mappa di profondità non trovata in .npy, uso immagine normalizzata (approssimativa)')
        img = cv2.imread(DEPTH_PATH, cv2.IMREAD_GRAYSCALE)
        return img.astype(np.float32) / 255.0 * 5.0

    def run_detection(self):
        print('[INFO] Caricamento modello YOLOv8...')
        model = YOLO('yolov8n.pt')
        print('[INFO] Caricamento immagine RGB...')
        image = cv2.imread(RGB_PATH)
        if image is None:
            print('[ERROR] Immagine RGB non trovata.')
            return

        print('[INFO] Rilevamento oggetti...')
        results = model(RGB_PATH)[0]
        depth_map = self.load_depth_map()
        image_width = image.shape[1] if image is not None else 640

        for box in results.boxes.data:
            x1, y1, x2, y2, conf, cls = box.tolist()
            label = model.names[int(cls)]

            if label.lower() == self.object_class:
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                bbox_center_x = int((x1 + x2) / 2)
                bbox_center_y = int((y1 + y2) / 2)
                depth_center = depth_map[bbox_center_y, bbox_center_x]

                print(f'[DEBUG] Depth al centro del box: {depth_center:.3f} m')

                if not np.isfinite(depth_center) or depth_center <= 0:
                    print('[WARN] Profondità non valida al centro del box.')
                    return

                mean_depth = depth_center  # Usa solo il valore centrale
                print(f'[INFO] Rilevato: {label} ({conf:.2f}) in box: {x1},{y1} → {x2},{y2}')
                print(f'[YOLO] bbox_center_x = {bbox_center_x}, image_width = {image_width}')
                print(f'[YOLO] mean_depth = {mean_depth:.3f} m')

                # Salva immagine con bounding box
                debug_img = image.copy()
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                debug_path = os.path.join(CAPTURE_DIR, 'depth_bbox_debug.png')
                cv2.imwrite(debug_path, debug_img)
                print(f'[DEBUG] Immagine bbox salvata in: {debug_path}')

                if mean_depth > THRESHOLD_METERS:
                    print('❌ Oggetto troppo lontano.')
                    return

                try:
                    tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
                    x = tf.transform.translation.x
                    y = tf.transform.translation.y
                    q = tf.transform.rotation
                    yaw = quaternion_to_yaw(q)

                    obj_x, obj_y = estimate_object_global_position(x, y, yaw, bbox_center_x, image_width, mean_depth)
                    print(f'[✅] Oggetto stimato in posizione globale: x = {obj_x:.2f}, y = {obj_y:.2f}')
                except Exception as e:
                    self.get_logger().warn(f"TF lookup failed: {e}")
                return

        print(f"[INFO] Nessun oggetto '{self.object_class}' rilevato nella scena.")

def main(args=None):
    if len(sys.argv) < 2:
        print("[ERROR] Specificare l'oggetto target: es. python3 check_depth_proximity_yolo.py chair")
        return

    rclpy.init()
    node = ObjectPositionEstimator(sys.argv[1])
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
