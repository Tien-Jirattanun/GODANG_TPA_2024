import numpy as np
from ultralytics import YOLO
import cv2

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

# Vision Class
class BallDetection:
    def __init__(self, model_path, camera_matrix, dist_coeffs, new_camera_matrix, roi):
        self.model = YOLO(model_path)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.new_camera_matrix = new_camera_matrix
        self.roi = roi
        self.class_names = ['purple', 'red']

    def undistort_image(self, img):
        dst = cv2.undistort(img, self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix) 
        x, y, w, h = self.roi
        return dst[y:y+h, x:x+w]

    def detect_objects(self, frame):
        results = self.model(frame)
        detections = []
        for bbox in results:
            boxes = bbox.boxes
            cls = boxes.cls.tolist()
            xyxy = boxes.xyxy
            conf = boxes.conf
            for i, class_index in enumerate(cls):
                class_name = self.class_names[int(class_index)]
                if class_name == 'red':
                    x1, y1, x2, y2 = map(int, xyxy[i])
                    detections.append([x1, y1, x2, y2])
        return detections

    def calculate_depth(self, focal_length, real_diameter, bounding_box_width):
        if bounding_box_width == 0:
            raise ValueError("Bounding box width cannot be zero")
        min_depth = (real_diameter * focal_length) / max(bounding_box_width)
        return min_depth

    def image_to_world_coordinates(self, u, v, depth):
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy

        X = depth * x_norm
        Y = depth * y_norm
        Z = depth

        return X, Y, Z

    def coordinates_image(self, detections):
        for detection in detections:
            x1, y1, x2, y2 = detection
            u = x1 + (x2 - x1) / 2
            v = y1 + (y2 - y1) / 2
            return u, v


# vision param
camera_matrix = np.array([[1029.138061543091, 0, 1013.24017],
                          [0, 992.6178560916601, 548.550898],
                          [0, 0, 1]])
dist_coeffs = np.array([0.19576717, -0.2477706, -0.00620366, 0.00395638, 0.10295289])
new_camera_matrix = np.array([[1074.76421, 0, 1022.62547],
                              [0, 1029.25677, 543.286518],
                              [0, 0, 1]])
roi = [13, 14, 1895, 1057]

# ROS node
class VisionNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'dept', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # vision constructor
        self.ball_detector = BallDetection("/home/tien/Documents/GitHub/BoutToHackNASA/source/godang_ws/src/vision/vision/best.pt", camera_matrix, dist_coeffs, new_camera_matrix, roi)

    def timer_callback(self):
        msg = Float32MultiArray()
        
        frame = cv2.imread("/home/tien/Documents/GitHub/BoutToHackNASA/source/godang_ws/src/vision/vision/frame_0225.jpg")
        frame = self.ball_detector.undistort_image(frame)

        detections = self.ball_detector.detect_objects(frame)
        # print(detections)

        focal_length_x = 1029.138061543091  
        focal_length_y = 992.6178560916601 
        real_diameter = 0.19
        bounding_box_width = [detection[2] - detection[0] for detection in detections]
        # print(f"Bounding box width: {bounding_box_width}")

        depth_x = self.ball_detector.calculate_depth(focal_length_x, real_diameter, bounding_box_width)
        depth_y = self.ball_detector.calculate_depth(focal_length_y, real_diameter, bounding_box_width)

        # print(f"The estimated depth of the ball from the camera (using focal length x) is {depth_x:.2f} meters.")
        # print(f"The estimated depth of the ball from the camera (using focal length y) is {depth_y:.2f} meters.")

        u, v = self.ball_detector.coordinates_image(detections)
        depth = self.ball_detector.calculate_depth(focal_length_x, real_diameter, bounding_box_width)
        X, Y, Z = self.ball_detector.image_to_world_coordinates(u, v, depth)
        # sent this
        # print(f"Real-world coordinates: X = {X:.2f} m, Y = {Y:.2f} m, Z = {Z:.2f} m")

        index_nearest = bounding_box_width.index(max(bounding_box_width))
        # print(f"Nearest bounding box: {detections[index_nearest]}")
            
        
        msg.data = [X, Y, Z]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    vision_node = VisionNode()

    rclpy.spin(vision_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# if __name__ == "__main__":
#     self.ball_detector = BallDetection("../ML_ball/best_yolov8.pt", camera_matrix, dist_coeffs, new_camera_matrix, roi)

#     frame = cv2.imread("../ML_ball/ball_images/frame_0225.jpg")
#     frame = self.ball_detector.undistort_image(frame)

#     detections = self.ball_detector.detect_objects(frame)
#     print(detections)

#     focal_length_x = 1029.138061543091  
#     focal_length_y = 992.6178560916601 
#     real_diameter = 0.19
#     bounding_box_width = [detection[2] - detection[0] for detection in detections]
#     print(f"Bounding box width: {bounding_box_width}")

#     depth_x = self.ball_detector.calculate_depth(focal_length_x, real_diameter, bounding_box_width)
#     depth_y = self.ball_detector.calculate_depth(focal_length_y, real_diameter, bounding_box_width)

#     print(f"The estimated depth of the ball from the camera (using focal length x) is {depth_x:.2f} meters.")
#     print(f"The estimated depth of the ball from the camera (using focal length y) is {depth_y:.2f} meters.")

#     u, v = self.ball_detector.coordinates_image(detections)
#     depth = self.ball_detector.calculate_depth(focal_length_x, real_diameter, bounding_box_width)
#     X, Y, Z = self.ball_detector.image_to_world_coordinates(u, v, depth)
#     # sent this
#     print(f"Real-world coordinates: X = {X:.2f} m, Y = {Y:.2f} m, Z = {Z:.2f} m")

#     index_nearest = bounding_box_width.index(max(bounding_box_width))
#     print(f"Nearest bounding box: {detections[index_nearest]}")
