#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO
# from vision_class import DistanceCalculator
import rclpy
from rclpy.node import Node

class DistanceCalculator:
    def __init__(self, focal_length, object_height):
        self.focal_length = focal_length  
        self.object_height = object_height  # Actual height of the object in meters

    def calculate_distance(self, object_height_in_image):
        distance = (self.focal_length * self.object_height) / object_height_in_image
        return distance

    def process_detections(self, results):
       
        distances = []
        for bbox in results:
            xmin, ymin, xmax, ymax = bbox
            object_height_in_image = ymax - ymin
            distance = self.calculate_distance(object_height_in_image)
            distances.append(distance)
        return distances


class Vision(Node):

    def __init__(self):
        super().__init__('planing_node')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 'distance', 10)
        timer_period = 2  # 0.5 hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.vision = DistanceCalculator(1,1)
        # load an official model
        self.model = YOLO("src/vision/vision/best_yolov8.pt")

    def distance(self, results):
        class_names = ['purple', 'red']
        detections = []
        for bbox in results:
            boxes = bbox.boxes
            boxes = bbox.boxes
            cls = boxes.cls.tolist()
            xyxy = boxes.xyxy
            conf = boxes.conf
            masks = bbox.masks
            for i, class_index in enumerate(cls):
                class_name = class_names[int(class_index)]
                confidence = conf[i]
                x, y, w, h = map(int, xyxy[i])
                if class_name == 'red':
                    detections.append([x, y, w, h])

        distances = []
        # detections = results.xyxy[0].cpu().numpy()
        for bbox in results:
            xmin, ymin, xmax, ymax = bbox
            bbox = [xmin, ymin, xmax, ymax]
            object_height_in_image = ymax - ymin
            distance = self.vision.calculate_distance(object_height_in_image)
            distances.append(distance)

        return distances

    def timer_callback(self):
        # input from camera
        results = self.model("src/vision/vision/AI/frame_0075.jpg")
        distances = self.distance(results)

        msg = Float32MultiArray()
        msg.data = distances
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    vision = Vision()
    rclpy.spin(vision)

    vision.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
