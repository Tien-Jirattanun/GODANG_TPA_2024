import sys
sys.path.append("/home/godang/BoutToHackNASA/source/godang_ws/src/vision/vision")

# vision import 
import numpy as np
import json
from silo_decision import Decision
from config import ConfigColorsilo
from ultralytics import YOLO
import cv2

# ros import
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# vision class
class SiloDetection:
    def __init__(self, model_path):
        self.silo = [["None", "None", "None"], ["None", "None", "None"], ["None", "None", "None"], ["None", "None", "None"], ["None", "None", "None"]] # silo array
        self.state = 0 # state of silo
        self.model = YOLO(model_path) # model
        self.class_names = ['silo'] # class name
        self.idx = -1 # index
        self.shortest_path_list = [4,0,3,1,2] # shortest path list
        self.shortest_path_state = 0 # shortest path state
        self.cap_silo = cv2.VideoCapture(0)
    
    def detect_silo(self, frame): # parameter : frame
        results = self.model(frame, conf=0.1)
        detections = []
        for bbox in results:
            boxes = bbox.boxes
            cls = boxes.cls.tolist()
            xyxy = boxes.xyxy
            conf = boxes.conf
            for i, class_index in enumerate(cls):
                class_name = self.class_names[int(class_index)]
                if class_name == 'silo':
                    x1, y1, x2, y2 = map(int, xyxy[i])
                    detections.append([x1, y1, x2 - x1, y2 - y1])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)    
                    cv2.imshow("frame", frame)
                    cv2.waitKey(500)      
                cv2.imwrite("..\\BoutToHackNASA\\source\\godang_ws\\src\\vision\\vision\\ans.png", frame)
        cv2.destroyAllWindows()
        return detections

    def display_rois(self, bbox, frame, n): # parameter : bounding box, frame, number of bounding box
        # split silo into 3 parts
        x, y, w, h = bbox
        small_height = h // 3 + 25
        rects = [(x, y + (small_height * i) - 75, w, small_height) for i in range(3)]

        # loop every part of silo
        for i, (x, y, w, h) in enumerate(rects):
            mask_ellipse = cv2.ellipse(np.zeros((h, w), dtype=np.uint8), (w//2, h//2), (w//2, h//2), 0, 0, 360, 255, -1)
            roi = frame[y:y+h, x:x+w]
            roi = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)

            mask_red = cv2.inRange(roi, ConfigColorsilo.RED_LOWER,  ConfigColorsilo.RED_UPPER)
            mask_blue = cv2.inRange(roi, ConfigColorsilo.BLUE_LOWER, ConfigColorsilo.BLUE_UPPER)

            red_area = cv2.countNonZero(cv2.bitwise_and(mask_red, mask_red, mask=mask_ellipse))
            blue_area = cv2.countNonZero(cv2.bitwise_and(mask_blue, mask_blue, mask=mask_ellipse))

            ellipse_area = np.pi * (w/2) * (h/2)
            red_percentage = (red_area / ellipse_area) * 100
            blue_percentage = (blue_area / ellipse_area) * 100
            print(f"{n} blue_percentage {blue_percentage}")
            print(f"{n} red_percentage {red_percentage}")

            if red_percentage > 40:
                self.silo[n][i] = "Red"
            elif blue_percentage > 40:
                self.silo[n][i] = "Blue"
            else:
                self.silo[n][i] = "None"
    
    def silo_decision(self, silo, team): # parameter : silo, team(red/blue)
        count = 0
        if team == "red":
            for i in silo:
                if "None" not in i:
                    count += 1
            if count == 5:
                return 10000000 
            if "None" in silo[self.shortest_path_list[self.idx]]:
                self.shortest_path_state = self.shortest_path_list[self.idx]
                return self.shortest_path_state
            else:
                self.idx -= 1
                self.shortest_path_state = self.shortest_path_list[self.idx]
                return self.shortest_path_state
            
        elif team == "blue":
            for i in silo:
                if "None" not in i:
                    count += 1
            if count == 5:
                return 10000000 
            if "None" in silo[self.shortest_path_list[self.idx]]:
                self.shortest_path_state = self.shortest_path_list[self.idx]
                return self.shortest_path_state
            else:
                self.idx -= 1
                self.shortest_path_state = self.shortest_path_list[self.idx]
                return self.shortest_path_state
            
        else:
            return "Wrong input"
     
   
# ROS class
class VisionSiloNode(Node):

    def __init__(self):
        super().__init__('vision_silo_node')
        self.publisher_ = self.create_publisher(Int32, 'silo_data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # vision initia        
        self.silo_detection = SiloDetection("/home/godang/BoutToHackNASA/source/godang_ws/src/vision/vision/best_silo.pt")
        self.cap_silo = cv2.VideoCapture(0)

    def timer_callback(self):
        msg = Int32()
        ret, frame = self.silo_detection.cap_silo.read()
        print(frame.shape)
        bboxs = sorted(self.silo_detection.detect_silo(frame))
        print(f"bboxs{bboxs}")
        [self.silo_detection.display_rois(bbox, frame, i) for i, bbox in enumerate(bboxs)]
        silo = self.silo_detection.silo
        print(self.silo_detection.silo)
        msg.data = self.silo_detection.silo_decision(silo, "blue")
        print(f"state : {msg.data}")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    vision_silo_node = VisionSiloNode()

    rclpy.spin(vision_silo_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vision_silo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()






# if __name__ == "__main__":
#     silo = [[],[],[],[],[]]
#     silo_detector = SiloDetection("..\\BoutToHackNASA\\source\\godang_ws\\src\\vision\\vision\\siloWeight.pt")
#     # ret, frame = silo_detector.cap.read()
#     frame = cv2.imread("..\\BoutToHackNASA\\source\\godang_ws\\src\\vision\\vision\\framekmutt_silo_0292.jpg")
#     frame = cv2.imread("..\\BoutToHackNASA\\source\\godang_ws\\src\\vision\\vision\\Screenshot 2024-06-03 122739.png")
#     print(frame.shape)

#     # silo detection
#     bboxs = sorted(silo_detector.detect_silo(frame))
#     print(bboxs)
    
#     # ball detection
#     [silo_detector.display_rois(bbox, frame, i) for i, bbox in enumerate(bboxs)] 
#     print(silo_detector.silo)

#     # decision making 
#     silo = silo_detector.silo
#     state = silo_detector.silo_decision(silo, "blue")
#     print(f"state : {state}")
