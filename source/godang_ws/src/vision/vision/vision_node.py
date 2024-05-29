from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO
# from vision_class import DistanceCalculator
import rclpy
from rclpy.node import Node
import numpy as np
import cv2

# class DistanceCalculator:
#     def __init__(self, focal_length, object_height):
#         self.focal_length = focal_length  
#         self.object_height = object_height  # Actual height of the object in meters

#     def calculate_distance(self, object_height_in_image):
#         distance = (self.focal_length * self.object_height) / object_height_in_image
#         return distance

#     def process_detections(self, results):
       
#         distances = []
#         for bbox in results:
#             xmin, ymin, xmax, ymax = bbox
#             object_height_in_image = ymax - ymin
#             distance = self.calculate_distance(object_height_in_image)
#             distances.append(distance)
#         return distances
# xmax


def BoxToPos(x,y,w,h):
   # camera center
   Cx = 1022.625
   Cy = 543.287
   fx = 1074.764
   fy = 1029.2567
   u_x = x + 0.5*w
   u_y = y + 0.5*h
   approx_x = (u_x-Cx)/fx
   approx_y = (u_y-Cy)/fy
   ball_diameter = 0.25 # m NEED TO BE UPDATED
   # is this Zc or distance??
   Zc = 0.5*(ball_diameter/w*fx + ball_diameter/h*fy)
   Zc_2 = np.sqrt(Zc*Zc/(1+approx_x*approx_x+approx_y*approx_y))
   Xc = approx_x * Zc
   Yc = approx_y * Zc
   Xc2 = approx_x * Zc_2
   Yc2 = approx_y * Zc_2
#    tx = 0
   ty = -0.47
   tz = 0.24
   print('X~', Xc, ' Z~', -ty - Yc) # should be close to ball radius
   print('X2~', Xc2, ' Z~', -ty - Yc2) # should be close to ball radius
   # camera mounting offset
   return (Xc, Zc+tz)
#    camera_mat = np.ze((3,3))
#    camera_mat[0,0] = fx
#    camera_mat[1,1] = fy
#    camera_mat[2,2] = 1
#    camera_mat[0,2] = Cx
#    camera_mat[1,2] = Cy
#    cam_inv = np.linalg.inv(camera_mat)
#    rot_C2R = np.matrix([1,0,0], [0,1,0], [0,0,1])
#    trans = np.array([tx,ty,tz])
#    uv = np.array([x+0.5*w, y+0.5*h])
#    res = Scaling_factor*cam_inv*uv - trans
#    return (res[0], res[2])

def GetBallPositions(results):
    class_names = ['purple', 'red']
    detections = []
    for bbox in results:
        boxes = bbox.boxes
        cls = boxes.cls.tolist()
        xyxy = boxes.xyxy
        # conf = boxes.conf
        # masks = bbox.masks
        for i, class_index in enumerate(cls):
            class_name = class_names[int(class_index)]
            # confidence = conf[i]
            x, y, w, h = map(int, xyxy[i])
            if class_name == 'red':
                detections.append(BoxToPos(x,y,w,h))
    return detections

def UndistortImg(img):
  # data from calibration
  mtx = np.matrix([[1.02896097e+03, 0, 1.01324017e+03], [0.0, 9.92389966e+02, 5.48550898e+02],[0, 0,  1]])
  dist = np.array([ 0.19576717, -0.2477706,  -0.00620366,  0.00395638,  0.10295289])
  newcameramtx = np.matrix([[1.07476421e+03, 0, 1.02262547e+03], [0, 1.02925677e+03, 5.43286518e+02],[0, 0, 1]])
  roi = [13, 14, 1895, 1057]

  dst = cv2.undistort(img, mtx, dist, None, newcameramtx) 
  # crop the image
  x, y, w, h = roi
  return dst[y:y+h, x:x+w]    

class Vision(Node):
    def __init__(self):
        super().__init__('planing_node')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 'distance', 10)
        timer_period = 2  # 0.5 hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.vision = DistanceCalculator(1,1)
        # load an official model
        self.model = YOLO("src/vision/vision/best.pt")

    def timer_callback(self):
        # input from camera
        # call UndistortImg then pass it to model
        results = self.model("src/vision/vision/frame_0127.jpg")
        msg = Float32MultiArray()
        balls = GetBallPositions(results)
        if balls:
          msg.data = balls
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
