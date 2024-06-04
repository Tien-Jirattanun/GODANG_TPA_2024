from std_msgs.msg import Float32MultiArray
from ultralytics import YOLOv10
import rclpy
from rclpy.node import Node
import numpy as np
import cv2

class_names = ['purple', 'red']

camera_matrix = np.array([[1029.138061543091, 0, 1013.24017],
                          [0, 992.6178560916601, 548.550898],
                          [0, 0, 1]])

dist_coeffs = np.array([ 0.19576996 ,-0.24765409, -0.00625207 , 0.0039396 ,  0.10282869])

new_camera_matrix = np.array([[1.08832011e+03, 0.00000000e+00 ,1.02215651e+03],
                                [0.00000000e+00,1.05041880e+03 ,5.39881529e+02],
                                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
roi = [0, 0, 1919, 1079]

def detect_objects(frame):
    model = YOLOv10("../../../../../models/bestv10_redball.pt")
    results = model(frame)
    detections = []
    for bbox in results:
        boxes = bbox.boxes
        cls = boxes.cls.tolist()
        xyxy = boxes.xyxy
        conf = boxes.conf
        for i, class_index in enumerate(cls):
            class_name = class_names[int(class_index)]
            x1, y1, x2, y2 = map(int, xyxy[i])
            detections.append([x1, y1, x2, y2])
    list_of_ball = [detections, conf, class_name]
    return list_of_ball

def image_to_robot_coordinates(u, v, depth):
    fx = new_camera_matrix[0, 0]
    fy = new_camera_matrix[1, 1]
    cx = new_camera_matrix[0, 2]
    cy = new_camera_matrix[1, 2]

    x_norm = (u - cx) / fx
    y_norm = (v - cy) / fy
    depth_T = depth + 0.215
    xyz_robot_coordinates = [x_norm, y_norm, depth_T]
    return xyz_robot_coordinates
    

def computeBallPosRobotframe(list_of_ball):
    ## radio of the ball
    radio_threshold = 1.2
    ## check if there are any balls
    if len(list_of_ball) == 0:
        return None
    ## check if there is any red balls
    red_ball = False
    for i in range(len(list_of_ball)):
        if list_of_ball[i][2] == 'red':
            red_ball = True    
            break
    
    ## if there is no red ball
    if red_ball == False:
        return 999

    ## first choose most confident ball and match radio
    sorted_conf_ball = sorted(list_of_ball, key=lambda x: x[1], reverse=True)
    for i in range(len(sorted_conf_ball)):
        diff_x = sorted_conf_ball[i][0][2] - sorted_conf_ball[i][0][0]
        diff_y = sorted_conf_ball[i][0][3] - sorted_conf_ball[i][0][1]
        if sorted_conf_ball[i][2] == 'red' and diff_x/diff_y < radio_threshold:
            ## compute the center of the ball
            x1, y1, x2, y2 = sorted_conf_ball[i][0]
            u = x1 + (x2 - x1) / 2
            v = y1 + (y2 - y1) / 2

            ## compute the image_to_robot_coordinates
            X, Y, Z = image_to_robot_coordinates(u, v)
            ball_pos = [X, Y, Z]

            return ball_pos
        
def R2WConversion(ball_pos,robot_position_in_world_position):
    x_r, y_r, theta_r = robot_position_in_world_position
    theta_r = np.deg2rad(theta_r)
    transformation_matrix = np.array([[np.cos(theta_r), -np.sin(theta_r), x_r],
                                        [np.sin(theta_r), np.cos(theta_r), y_r],
                                        [0, 0, 1]])
    
    X, Y, Z = ball_pos
    robot_coords_homogeneous = np.array([Z, -X, 1])
    world_coords_homogeneous = np.dot(transformation_matrix, robot_coords_homogeneous)
    theta_w = np.rad2deg((theta_r))
    return world_coords_homogeneous[0], world_coords_homogeneous[1], theta_w

    



            
        


    


    
        
       
        


    


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
