import sys
sys.path.append("/home/godang/BoutToHackNASA/source/godang_ws/src/vision/vision")

from std_msgs.msg import Float32MultiArray
from ultralytics import YOLOv10
import rclpy
from rclpy.node import Node
import numpy as np
import cv2


class_names = ['purple','red']

camera_matrix = np.array([[1029.138061543091, 0, 1013.24017],
                          [0, 992.6178560916601, 548.550898],
                          [0, 0, 1]])

dist_coeffs = np.array([ 0.19576996 ,-0.24765409, -0.00625207 , 0.0039396 ,  0.10282869])

new_camera_matrix = np.array([[1.08832011e+03, 0.00000000e+00 ,1.02215651e+03],
                                [0.00000000e+00,1.05041880e+03 ,5.39881529e+02],
                                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
roi = [0, 0, 1919, 1079]

focal_length_x = 1029.138061543091  
focal_length_y = 992.6178560916601 
real_diameter = 0.19
model = YOLOv10("src/vision/vision/bestv10_redball.pt")



def detect_objects(frame):
    list_of_ball = []
    results = model(frame, conf=0.2)
    
    class_names = model.names  
    
    for bbox in results:
        boxes = bbox.boxes
        cls = boxes.cls.tolist()
        xyxy = boxes.xyxy.tolist()
        conf = boxes.conf.tolist()
        
        for i, class_index in enumerate(cls):
            class_name = class_names[int(class_index)]
            x1, y1, x2, y2 = map(int, xyxy[i])
            detection = [x1, y1, x2, y2]
            confidence = float(conf[i])
            print(f"class_name{class_name}")
            list_of_ball.append([detection, confidence, class_name])
            # print(list_of_ball)
    
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
    if list_of_ball == []:
        return []
    ## check if there is any red balls
    red_ball = False
    for i in range(len(list_of_ball)):
        if list_of_ball[i][2] == 'red':
            red_ball = True    
            break
    
    ## if there is no red ball
    if red_ball == False:
        ball_pos = []
        return ball_pos

    ## first choose most confident ball and match radio
    # print(list_of_ball)
    if list_of_ball != []:
        sorted_conf_ball = sorted(list_of_ball, key=lambda x: x[1], reverse=True)
        for i in range(len(sorted_conf_ball)):
            diff_x = sorted_conf_ball[i][0][2] - sorted_conf_ball[i][0][0]
            diff_y = sorted_conf_ball[i][0][3] - sorted_conf_ball[i][0][1]
            if sorted_conf_ball[i][2] == 'red':
                ## compute the center of the ball
                x1, y1, x2, y2 = sorted_conf_ball[i][0]
                u = x1 + (x2 - x1) / 2
                v = y1 + (y2 - y1) / 2

                ## Get depth
                depth_x = (real_diameter * focal_length_x) / (x2-x1)
                

                ## compute the image_to_robot_coordinates
                X, Y, Z = image_to_robot_coordinates(u, v, depth_x)
                ball_pos = [X, Y, Z]

                return ball_pos
        else:
            ball_pos = []
    return ball_pos
        
def R2WConversion(ball_pos,robot_position_in_world_position):
    x_r, y_r, theta_r = robot_position_in_world_position
    theta_r = np.deg2rad(theta_r)
    transformation_matrix = np.array([[np.cos(theta_r), -np.sin(theta_r), x_r],
                                        [np.sin(theta_r), np.cos(theta_r), y_r],
                                        [0, 0, 1]])
        
    X = ball_pos[0]
    Y = ball_pos[1]
    Z = ball_pos[2]
    robot_coords_homogeneous = np.array([Z, -X, 1])
    world_coords_homogeneous = np.dot(transformation_matrix, robot_coords_homogeneous)
    theta_w = np.rad2deg((theta_r))
    return world_coords_homogeneous[0], world_coords_homogeneous[1], theta_w



def UndistortImg(img):
    camera_matrix = np.array([[1029.138061543091, 0, 1013.24017],
                            [0, 992.6178560916601, 548.550898],
                            [0, 0, 1]])
    dist_coeffs = np.array([ 0.19576996 ,-0.24765409, -0.00625207 , 0.0039396 ,  0.10282869])
    new_camera_matrix = np.array([[1.08832011e+03, 0.00000000e+00 ,1.02215651e+03],
        [0.00000000e+00,1.05041880e+03 ,5.39881529e+02],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    roi = [0, 0, 1919, 1079]  

    dst = cv2.undistort(img, camera_matrix, dist_coeffs,None, new_camera_matrix)
    x, y, w, h = roi
    frame_undistorted = dst[y:y+h, x:x+w]
    return frame_undistorted

class VisionBallNode(Node):
    def __init__(self):
        super().__init__('vision_ball_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'ball_data', 10)
        timer_period = 2  # 0.5 hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.vision = DistanceCalculator(1,1)
        # load an official model        
        self.robot_position_in_world_position = [0,0,90]
        self.cap_ball = cv2.VideoCapture(2)
        self.cap_ball.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap_ball.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    def timer_callback(self):
        ret, frame = self.cap_ball.read()
        
        # input from camera
        # call UndistortImg then pass it to model
        # frame = cv2.imread("./src/vision/vision/frame_0225.jpg")
        # results = self.model("src/vision/vision/frame_0127.jpg")
        msg = Float32MultiArray()
        img_undistorted =UndistortImg(frame)
        balls = detect_objects(img_undistorted)
        BallPosRobot = computeBallPosRobotframe(balls)
        if BallPosRobot != []:
            world_Conversion = R2WConversion(BallPosRobot,self.robot_position_in_world_position)
            print(f"world_Conversion {world_Conversion}")
            if world_Conversion:
                msg.data = world_Conversion
        else:
            msg.data = [9.99,9.99,9.99]
            
        print(msg.data)
            
        self.publisher_.publish(msg)
            


def main(args=None):
    rclpy.init(args=args)
    vision = VisionBallNode()
    rclpy.spin(vision)

    vision.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()