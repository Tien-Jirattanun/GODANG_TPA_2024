import numpy as np
from ultralytics import YOLOv10
import time

import cv2

class BallDetection:
    def __init__(self, model_path, camera_matrix, dist_coeffs, new_camera_matrix, roi):
        self.model = YOLOv10(model_path)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.new_camera_matrix = new_camera_matrix
        self.roi = roi
        self.class_names = ['purple', 'red']
        self.robot_position = [0, 0, 0]

    def read_img_from_camera(self):
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()
        cv2.destroyAllWindows()
        return frame
    
    def test_function(self):
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            frame = self.undistort_image(frame)
            if not ret:
                break
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            if cv2.waitKey(1) & 0xFF == ord('s'):
                detections = self.detect_objects(frame)
                if len(detections) == 0:
                    print("No balls detected")
                    print(detections)

                focal_length_x = 1029.138061543091  
                focal_length_y = 992.6178560916601 
                real_diameter = 0.19
                bounding_box_width = [detection[2] - detection[0] for detection in detections]
                bounding_box_height = [detection[3] - detection[1] for detection in detections]
                print(f"Bounding box height: {bounding_box_height}")
                print(f"Bounding box width: {bounding_box_width}")

                depth_xy = self.calculate_depth(focal_length_x, focal_length_y, real_diameter, bounding_box_width,bounding_box_height)
                print(f"depth_xy {depth_xy}")

                for i in range(len(depth_xy)):
                    print(f"The estimated depth of the ball from the camera (using focal length x) is {depth_xy[i][0]:.2f} meters.")
                    print(f"The estimated depth of the ball from the camera (using focal length y) is {depth_xy[i][1]:.2f} meters.")


                uv = self.coordinates_image(detections)
                depth = self.calculate_min_depth(focal_length_x, real_diameter,bounding_box_width,bounding_box_height)

                for i in range(len(uv)):
                    u, v = uv[i]
                    xyz_robot_coordinates = self.image_to_robot_coordinates(u, v, depth_xy)

                    print(f"xyz_robot_coordinates {len(xyz_robot_coordinates)}")

                    X, Y, Z_x, Z_y = xyz_robot_coordinates[i]
                    print(f"Robot coordinates: X = {X:.2f} m, Y = {Y:.2f} m, Z_x = {Z_x:.2f} m, Z_y = {Z_y:.2f} m")

                    robot_to_world_coordinates = self.robot_to_world_coordinates(xyz_robot_coordinates[i])
                    print(f"robot_to_world_coordinates  X_r = {robot_to_world_coordinates[0]:.2f} m, Y_r = {robot_to_world_coordinates[1]:.2f} m, Z_x_r = {robot_to_world_coordinates[2]:.2f} m, Z_y_r = {robot_to_world_coordinates[3]:.2f} m")

                


            

    def undistort_image(self, img):
        dst = cv2.undistort(img, self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix) 
        x, y, w, h = self.roi
        frame_undistorted = dst[y:y+h, x:x+w]
 
        return frame_undistorted

    def detect_objects(self, frame):
        results = self.model(frame, conf=0.1)

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

    def calculate_min_depth(self, focal_length, real_diameter, bounding_box_width, bounding_box_height):
        if bounding_box_width == 0 or bounding_box_height ==0:
            raise ValueError("Bounding box width cannot be zero")
        min_depth = (real_diameter * focal_length) / max(bounding_box_width)
        return min_depth
    

    #requirements
    def calculate_depth(self, focal_length_x, focal_length_y, real_diameter, bounding_box_width,bounding_box_height):
        distance_x =[]
        distance_y =[]
        distance_xy =[]
        if len(bounding_box_width) == 0 or len(bounding_box_height) ==0:
            raise ValueError("Bounding box width cannot be zero")
        for i in range(len(bounding_box_width)):
            if bounding_box_width[i] == 0 or bounding_box_height[i] ==0:
                raise ValueError("Bounding box width cannot be zero")
            depth_x = (real_diameter * focal_length_x) / (bounding_box_width[i])
            distance_x.append(depth_x)
            depth_y = (real_diameter * focal_length_y) / (bounding_box_height[i])
            distance_y.append(depth_y)
            distance_xy.append([depth_x,depth_y])
        return distance_xy
    

    def image_to_robot_coordinates(self, u, v, depth):
        fx = self.new_camera_matrix[0, 0]
        fy = self.new_camera_matrix[1, 1]
        cx = self.new_camera_matrix[0, 2]
        cy = self.new_camera_matrix[1, 2]

        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy
        xyz_robot_coordinates =[]
        z_translation = 0.215

        for i in range(len(depth)):

            X = depth[i][0] * x_norm
            Y = depth[i][1] * y_norm
            Z_x =depth[i][0] + z_translation
            Z_y = depth[i][1] + z_translation
            xyz_robot_coordinates.append([X, Y, Z_x, Z_y])

        return xyz_robot_coordinates
    

    

    def robot_to_world_coordinates(self, xyz_robot_coordinates):
        x_r, y_r, theta_r = self.robot_position
       
        robot_point = np.array([xyz_robot_coordinates[2] + x_r, xyz_robot_coordinates[0] + y_r])
        world_point_2d = np.append(robot_point,theta_r)
        
        return world_point_2d
    

    def coordinates_image(self, detections):
        uv = []
        for detection in detections:
            x1, y1, x2, y2 = detection
            u = x1 + (x2 - x1) / 2
            v = y1 + (y2 - y1) / 2
            uv.append([u, v])

        return uv
    
    def error(self, detections):
        x_error_ball =[]
        uv = self.coordinates_image(detections)
        fx = self.new_camera_matrix[0, 0]
        cx = self.new_camera_matrix[0, 2]
        for uv in uv:
            u, v = uv   
            x_error = float(((u - cx) / fx) * 10)
            
            x_error_ball.append(int(abs(x_error)))
        return x_error_ball
camera_matrix = np.array([[1029.138061543091, 0, 1013.24017],
                          [0, 992.6178560916601, 548.550898],
                          [0, 0, 1]])
dist_coeffs = np.array([ 0.19576996 ,-0.24765409, -0.00625207 , 0.0039396 ,  0.10282869])
new_camera_matrix = np.array([[1.08832011e+03, 0.00000000e+00 ,1.02215651e+03],
 [0.00000000e+00,1.05041880e+03 ,5.39881529e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
roi = [0, 0, 1919, 1079]

if __name__ == "__main__":

    ball_detector = BallDetection("../ML_ball/bestv10_redball.pt", camera_matrix, dist_coeffs, new_camera_matrix, roi)
    frame = ball_detector.read_img_from_camera()

    frame = ball_detector.undistort_image(frame)

    detections = ball_detector.detect_objects(frame)
    if len(detections) == 0:
        print("No balls detected")

    else:
        print(detections)
        focal_length_x = 1029.138061543091  
        focal_length_y = 992.6178560916601 
        real_diameter = 0.19
        bounding_box_width = [detection[2] - detection[0] for detection in detections]
        bounding_box_height = [detection[3] - detection[1] for detection in detections]

        depth_xy = ball_detector.calculate_depth(focal_length_x, focal_length_y, real_diameter, bounding_box_width,bounding_box_height)
        uv = ball_detector.coordinates_image(detections)
        depth = ball_detector.calculate_min_depth(focal_length_x, real_diameter,bounding_box_width,bounding_box_height)

        for i in range(len(uv)):
            u, v = uv[i]
            xyz_robot_coordinates = ball_detector.image_to_robot_coordinates(u, v, depth_xy)
            X, Y, Z_x, Z_y = xyz_robot_coordinates[i]
            print(f"Robot coordinates: X = {X:.2f} m, Y = {Y:.2f} m, Z_x = {Z_x:.2f} m, Z_y = {Z_y:.2f} m")
            robot_to_world_coordinates = ball_detector.robot_to_world_coordinates(xyz_robot_coordinates[i])
            print(f"robot_to_world_coordinates  X_r = {robot_to_world_coordinates[0]:.2f} m, Y_r = {robot_to_world_coordinates[1]:.2f} m, Theta_r = {robot_to_world_coordinates[2]:.2f} °")
        error = ball_detector.error(detections)
        print(f"Error: {error}")