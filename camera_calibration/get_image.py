import cv2
import numpy as np

cap = cv2.VideoCapture(0)
count = 0
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
#1080/1920

HEIGHT = 640
SCALED_W = round(1920/1080 * HEIGHT)

START_W = round((SCALED_W - 640)/2)

while True:
    ret, frame = cap.read()
    
    frame =cv2.resize(frame,(1138,640))
    frame = frame[:, START_W:START_W+HEIGHT]

    print(frame.shape)
    if not ret:
        break


    if cv2.waitKey(1) & 0xFF == ord('s'):
        count += 1
        cv2.imwrite(f'./Abuday1/calibration_{count}.jpg', frame)
        print("Image saved")

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cap.destroyAllWindows()


# import numpy as np

# def robot_to_world_coordinates(self, xyz_robot_coordinates):
#     robot_to_world_coordinates = []

#     # Assume the robot's position in the world frame is known
#     x_r = self.robot_position[0]
#     y_r = self.robot_position[1]
#     z_r = self.robot_position[2]
    
#     # Assume the robot's orientation in the world frame is given by a rotation matrix
#     R = self.robot_orientation_matrix

#     # Translation vector
#     T = np.array([x_r, y_r, z_r])

#     for coord in xyz_robot_coordinates:
#         X_r = coord[0]
#         Y_r = coord[1]
#         Z_x_r = coord[2]
#         Z_y_r = coord[3]

#         # Convert to homogeneous coordinates for transformation
#         robot_point_x = np.array([X_r, Y_r, Z_x_r, 1])
#         robot_point_y = np.array([X_r, Y_r, Z_y_r, 1])

#         # Apply the transformation
#         world_point_x = R @ robot_point_x[:3] + T
#         world_point_y = R @ robot_point_y[:3] + T

#         robot_to_world_coordinates.append(world_point_x)
#         robot_to_world_coordinates.append(world_point_y)

#     return robot_to_world_coordinates
