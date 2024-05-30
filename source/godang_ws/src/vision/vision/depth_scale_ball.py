
import numpy as np
from ultralytics import YOLO
import cv2
# Load a model
model = YOLO("../ML_ball/best_yolov8.pt")  # load an official model
# model = YOLO("path/to/best.pt")  # load a custom trained model

# Export the model
# model.export(format="onnx")
frame = cv2.imread("../ML_ball/ball_images/frame_0225.jpg")

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


frame = UndistortImg(frame)
# print(f"frame Original image shape: {frame.shape}")

print(f"dst Original image shape: {frame.shape}")

results = model(frame) 
class_names = ['purple','red']

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
        x1, y1, x2, y2 = map(int, xyxy[i])
        if class_name == 'red':
            detections.append([x1, y1, x2, y2])
#             cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
#             cv2.imshow('frame', frame)
#             cv2.waitKey(500)
# cv2.destroyAllWindows()



print(detections)
def calculate_depth(focal_length, real_diameter, bounding_box_width):
    depth = []
    if bounding_box_width == 0:
        raise ValueError("Bounding box width cannot be zero")
    
    
    min_depth = (real_diameter * focal_length) / max(bounding_box_width)

    return min_depth

focal_length_x = 1029.138061543091  
focal_length_y = 992.6178560916601 
real_diameter = 0.19
bounding_box_width = []
for detection in detections:
    bounding_box_width.append(detection[2]-detection[0])
print(f"Bounding box width: {bounding_box_width}")

depth_x = calculate_depth(focal_length_x, real_diameter, bounding_box_width)
depth_y = calculate_depth(focal_length_y, real_diameter, bounding_box_width)

print(f"The estimated depth of the ball from the camera (using focal length x) is {depth_x:.2f} meters.")
print(f"The estimated depth of the ball from the camera (using focal length y) is {depth_y:.2f} meters.")




def image_to_world_coordinates(u, v, depth, camera_matrix):
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    
    x_norm = (u - cx) / fx
    y_norm = (v - cy) / fy
    
    X = depth * x_norm
    Y = depth * y_norm
    Z = depth
    
    return X, Y, Z

def coordinates_image(detections):
    for detection in detections:
        x1, y1, x2, y2 = detection
        u = x1 + (x2 - x1) / 2
        v = y1 + (y2 - y1) / 2
        return u, v
camera_matrix = np.array([[1029.138061543091, 0,  1.01324017e+03],  
                          [0, 992.6178560916601,  5.48550898e+02],  
                          [0, 0, 1]])




u, v = coordinates_image(detections)
depth = calculate_depth(focal_length_x, real_diameter, bounding_box_width)

X, Y, Z = image_to_world_coordinates(u, v, depth, camera_matrix)
print(f"Real-world coordinates: X = {X:.2f} m, Y = {Y:.2f} m, Z = {Z:.2f} m")
index_nearest = bounding_box_width.index(max(bounding_box_width))
print(f"nearest_Bounding box width: {detections[index_nearest]}")


'''
test Aj code
# def BoxToPos(x,y,w,h):
#    # camera center
#    Cx = 1022.625
#    Cy = 543.287
#    fx = 1074.764
#    fy = 1029.2567
#    u_x = x + 0.5*w
#    u_y = y + 0.5*h
#    approx_x = (u_x-Cx)/fx
#    approx_y = (u_y-Cy)/fy
#    ball_diameter = 0.15 # m NEED TO BE UPDATED
#    # is this Zc or distance??
#    Zc = 0.5*(ball_diameter/w*fx + ball_diameter/h*fy)
#    Zc_2 = np.sqrt(Zc*Zc/(1+approx_x*approx_x+approx_y*approx_y))
#    Xc = approx_x * Zc
#    Yc = approx_y * Zc
#    Xc2 = approx_x * Zc_2
#    Yc2 = approx_y * Zc_2
# #    tx = 0
#    ty = -0.47
#    tz = 0.24
#    print('X~', Xc, ' Z~', -ty - Yc) # should be close to ball radius
#    print('X2~', Xc2, ' Z~', -ty - Yc2) # should be close to ball radius
#    # camera mounting offset
#    return (Xc, Zc+tz)


# print('BoxToPos', BoxToPos(1208, 709, 1289, 798))
'''