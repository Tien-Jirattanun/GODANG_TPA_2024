import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import time
from config import ColorConfig

def calculate_distance(center1, center2):
    distance = np.sqrt((center1[0] - center2[0])**2 + (center1[1] - center2[1])**2)
    return distance

def color_detect(frame,i):
    red_lower = ColorConfig.RED_LOWER
    red_upper = ColorConfig.RED_UPPER
    
    blue_lower = ColorConfig.BLUE_LOWER
    blue_upper = ColorConfig.BLUE_UPPER

    purple_lower = ColorConfig.PURPLE_LOWER
    purple_upper =  ColorConfig.PURPLE_UPPER

    # dist_to_blue = np.linalg.norm(bgr - (blue_lower + blue_upper) / 2)
    # dist_to_purple = np.linalg.norm(bgr - (purple_lower + purple_upper) / 2)
    x, y, r = i
    x = int(x)
    y = int(y)
    r = int(r)
    # print("x, y, r : ", x,y,r)
    y1 = y-r
    y2 = y+r
    x1 = x-r
    x2 = x+r
    if (y - r < 0):
        y1 = 0
    if (y + r > frame.shape[0]):
        y2 = frame.shape[0] - 1 
    if (x - r < 0):
        x1 = 0
    if (x + r > frame.shape[1]):
        x2 = frame.shape[1] - 1 
    frames = cv2.cvtColor(frame, cv2.COLOR_RGB2YCrCb)
    roi = frames[y1:y2, x1:x2]
    cv2.rectangle(frame, (x1,y1), (x2,y2), (255,0,0), 2)
    # print("roi size : ", roi.shape)
    # cv2.imshow("roi", cv2.resize(roi, (200,200)))
    circles_area = np.pi * (i[2]) ** 2  
    circles_area = (i[2]*2) ** 2     
    # print("area :", roi.shape)
    # print("circle area :", i[2])

    mask_blue = cv2.inRange(roi, blue_lower, blue_upper)
    blue_area = cv2.countNonZero(mask_blue)
    # mask = cv2.inRange(frames, blue_lower, blue_upper)
    blue_percentage = (blue_area / circles_area) * 100
    # masked_img  = cv2.bitwise_and(frames, frame, mask=mask)
    # cv2.imshow('Before circles', cv2.resize(roi, (200, 200)))
    # cv2.imshow('after circles', cv2.resize(masked_img, (200, 200)))

    mask_purple = cv2.inRange(roi, purple_lower, purple_upper)
    purple_area = cv2.countNonZero(mask_purple)
    purple_percentage = (purple_area / circles_area) * 100

    mask_red = cv2.inRange(roi, red_lower, red_upper)
    red_area = cv2.countNonZero(mask_red)
    red_percentage = (red_area / circles_area) * 100
    print("blue_percentage",blue_percentage)
    print("purple_percentage",purple_percentage)
    print("red_percentage",red_percentage)

    if (blue_percentage > 60) and (blue_percentage > purple_percentage) and (blue_percentage > red_percentage):
        # print("blue_percentage",blue_percentage)
        # print("purple_percentage",purple_percentage)
        # draw_rectangle_blue(frame, x, y, w, h, (0, 0, 255))
        return "Blue"
    #if purple_percentage > blue_percentage:
    elif (red_percentage > 60) and (red_percentage > blue_percentage) and (red_percentage > purple_percentage):
        return "red"
    elif (purple_percentage > 60) and (purple_percentage > blue_percentage) and (purple_percentage > red_percentage):
        return "None"
    else:
        return "None"
    # if  blue_percentage < 10:
    #     print("purple_percentage",purple_percentage)
    #     return "Purple"

def color_detect_red(frame, bgr, i):

    x, y, r = i
    x = int(x)
    y = int(y)
    r = int(r)
    # print("x, y, r : ", x,y,r)
    y1 = y-r
    y2 = y+r
    x1 = x-r
    x2 = x+r
    if (y - r < 0):
        y1 = 0
    if (y + r > frame.shape[0]):
        y2 = frame.shape[0] - 1 
    if (x - r < 0):
        x1 = 0
    if (x + r > frame.shape[1]):
        x2 = frame.shape[1] - 1 
    roi = frame[y1:y2, x1:x2]
    red_lower = ColorConfig.RED_LOWER
    red_upper = ColorConfig.RED_UPPER

    purple_lower = ColorConfig.PURPLE_LOWER
    purple_upper = ColorConfig.PURPLE_UPPER

    dist_to_red = np.linalg.norm(bgr - (red_lower + red_upper) / 2)
    dist_to_purple = np.linalg.norm(bgr - (purple_lower + purple_upper) / 2)

    if dist_to_red < dist_to_purple:
        return "Red"
    else:
        return "Purple"
    
def color_detect_blue(frame,i):
    
    
    blue_lower = ColorConfig.BLUE_LOWER
    blue_upper = ColorConfig.BLUE_UPPER

    purple_lower = ColorConfig.PURPLE_LOWER
    purple_upper =  ColorConfig.PURPLE_UPPER

    # dist_to_blue = np.linalg.norm(bgr - (blue_lower + blue_upper) / 2)
    # dist_to_purple = np.linalg.norm(bgr - (purple_lower + purple_upper) / 2)
    x, y, r = i
    x = int(x)
    y = int(y)
    r = int(r)
    # print("x, y, r : ", x,y,r)
    y1 = y-r
    y2 = y+r
    x1 = x-r
    x2 = x+r
    if (y - r < 0):
        y1 = 0
    if (y + r > frame.shape[0]):
        y2 = frame.shape[0] - 1 
    if (x - r < 0):
        x1 = 0
    if (x + r > frame.shape[1]):
        x2 = frame.shape[1] - 1 
    roi = frame[y1:y2, x1:x2]
    # print("roi size : ", roi.shape)
    # cv2.imshow("roi", cv2.resize(roi, (200,200)))
    circles_area = np.pi * (i[2]) ** 2    
    mask_blue = cv2.inRange(roi, blue_lower, blue_upper)
    blue_area = cv2.countNonZero(mask_blue)
    blue_percentage = (blue_area / circles_area) * 100
    mask_purple = cv2.inRange(roi, purple_lower, purple_upper)
    purple_area = cv2.countNonZero(mask_purple)
    purple_percentage = (purple_area / circles_area) * 100
    
    print("blue_percentage",blue_percentage)

    if blue_percentage > purple_percentage:
        # print("blue_percentage",blue_percentage)
        # print("purple_percentage",purple_percentage)
        # draw_rectangle_blue(frame, x, y, w, h, (0, 0, 255))
    
        return "Blue"
    #if purple_percentage > blue_percentage:
    else:
        return "Purple"
    # if  blue_percentage < 10:
    #     print("purple_percentage",purple_percentage)
    #     return "Purple"
    
def color_detect_purple(frame,i):   
    purple_lower = ColorConfig.PURPLE_LOWER
    purple_upper = ColorConfig.PURPLE_UPPER
    circles_area = np.pi * (i[2]) ** 2
    x, y, r = i
    x = int(x)
    y = int(y)
    r = int(r)
    # print("x, y, r : ", x,y,r)
    y1 = y-r
    y2 = y+r
    x1 = x-r
    x2 = x+r
    if (y - r < 0):
        y1 = 0
    if (y + r > frame.shape[0]):
        y2 = frame.shape[0] - 1 
    if (x - r < 0):
        x1 = 0
    if (x + r > frame.shape[1]):
        x2 = frame.shape[1] - 1 
    roi = frame[y1:y2, x1:x2]
    # roi = frame[i[1]-i[2]:i[1]+i[2], i[0]-i[2]:i[0]+i[2]]
    mask_purple = cv2.inRange(roi, purple_lower, purple_upper)
    purple_area = cv2.countNonZero(mask_purple)
    purple_percentage = (purple_area / circles_area) * 100
    
    if  purple_percentage > 50:
        print("purple_percentage",purple_percentage)
        return "Purple"
    else:
        return "None"

# def draw_rectangle_Red(img, x, y, w, h, color):
#     # cv2.rectangle(frame, (x,y), (x+w,y+h), color, 2)
#     # cx = (x + (x + w)) / 2
#     # cy = (y + (y + h)) / 2
#     bottomLeftCornerOfText = (x, y)
#     position = (x, y)
#     position = (((x + (x + w)) / 2), ((y + (y + h)) / 2))
#     text = "red Ball x, y" + str(position)
#     cv2.putText(frame,"red" +text,bottomLeftCornerOfText,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
#     frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
#     # cv2.circle(frame, (frame.shape[1] //2, frame.shape[0] // 2),2, (255, 255, 255), 1)
#     distance = calculate_distance(position, frame_center)
#     return distance,position
    
def draw_rectangle_blue(img, x, y, w, h, color):
    # cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)

    bottomLeftCornerOfText = (x, y)

    position = (((x + (x + w)) / 2), ((y + (y + h)) / 2))
    position = (x, y)
    text = "Blue Ball x, y" + str(position)
    cv2.putText(img,text,bottomLeftCornerOfText,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
    frame_center = (img.shape[1] // 2, img.shape[0] // 2)
    # cv2.circle(
    # frame, (frame.shape[1] //2, frame.shape[0] // 2),2, (255, 255, 255), 1)
    distance = calculate_distance(position, frame_center)
    return distance,position
    
    # distance_list.append(distance)
    # if distance ==  min(distance_list):
    #     cv2.rectangle(frame, (x,y), (x+w,y+h), color, 2)

def draw_rectangle(img, x, y, w, h, i, color):

    bottomLeftCornerOfText = (x, y)

    position = (((x + (x + w)) / 2), ((y + (y + h)) / 2))
    position = (x, y)
    text = color + " Ball x, y" + str(position) + " " + str(i[2])
    cv2.putText(img,text,bottomLeftCornerOfText,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
    frame_center = (img.shape[1] // 2, img.shape[0] // 2)
    # cv2.circle(
    # frame, (frame.shape[1] //2, frame.shape[0] // 2),2, (255, 255, 255), 1)
    distance = calculate_distance(position, frame_center)
    return distance,position
    
    # distance_list.append(distance)
    # if distance ==  min(distance_list):
    #     cv2.rectangle(frame, (x,y), (x+w,y+h), color, 2)
#class_color

def blur_gripper(img):
    # (1964, 3024, 3)
    # x = 3024, y = 1964
    # cv2.rectangle(frame, (x,y), (x+w,y+h), color, 2)
    x = 775
    y = 1964
    w = 1000
    h = 1000
    draw_rec_griper = cv2.rectangle(img, (x, y), (2500, 1450), (0, 0, 0), -1)
    # draw_rec_griper_2 =cv2.rectangle(img, (700, 400), (1200, 1200), (0, 0, 0), -1)
    # All black in rectangle



def detect_circles(img):
    distance_list = []
    position_list = []
    
    count = 0
    text = ""
    frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
    # bottomLeftCornerOfText = (0, 0)
    
    # frame = cv2.imread(frame) # Read the image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    mask = np.zeros_like(gray)

    # gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    gray_blurred = cv2.GaussianBlur(h, (9, 9), 2)
    thresh,gray_bin = cv2.threshold(gray_blurred,100,130,cv2.THRESH_BINARY)

    # t_lower = 70 # Lower Threshold 
    # t_upper = 80 # Upper threshold 
    # aperture_size = 3 # Aperture size 
    # L2Gradient = True # Boolean 
    # canny = cv2.Canny(   s, t_lower, t_upper, 
    #                             apertureSize = aperture_size,  
    #                             L2gradient = L2Gradient )

    showbeforecircle = gray_blurred.copy()
    showbeforecircle = cv2.resize(showbeforecircle, (400, 400))

    # circle (x, y, r)
    circles = cv2.HoughCircles(
        h,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=100,
        param1=25,
        param2=30,
        minRadius=270, # 50
        maxRadius=300 # 100
    )
    print("circle : ", circles)
    cv2.imshow('Before circles', cv2.resize(h, (400, 400)))
    if circles is not None:
        circles = np.uint16(np.around(circles))
        mask = np.zeros_like(h)
        #result = cv2.bitwise_and(frame, frame, mask=mask)
        # distance_list = []
        # position_list = []
        for i in circles[0, :]:
            
            #if 0 <= i[0] < frame.shape[1] and 0 <= i[1] < frame.shape[0]:
            # center_pixel_color = frame[i[1], i[0]]
            # class_color_blue = color_detect_blue(frame,i)# color_detect_blue(center_pixel_color)
            # class_color_red = color_detect_red(frame,center_pixel_color, i)
            # class_color_purple = color_detect_purple(frame,i)
            class_color = color_detect(frame,i)
            # print("RGB values at center of circle:", center_pixel_color, "Classified as:", class_color)

            if class_color != "None" :
                # count += 1
                cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(mask, (i[0], i[1]), i[2], 255, thickness=cv2.FILLED)
                cv2.imshow('mask', cv2.resize(mask, (400, 400)))
            
                # distance_list = []
                # position_list = []
                distance, position = draw_rectangle(frame, i[0], i[1], i[2], i[2], i, class_color)        
                # print(count)
                # for i in range(0, count):
                if distance not in distance_list:
                    
                    distance_list.append(distance)
                    position_list.append(position)
                
            # if class_color_red == "Red":
            #     cv2.putText(frame, "Red", (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            # if class_color_blue == "Purple":
            #     cv2.putText(frame, "Purple", (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            # if class_color_purple == "Purple":
                # cv2.putText(frame, "Purple", (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            else:
                pass
        if distance_list != []:
            print("distance list : ", distance_list)
            print("position list : ", position_list)
            # if distance ==  min(distance_list):
            
            index = distance_list.index(min(distance_list))
            position = position_list[index]
            pos = (int(position[0]) + 20, int(position[1]) + 20)
                #for i in position:
                #    pos.append(int(i))
            print("nearest --","distance",distance),print("position",position)
            print(f"pos : {pos}")
            cv2.putText(frame,"near",pos,cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),5)
            cv2.line(frame, pos, frame_center, (0, 0, 0), 5)
                
        cv2.putText(frame, f"total ={count}", (300, 300), cv2.FONT_HERSHEY_SIMPLEX, 10, (255, 255, 255), 5)
                
                
                # distance_list = []
                # position_list = []
                # distance, position = draw_rectangle_blue(frame, i[0], i[1], i[2], i[2], (0, 0, 255))

                
              

                # # print(count)
                # for i in range(0, count):
                #     if distance not in distance_list:
                        
                #         distance_list.append(distance)
                #         position_list.append(position)
                
                # if distance_list!= []:
                #         if distance ==  min(distance_list):
                            
                #             index=distance_list.index(distance)
                #             position = position_list[index]
                #             pos = (int(position[0]), int(position[1]))
                #                 #for i in position:
                #                 #    pos.append(int(i))
                #             print("nearest --","distance",distance),print("position",position)
                #             print(f"pos : {pos}")
                #             cv2.putText(frame,"near",pos,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
                #print(# The `distance` variable is used to calculate the distance between the center
                # of a detected circle and the center of the frame. It is calculated using the
                # `calculate_distance` function, which takes in the coordinates of the two
                # points and uses the Euclidean distance formula to calculate the distance. The
                # distance is then used to determine the closest circle to the center of the
                # frame.
                # distance_list)
                #print(position_list)
    
                        
                
            # red_intensity = center_pixel_color[2]
                
            # if red_intensity > 200:
            #     max_red_intensity = red_intensity
            #     try:
            #         max_red_intensity_location = (i[0], i[1])
            #     except:
            #         pass
                # center_max = (i[0], i[1])
                # radius_max= i[2]
                # x_max, y_max = center_max[0] - radius_max, center_max[1] - radius_max
                # w_max, h_max = 2 * radius_max, 2 * radius_max
                # draw_rectangle(frame, x_max, y_max, w_max, h_max, (255, 255, 255))
                
                
                        
       # result = cv2.bitwise_and(frame, frame, mask=mask)
    

    # cv2.putText(frame, f"total ={count}", (300, 300), cv2.FONT_HERSHEY_SIMPLEX, 10, (255, 255, 255), 5)
    # print("Maximum red intensity:", max_red_intensity)
    # print("Coordinates of the maximum red intensity pixel:", max_red_intensity_location)

    
    # cv2.imshow('Result with Circles Masked', result)
    
    # cv2.imshow('detected circles', gray_blurred)
    # cv2.imshow('detected circles', mask)

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return mask, count, text



# detect_circles('20231010_161302.jpg')
cap = cv2.VideoCapture("D:\\download\highred.mov")    
cap = cv2.VideoCapture("D:\\download\highblue.mov")    
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_EXPOSURE, 2) 
while True:
    ret, frame = cap.read() 
    blur_gripper(frame)
    if not ret:
        break
    # center of frame
    frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
    cv2.circle(frame, (frame.shape[1] //2, frame.shape[0] // 2),2, (255, 255, 255), 1)

        
    # detect_circles(frame)
    mask, count, text = detect_circles(frame)
    # blur_gripper(frame)

    cv2.putText(frame, f"total ={count}", (300, 300), cv2.FONT_HERSHEY_SIMPLEX, 10, (255, 255, 255), 5)
    showmask = mask.copy()
    showframe = frame.copy()

    showmask = cv2.resize(showmask, (400, 400))
    showframe = cv2.resize(showframe, (800, 800))
    
    # cv2.putText(frame,"red" +text,bottomLeftCornerOfText,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
    #cv2.imshow('frame circles', showframe)
    # cv2.imshow('detected circles', showmask)
    cv2.imshow('full frame circles', frame)


    # cv2.imshow('Circle Detection', result_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(frame.shape)
        break

cap.release()
cv2.destroyAllWindows()
