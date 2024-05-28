import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import time
from config import ColorConfig_cammid, blur_bg

class BallDetector:
    x1 = blur_bg.x1
    y1 = blur_bg.y1
    x2 = blur_bg.x2
    y2 = blur_bg.y2
    def __init__(self, video_source=0):
        self.cap = cv2.VideoCapture(video_source)
        self.cap = cv2.VideoCapture("./video/105CF7B8-0A2C-4E79-BAD1-E8399F5160C3.mp4")     
        # self.cap = cv2.VideoCapture("D:\\download\lowblue.mov") 
        self.l_b = ColorConfig_cammid.BLUE_LOWER
        self.u_b = ColorConfig_cammid.BLUE_UPPER
        self.l_r = ColorConfig_cammid.RED_LOWER
        self.u_r = ColorConfig_cammid.RED_UPPER
        self.lower_purple = ColorConfig_cammid.PURPLE_LOWER
        self.upper_purple = ColorConfig_cammid.PURPLE_UPPER
        self.frame = 0

    def distance_to_center(self, rect, frame_shape):
        rect_center_x = rect[0] + rect[2] // 2  
        frame_center_x = frame_shape[1] // 2 
        distance = abs(frame_center_x - rect_center_x)
        return distance

    def min_distance_to_center(self, rects, frame_shape):
        distances = [self.distance_to_center(rect, frame_shape) for rect in rects]
        print(f"distance {distances}")
        min_distance = min(distances)
        return min_distance

    def number_of_balls(self, rects):
        return len(rects)

    def blur_bg(self, img,x1=x1,x2=x2,y1=y1,y2=y2):
        cv2.rectangle(self.frame, (x1, y1), (x2, y2), (0, 0, 0), -1)

    def print_to(self, rects, frame_shape):
        print(f"distance {[self.distance_to_center(rect, frame_shape) for rect in rects]}")
        distances = [self.distance_to_center(rect, frame_shape) for rect in rects]
        print(f"min_distance {min(distances)}")
        

    def detect_balls(self, color_lower, color_upper, min_size=(100, 100)):


        ycrcb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2YCrCb)
        mask = cv2.inRange(ycrcb, color_lower, color_upper)
        _, mask1 = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
        cnts, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        rects = []

        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            if cv2.contourArea(c) > ColorConfig_cammid.boundingRect and w > min_size[0] and h > min_size[1]:
                rects.append((x, y, w, h))

        return rects

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        self.frame = frame

        self.blur_bg(self.frame)
        blue_rects = self.detect_balls(self.l_b, self.u_b)
        red_rects = self.detect_balls(self.l_r, self.u_r)
        purple_rects = self.detect_balls(self.lower_purple, self.upper_purple)

        for x, y, w, h in blue_rects:
            cv2.rectangle(self.frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

        for x, y, w, h in red_rects:
            cv2.rectangle(self.frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

        for x, y, w, h in purple_rects:
            cv2.rectangle(self.frame, (x, y), (x+w, y+h), (128, 0, 128), 2)
        self.print_to(blue_rects, self.frame.shape)
        self.print_to(red_rects, self.frame.shape)
        self.print_to(purple_rects, self.frame.shape)
        cv2.imshow('frame', self.frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            return False
        return True

    def run(self):
        while self.process_frame():
            pass

if __name__ == "__main__":
    ball_detector = BallDetector()
    ball_detector.run()
    #ball_detector.print_to()