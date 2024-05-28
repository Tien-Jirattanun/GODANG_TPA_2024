import cv2
import numpy as np
import imutils
import json
from silo_decision import Decision
from config import ColorConfig, SiloConfig

class SiloDetection:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.result = [["None", "None", "None"], ["None", "None", "None"], ["None", "None", "None"], ["None", "None", "None"], ["None", "None", "None"]]

    def split_rectangle_into_rois(self, large_rect):
        x, y, width, height = large_rect
        small_height = height // 3
        rects = [(x, y + i * small_height, width, small_height) for i in range(3)]
        return rects

    def color_detect(self, bgr, color_bounds):
        return cv2.inRange(bgr, color_bounds[0], color_bounds[1])

    def display_rois(self, small_ellipses, frame, n):
        for i, (x, y, w, h) in enumerate(small_ellipses):
            mask_ellipse = cv2.ellipse(np.zeros((h, w), dtype=np.uint8), (w//2, h//2), (w//2, h//2), 0, 0, 360, 255, -1)
            roi = frame[y:y+h, x:x+w]

            mask_red = self.color_detect(roi, (ColorConfig.RED_LOWER, ColorConfig.RED_UPPER))
            mask_blue = self.color_detect(roi, (ColorConfig.BLUE_LOWER, ColorConfig.BLUE_UPPER))

            red_area = cv2.countNonZero(cv2.bitwise_and(mask_red, mask_red, mask=mask_ellipse))
            blue_area = cv2.countNonZero(cv2.bitwise_and(mask_blue, mask_blue, mask=mask_ellipse))

            ellipse_area = np.pi * (w/2) * (h/2)
            red_percentage = (red_area / ellipse_area) * 100
            blue_percentage = (blue_area / ellipse_area) * 100

            if red_percentage > 10:
                self.result[n][i] = "Red"
            elif blue_percentage > 20:
                self.result[n][i] = "Blue"
            else:
                self.result[n][i] = "None"

    def ellipse(self, small_rectangles, color):
        for rect in small_rectangles:
            center = (rect[0] + rect[2] // 2, rect[1] + rect[3] // 2 - 12)
            axes_length = (rect[2] // 3, rect[3] // 3)
            cv2.ellipse(self.frame, center, axes_length, 0, 0, 360, color, 2)

    def silo_roi(self, silo_config, color, n):
        x, y, w, h = silo_config
        cv2.rectangle(self.frame, (x, y), (x + w, y + h), color, 2)
        large_rectangle = (x, 400, 150, 230)
        small_rectangles = self.split_rectangle_into_rois(large_rectangle)
        self.ellipse(small_rectangles, (255, 255, 255))
        self.display_rois(small_rectangles, self.frame, n)

    def run(self):
        while True:
            ret, self.frame = self.cap.read()
            self.frame = cv2.flip(self.frame, 0)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print(f"Silo : {self.result[2]}")
                break

            self.silo_roi(SiloConfig.silo_roi_1, (0, 255, 0), 0)
            self.silo_roi(SiloConfig.silo_roi_2, (255, 255, 255), 1)
            self.silo_roi(SiloConfig.silo_roi_3, (255, 0, 255), 2)
            self.silo_roi(SiloConfig.silo_roi_4, (0, 255, 255), 3)
            self.silo_roi(SiloConfig.silo_roi_5, (0, 0, 0), 4)

            cv2.imshow("frame", self.frame)

        self.cap.release()
        cv2.destroyAllWindows()
    
    def savetojson(self):
        write = {"silo_states":{1: self.result}}
        with open("result.json", "w") as f:
            json.dump(write, f)
        with open('result.json', 'r') as f:
            data = json.load(f)
            return data["silo_states"]["1"]

if __name__ == "__main__":
    silo_detector = SiloDetection()
    silo_detector.run()
    silo = silo_detector.savetojson()
    what = Decision(silo)