import numpy as np
import json
from silo_decision import Decision
from config import ConfigColorsilo
from ultralytics import YOLOv10
import cv2

class SiloDetection:
    def __init__(self, model_path, camera_matrix, dist_coeffs, new_camera_matrix, roi):
        # self.cap = cv2.VideoCapture(0)
        # self.cap = cv2.VideoCapture("C:\\Users\\User\\Pictures\\Camera Roll\\WIN_20240509_22_44_50_Pro.mp4")
        self.result = [["None", "None", "None"], ["None", "None", "None"], ["None", "None", "None"], ["None", "None", "None"], ["None", "None", "None"]]
        # self.frame = None
        self.state = 0
        self.model = YOLOv10(model_path)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.new_camera_matrix = new_camera_matrix
        self.roi = roi
        self.class_names = ['silo']
    
    def detect_silo(self, frame):
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

    def split_rectangle_into_rois(self, bbox):
        x, y, w, h = bbox
        small_height = h // 3
        rects = [(x, y + (small_height * i), w, small_height) for i in range(3)]
        return rects

    def draw_ellipse(self, small_rectangles, frame):
        for (x, y, w, h) in small_rectangles:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)  
            center = ((x//2) + ((x+w)//2), y+h // 2)
            axes_length = (w // 2, h // 2)
            cv2.circle(frame, center, 5, (0, 255, 0), 1)
            cv2.ellipse(frame, center, axes_length, 0, 0, 360, (255, 255, 255), 1)

    def color_detect(self, bgr, color_bounds):
        return cv2.inRange(bgr, color_bounds[0], color_bounds[1])

    def display_rois(self, small_rectangles, frame, n):
        for i, (x, y, w, h) in enumerate(small_rectangles):
            mask_ellipse = cv2.ellipse(np.zeros((h, w), dtype=np.uint8), (w//2, h//2), (w//2, h//2), 0, 0, 360, 255, -1)
            roi = frame[y:y+h, x:x+w]
            roi = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)

            mask_red = self.color_detect(roi, (ConfigColorsilo.RED_LOWER, ConfigColorsilo.RED_UPPER))
            mask_blue = self.color_detect(roi, (ConfigColorsilo.BLUE_LOWER, ConfigColorsilo.BLUE_UPPER))

            #cv2.imwrite(f"..\\BoutToHackNASA\\source\\godang_ws\\src\\vision\\vision\\mask{n}{i}.png", mask_red)
            cv2.imwrite(f"..\\BoutToHackNASA\\source\\godang_ws\\src\\vision\\vision\\red_Silo{n}_{i}.png", mask_red)
            cv2.imwrite(f"..\\BoutToHackNASA\\source\\godang_ws\\src\\vision\\vision\\blue_Silo{n}_{i}.png", mask_blue)
            red_area = cv2.countNonZero(cv2.bitwise_and(mask_red, mask_red, mask=mask_ellipse))
            blue_area = cv2.countNonZero(cv2.bitwise_and(mask_blue, mask_blue, mask=mask_ellipse))

            ellipse_area = np.pi * (w/2) * (h/2)
            red_percentage = (red_area / ellipse_area) * 100
            blue_percentage = (blue_area / ellipse_area) * 100
            print(f"{n} blue_percentage {blue_percentage}")
            print(f"{n} red_percentage {red_percentage}")

            if red_percentage > 40:
                self.result[n][i] = "Red"
            elif blue_percentage > 40:
                self.result[n][i] = "Blue"
            else:
                self.result[n][i] = "None"
            # print(f"ROI {n+1} {i+1} : {self.result[n][i]} ------ percentage 'red' : {red_percentage} 'blue' :{blue_percentage}")
    
    def draw_rectangles(self, silo_config, color, n):
        x, y, w, h = silo_config
        cv2.rectangle(self.frame, (x, y), (x + w, y + h), color, 2)
        large_rectangle = (x, y, w, h)
        small_rectangles = self.split_rectangle_into_rois(large_rectangle)
        self.draw_ellipse(small_rectangles)
        self.display_rois(small_rectangles, self.frame, n)
    
    def save_to_json(self):
        write = {"silo_states":{1: self.result}}
        with open("result.json", "w") as f:
            json.dump(write, f)
        with open('result.json', 'r') as f:
            data = json.load(f)
            # print(f"data {data}")
            return data["silo_states"]["1"]
 
    def Tuning(self):
         # Purple-Blue Tuning
        self.frame = cv2.imread("D:\\download\\dd7a39d5-d8b1-4599-93b0-caf9a78218dc.jpg")
        # Red-Blue Tuning
        self.frame = cv2.imread("D:\\download\\1039898b-4aeb-4b83-b6e0-149e52dfc488.jpg")
        self.frame = cv2.resize(self.frame, (640,480))
        img = cv2.cvtColor(self.frame , cv2.COLOR_BGR2YCrCb)
        img = cv2.resize(img,(400,400))
        original_img = self.frame .copy() 
        original_img = cv2.resize(original_img,(400,400))
        
        Lowblue  = cv2.getTrackbarPos("LB","Trackbar")
        Lowgreen = cv2.getTrackbarPos("LG","Trackbar")
        Lowred   = cv2.getTrackbarPos("LR","Trackbar")
    
        Highblue  = cv2.getTrackbarPos("HB","Trackbar")
        Highgreen = cv2.getTrackbarPos("HG","Trackbar")
        Highred   = cv2.getTrackbarPos("HR","Trackbar")
        
        L = [Lowblue, Lowgreen, Lowred]
        H = [Highblue, Highgreen, Highred]
        # L = [80, 0, 100]
        # H = [150, 90, 161]

        # print(L)
        lower = np.array(L, np.uint8)
        upper = np.array(H, np.uint8)
        mask = cv2.inRange(img, lower, upper)
        masked_img  = cv2.bitwise_and(img, img, mask=mask)
        YcrcbToRGB  = cv2.cvtColor(masked_img, cv2.COLOR_YCrCb2BGR)
    
        cv2.imshow("Trackbar",masked_img)
        cv2.imshow("Original Image", original_img)
        cv2.imshow("YcrcbToRGB", YcrcbToRGB)
        
    def run(self):
        while True:
            ret, self.frame = self.cap.read()
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print(f"Silo : {self.result[2]}")
                print(self.frame.shape)
                break

            # self.draw_rectangles(SiloConfig.silo_roi_1, (0, 255, 0), 0)
            # self.draw_rectangles(SiloConfig.silo_roi_2, (255, 255, 255), 0)
            # self.draw_rectangles(SiloConfig.silo_roi_3, (255, 0, 255), 1)
            # self.draw_rectangles(SiloConfig.silo_roi_4, (0, 0, 255), 2)
    
            silo = self.save_to_json()
            what = Decision(silo,"red")
            self.state = what.silo_decision()
            print(f"what {what.silo_decision()}")
            print(silo)
            # self.draw_rectangles(SiloConfig.silo_roi_5, (0, 255, 255), 4)

            cv2.imshow("frame", self.frame)

        # self.cap.release()
        cv2.destroyAllWindows()
    
    def Trackbar(self):
        cv2.namedWindow("Trackbar")
        # cv2.namedWindow("Blue Trackbar")

        def display(value):
          pass

        cv2.createTrackbar("LB","Trackbar",ColorConfigsilo.RED_LOWER[0],255,display)
        cv2.createTrackbar("LG","Trackbar",ColorConfigsilo.RED_LOWER[1],255,display)
        cv2.createTrackbar("LR","Trackbar",ColorConfigsilo.RED_LOWER[2],255,display)
        # cv2.createTrackbar("LA","Trackbar",0,255,display)
        cv2.createTrackbar("HB","Trackbar",ColorConfigsilo.RED_UPPER[0],255,display)
        cv2.createTrackbar("HG","Trackbar",ColorConfigsilo.RED_UPPER[1],255,display)
        cv2.createTrackbar("HR","Trackbar",ColorConfigsilo.RED_UPPER[2],255,display)

camera_matrix = np.array([[1029.138061543091, 0, 1013.24017],
                          [0, 992.6178560916601, 548.550898],
                          [0, 0, 1]])
dist_coeffs = np.array([0.19576717, -0.2477706, -0.00620366, 0.00395638, 0.10295289])
new_camera_matrix = np.array([[1074.76421, 0, 1022.62547],
                              [0, 1029.25677, 543.286518],
                              [0, 0, 1]])
roi = [13, 14, 1895, 1057]

if __name__ == "__main__":
    silo_detector = SiloDetection("..\\BoutToHackNASA\\source\\godang_ws\\src\\vision\\vision\\siloWeight.pt", camera_matrix, dist_coeffs, new_camera_matrix, roi)
    # ret, silo_detector.frame = silo_detector.cap.read()
    frame = cv2.imread("..\\BoutToHackNASA\\source\\godang_ws\\src\\vision\\vision\\framekmutt_silo_0292.jpg")
    frame = cv2.imread("..\\BoutToHackNASA\\source\\godang_ws\\src\\vision\\vision\\Screenshot 2024-06-03 122739.png")
    print(frame.shape)
    bboxs = sorted(silo_detector.detect_silo(frame))
    print(bboxs)

    smalls_roi = [silo_detector.split_rectangle_into_rois(bbox) for bbox in bboxs]
    print(smalls_roi)

    show = cv2.imread("..\\BoutToHackNASA\\source\\godang_ws\\src\\vision\\vision\\ans.png")
    [silo_detector.draw_ellipse(small_roi, show) for small_roi in smalls_roi]
    cv2.imwrite("..\\BoutToHackNASA\\source\\godang_ws\\src\\vision\\vision\\show.png", show)
    
    [silo_detector.display_rois(small_roi, frame, i) for i, small_roi in enumerate(smalls_roi)]
    print(silo_detector.result)

    silo = silo_detector.save_to_json()
    what = Decision(silo,"blue")
    silo_detector.state = what.silo_decision()
    print(f"state : {what.silo_decision()}")
