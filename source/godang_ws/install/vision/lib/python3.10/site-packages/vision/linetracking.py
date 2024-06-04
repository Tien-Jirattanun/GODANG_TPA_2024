import cv2 
import numpy as np

# cap = cv2.VideoCapture(1)
class linetrack:
    threshold = 40
    height = 300
    width = 300
    x_pos = 100
    middleX_pos = 200
    Ly_pos = 50
    Ry_pos = 250 
    def __init__(self, frame):
        self.frame = frame
        self.RGBframe = 0
        self.BWframe = 0
        self.gray = 0
        self.bingray = 0
        self.hsv = 0
        self.binary_image = 0
        self.error = 0
        self.hoError = 0
        self.verErrorL = 0
        self.verErrorR = 0
        self.info_Horizon = ""
        self.info_ver_Left = ""
        self.info_ver_Right = ""

    def BGR2BIN(self, threshold=threshold, height=height, width=width):
        framess = self.frame
        frames = cv2.resize(framess,(height,width))
        frame = cv2.GaussianBlur(frames,(23,23),cv2.BORDER_DEFAULT)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.gray = gray
        thresh,bingray = cv2.threshold(gray,140,255,cv2.THRESH_BINARY)
        self.bingray = bingray

        hsv = cv2.cvtColor(frame , cv2.COLOR_RGB2HSV)

        h, s, v = cv2.split(hsv)
        thresh,th1 = cv2.threshold(h,threshold,255,cv2.THRESH_BINARY_INV)
        showLine = th1.copy()
        self.hsv = h
        self.RGBframe = frames
        self.binary_image = th1
        self.BWframe = showLine
        return self.binary_image
    
    def leftVertical(self, y_pos=Ly_pos):
        apos = 999
        height, width = self.binary_image.shape
        binary_image = self.binary_image[:, y_pos:y_pos+1]
        binary_image_index = np.where(np.array(binary_image[:, 0]) == 0)

        section_height = height // 30
        binary_image_8bit = []

        for i in range(30):
            section = binary_image[i * section_height:(i + 1) * section_height, :]
            binary_value = 1 if np.any(section == 0) else 0
            binary_image_8bit.append(binary_value)

        if len(binary_image_index[0]) > 0:
            apos = np.average(binary_image_index[0])
        if 0 in binary_image_8bit:
            self.verErrorL = 150 - int(np.array2string(np.round(apos).astype(np.int32), separator=''))
        else:
            self.verErrorL = 99
        self.info_ver_Left = np.array2string(np.round(apos).astype(np.int32), separator='') + ':' + ''.join(map(str, binary_image_8bit))
        return self.verErrorL

    def rightVertical(self, y_pos=Ry_pos):
        apos = 999
        height, width = self.binary_image.shape
        binary_image = self.binary_image[:, y_pos:y_pos+1]
        binary_image_index = np.where(np.array(binary_image[:, 0]) == 0)

        section_height = height // 30
        binary_image_8bit = []

        for i in range(30):
            section = binary_image[i * section_height:(i + 1) * section_height, :]
            binary_value = 1 if np.any(section == 0) else 0
            binary_image_8bit.append(binary_value)

        if len(binary_image_index[0]) > 0:
            apos = np.average(binary_image_index[0])
        if 0 in binary_image_8bit:
            self.verErrorR = 150 - int(np.array2string(np.round(apos).astype(np.int32), separator=''))
        else:
            self.verErrorR = 99

        self.info_ver_Right = np.array2string(np.round(apos).astype(np.int32), separator='') + ':' + ''.join(map(str, binary_image_8bit))
        return self.verErrorR

    def upperHorizontal(self, x_pos=x_pos):
        apos = 999
        binary_image = self.binary_image[x_pos:x_pos+1, :]
        binary_image_index = np.where(np.array(binary_image[0]) == 0)
        height, width = binary_image.shape
        section_width = width // 30
        binary_image_8bit = []

        for i in range(30):
            section = binary_image[:, i * section_width:(i + 1) * section_width]
            binary_value = 1 if np.any(section == 0) else  0
            binary_image_8bit.append(binary_value)

        if len(binary_image_index[0]) > 0:
            apos = np.average(binary_image_index[0])

        if 0 in binary_image_8bit:
            error = binary_image_8bit.index(0) + 3
            error = error - 14
            self.error = error
        else:
            error = 99
            self.error = error
        return self.error

    def lowerHorizontal(self, x_pos=middleX_pos):
            apos = 999
            binary_image = self.binary_image[x_pos:x_pos+1, :]
            binary_image_index = np.where(np.array(binary_image[0]) == 0)
            height, width = binary_image.shape
            section_width = width // 30
            binary_image_8bit = []

            for i in range(30):
                section = binary_image[:, i * section_width:(i + 1) * section_width]
                binary_value = 1 if np.any(section == 0) else  0
                binary_image_8bit.append(binary_value)

            if len(binary_image_index[0]) > 0:
                apos = np.average(binary_image_index[0])

            self.info_Horizon = np.array2string(np.round(apos).astype(np.int32), separator='') + ':' + ''.join(map(str, binary_image_8bit))
            if 0 in binary_image_8bit:
                self.hoError = 150 - int(np.array2string(np.round(apos).astype(np.int32), separator=''))
            else:
                self.hoError = 99

            return self.hoError

    def visualize_Line(self):
        x1 = self.x_pos
        x2 = self.middleX_pos
        y1 = self.Ly_pos
        y2 = self.Ry_pos
        # Draw line
        # cv2.line(self.BWframe, (0, x1), (self.width, x1), (255, 255, 255), 1)
        cv2.line(self.BWframe, (0, x2), (self.width, x2), (255, 255, 255), 1)
        cv2.line(self.BWframe, (y1, 0), (y1, self.height), (255, 255, 255), 1)
        cv2.line(self.BWframe, (y2, 0), (y2, self.height), (255, 255, 255), 1) 
        return self.BWframe
    
    def visualize(self, higherror, lowerror, lerror, rerror):
        RGB = self.RGBframe
        Bin = self.BWframe
        cv2.imshow("RGB", RGB)
        cv2.imshow("Bin", Bin)
        print(f"Left : {self.info_ver_Left} -------------- Right : {self.info_ver_Right}")
        print(f"Line Tracking : {self.info_Horizon}")
        print(f"High Error : {higherror}")
        print(f"Low Error : {lowerror}")
        print(f"Left Error : {lerror}")
        print(f"Right Error : {rerror}")

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    linetracking = linetrack(frame)
    frame = linetracking.BGR2BIN()
    lowerError = linetracking.lowerHorizontal()
    upperError = linetracking.upperHorizontal()
    leftError = linetracking.leftVertical()
    rightError = linetracking.rightVertical()
    linetracking.visualize(lowerError, upperError, leftError, rightError)
