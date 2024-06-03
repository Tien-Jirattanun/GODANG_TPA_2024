
import numpy as np
import cv2
class ConfigColorsilo:
    # RED_LOWER = np.array([0, 0, 100], np.uint8)
    # RED_UPPER = np.array([80, 80, 255], np.uint8)
    # BLUE_LOWER = np.array([100, 0, 0], np.uint8)
    # BLUE_UPPER = np.array([255, 153, 48], np.uint8)
    # PURPLE_LOWER = np.array([80, 0, 100], np.uint8)
    # PURPLE_UPPER = np.array([150, 90, 161], np.uint8)

    RED_LOWER = np.array([0, 175, 0], np.uint8)
    RED_UPPER = np.array([255, 255, 255], np.uint8)
    BLUE_LOWER = np.array([0, 0, 155], np.uint8)
    BLUE_UPPER = np.array([255, 255, 255], np.uint8)
    PURPLE_LOWER = np.array([0, 135, 0], np.uint8)
    PURPLE_UPPER = np.array([135, 255, 255], np.uint8)

# class SiloConfig:
#     silo_roi_1 = (390,400,540,630)
#     silo_roi_2 = (710,400,860,630)
#     silo_roi_3 = (980,400,1130,630)
#     silo_roi_4 = (1340,400,1490,630)
#     silo_roi_5 = (1730,400,1870,630)

class ColorConfigball:
    RED_LOWER = np.array([140, 85, 0])
    RED_UPPER = np.array([179, 255, 255])
    BLUE_LOWER = np.array([112, 96, 46])
    BLUE_UPPER = np.array([179, 255, 255])
    PURPLE_LOWER = np.array([117, 0, 0])
    PURPLE_UPPER = np.array([138, 255, 255])

class BallConfig:
        dp=1
        minDist=100
        param1=25
        param2=30
        minRadius=125# 50
        maxRadius=200 # 100

class ColorConfig_cammid:
    RED_LOWER = np.array([112, 151, 0])
    RED_UPPER = np.array([179, 255, 255])
    BLUE_LOWER = np.array([28, 122, 60])
    BLUE_UPPER = np.array([116, 255, 255])
    PURPLE_LOWER = np.array([120, 62, 10])
    PURPLE_UPPER = np.array([141, 255, 255])
    boundingRect = 5000000