import numpy as np
import cv2
imagePoints=np.array([[317,690],
                       [376,575],
                       [1083,575],
                       [1011,604],
                       [1047,603],
                       [1084,603],
                       [1011,632],
                       [1047,632],
                       [1084,632]], dtype=np.float32)


# image 2_2

image = cv2.imread('./pattern_images_9boxs/cap_2_1.jpg')
for point in imagePoints:
    cv2.circle(image, (int(point[0]), int(point[1])), 5, (0, 255, 0), -1)
    cv2.imshow('image', image)
    cv2.waitKey(500)
cv2.destroyAllWindows()