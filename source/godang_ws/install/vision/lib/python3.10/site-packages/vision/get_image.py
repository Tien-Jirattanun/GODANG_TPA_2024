import cv2
import numpy as np


cap = cv2.VideoCapture(0)


while True:
    ret, frame = cap.read()
    if not ret:
        break

    if cv2.waitKey(1) & 0xFF == ord('s'):
        # cv2.imwrite('./test_carlibrate/red1_test_4angle.jpg', frame)
        print("Image saved")

    cv2.imshow('frame', frame)
    print(frame.shape)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cap.destroyAllWindows()