import cv2
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
while True:
    ret, frame = cap.read()
    
    if not ret:
        pass
    cv2.imshow("output", frame)
    print(frame.shape)
    cv2.waitKey(30)


cap.release()
cv2.destroyAllWindows()