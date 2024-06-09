import cv2
import glob
cap = cv2.VideoCapture(0)


image = glob.glob('./pattern_images/*.jpg')
count = 0

# while True:
#     ret, frame = cap.read()
#     cv2.imshow('frame', frame)

#     if cv2.waitKey(1) & 0xFF == ord('c'):
#         count += 1

#         cv2.imwrite(f'./pattern_images_9boxs/cap_2_{count}.jpg', frame)
#         print("Image saved")
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break


# cap.release()
# cap.destroyAllWindows()

for image in image:
    img = cv2.imread(image)
    cv2.imshow('image', img)
    cv2.waitKey(500)
    count += 1
    print(f"Image {count} displayed")