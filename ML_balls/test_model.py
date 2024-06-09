# from ultralytics import YOLO
# from PIL import Image
# import cv2
# model = YOLO("./ML_ball/best.pt") 

# cap = cv2.VideoCapture("./video/WIN_20240509_22_46_14_Pro.mp4")
# # # Open the video file
# # video_path = "path/to/your/video/file.mp4"
# # cap = cv2.VideoCapture(video_path)

# # Loop through the video frames
# while cap.isOpened():
#     success, frame = cap.read()

#     if success:
#         results = model(frame)
#         annotated_frame = results[0].plot()

#         # for i, r in enumerate(results):
#         #     im_bgr = r.plot(labels = False)  # BGR-order numpy array
#         #     im_rgb = Image.fromarray(im_bgr[..., ::-1])  # RGB-order PIL image

#         #     r.show()
#         #     r.save(filename=f"results{i}.jpg")

#         cv2.imshow("YOLOv8 Inference", annotated_frame)
#         if cv2.waitKey(1) & 0xFF == ord("q"):
#             break
#     else:
#         # Break the loop if the end of the video is reached
#         break

# # Release the video capture object and close the display window
# cap.release()
# cv2.destroyAllWindows()

from ultralytics import YOLOv10
from PIL import Image
import cv2
import numpy as np
model = YOLOv10("./blueballWeight.pt") 
import torch 
# import utils
# model_path = "./ML_ball/best.pt"
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='./ML_ball/best_v5.pt', force_reload=True, trust_repo=True)

cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture("./video_compet/Screen Recording 2567-05-11 at 15.15.44.mp4")
# Define custom offsets for each class
class_custom_offsets = {
    'blue': (0+50, 0-10),
    'purple': (0+60, 0),

}
 # print("file exists?", os.path.exists('./WIN_20240509_22_46_14_Pro.mp4'))

class_names = ['blue', 'purple']
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        results = model(frame)
        annotated_frame = results[0].plot()     
        for result in results:
            boxes = result.boxes
            cls = boxes.cls.tolist()
            xyxy = boxes.xyxy
            conf = boxes.conf
            masks = result.masks
            for i, class_index in enumerate(cls):
                class_name = class_names[int(class_index)]
                confidence = conf[i]
                x, y, w, h = map(int, xyxy[i])

                
                cv2.rectangle(frame, (x, y), (w, h), (0, 0, 255), 2)
                text_offset_x, text_offset_y = class_custom_offsets.get(class_name, (0, 0))
                org = (x + text_offset_x, y + text_offset_y)
                label = f"{class_name}: {confidence:.2f}"
                cv2.putText(frame, label, org, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), thickness=3, lineType=cv2.LINE_AA)
        cv2.resizeWindow("Video", 800, 600)
        cv2.imshow("Video", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
cap.release()
cv2.destroyAllWindows()