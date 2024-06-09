from ultralytics import YOLO

# Load a model
model = YOLO("./ML_ball/best_yolov8.pt")  # load an official model
# model = YOLO("path/to/best.pt")  # load a custom trained model

# Export the model
# model.export(format="onnx")
results = model("./ML_ball/ball_images/frame_0077.jpg", )  # predict on an image

class DistanceCalculator:
    def __init__(self, focal_length, object_height):
        self.focal_length = focal_length  
        self.object_height = object_height  # Actual height of the object in meters

    def calculate_distance(self, object_height_in_image):
        distance = (self.focal_length * self.object_height) / object_height_in_image
        return distance

    def process_detections(self, results):
       
        distances = []
        # detections = results.xyxy[0].cpu().numpy()
        for bbox in results:
            xmin, ymin, xmax, ymax = bbox
            bbox = [xmin, ymin, xmax, ymax]
            object_height_in_image = ymax - ymin
            distance = self.calculate_distance(object_height_in_image)
            distances.append(distance)
        return distances

# Example usage
focal_length_pixels = 28  # example focal length in pixels
ball_height = 0.22  # standard soccer ball diameter in meters

calculator = DistanceCalculator(focal_length_pixels, ball_height)
class_names = ['purple','red']
detections = []
for bbox in results:
    boxes = bbox.boxes
    boxes = bbox.boxes
    cls = boxes.cls.tolist()
    xyxy = boxes.xyxy
    conf = boxes.conf
    masks = bbox.masks
    for i, class_index in enumerate(cls):
        class_name = class_names[int(class_index)]
        confidence = conf[i]
        x, y, w, h = map(int, xyxy[i])
        if class_name == 'red':
            detections.append([x, y, w, h])

# detections = [[x, y, w, h], [x, y, w, h]]  # Example bounding boxes for balls in an image

# Process detections to calculate distances
distances = calculator.process_detections(detections)
print("Distances to balls:", distances)
