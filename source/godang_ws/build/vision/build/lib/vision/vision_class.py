from ultralytics import YOLO

# Load a model

# model = YOLO("path/to/best.pt")  # load a custom trained model

# Export the model
# model.export(format="onnx")
  # predict on an image

class DistanceCalculator:
    def __init__(self, focal_length, object_height):
        self.focal_length = focal_length  
        self.object_height = object_height  # Actual height of the object in meters

    def calculate_distance(self, object_height_in_image):
        distance = (self.focal_length * self.object_height) / object_height_in_image
        return distance

    def process_detections(self, results):
       
        distances = []
        for bbox in results:
            xmin, ymin, xmax, ymax = bbox
            object_height_in_image = ymax - ymin
            distance = self.calculate_distance(object_height_in_image)
            distances.append(distance)
        return distances
