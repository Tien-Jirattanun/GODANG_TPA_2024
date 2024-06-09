from ultralytics import YOLO

# Load a model
model = YOLO("./ML_ball/best_yolov8.pt")  # load an official model
# model = YOLO("path/to/best.pt")  # load a custom trained model

# Export the model
# model.export(format="onnx")
results = model("./ML_ball/ball_images/frame_0066.jpg")  # predict on an image
