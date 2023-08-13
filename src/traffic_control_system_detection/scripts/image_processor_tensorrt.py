import numpy
from ultralytics import YOLO

# Load a model
model = YOLO('/home/mxck/adc/src/traffic_control_system_detection/model_files/best.engine', task = 'detect')  # pretrained YOLOv8n model

# Run batched inference on a list of images
results = model(['/home/mxck/adc/src/traffic_control_system_detection/scripts/2.jpeg'], device = 0, imgsz = 320, conf = 0.5)  # return a list of Results objects

# Process results list
for result in results:
    boxes = result.boxes  # Boxes object for bbox outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs

boxes=boxes.data.cpu()
boxes=boxes.numpy()

print("boxes that come out of the model")
print(boxes)

