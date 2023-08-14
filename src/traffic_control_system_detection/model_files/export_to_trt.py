from ultralytics import YOLO

#Load model
model= YOLO('/home/mxck/adc/src/traffic_control_system_detection/model_files/best_s.pt')

#Export to model
model.export(format='engine', device=0)