import cv2
#import cv2.dnn.cuda as cuda_dnn #*1 for gpu use (open cv with cuda support necessary)
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String


#cuda_dnn.dnn_cuda.init_device() #*1 for gpu use (open cv with cuda support necessary)
CLASSES = ['cross_parking','overtaking_allowed','overtaking_forbidden','parallel_parking','pit_in','pit_out']

model_path = "/home/johannes/Desktop/Projekte/VDIADC/Training environment/Inferencing environment/best.onnx"

#set up model
model: cv2.dnn.Net = cv2.dnn.readNetFromONNX(model_path)    #model = cuda_dnn.readNetFromONNX(model_path) #*1

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(Image,'image_topic',self.process_image,10)
        self.publisher = self.create_publisher(String,'detections_topic',10)
        self.cv_bridge = CvBridge()

    def process_image(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Preprocess input
        [height, width, _] = cv_image.shape
        length = max((height, width))
        image = np.zeros((length, length, 3), np.uint8)
        image[0:height, 0:width] = cv_image
        blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640), swapRB=True)    #blob = cuda_dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640), swapRB=True) #*1
        model.setInput(blob)

        # Run inference
        outputs = model.forward()

        # Preprocess output
        outputs = np.array([cv2.transpose(outputs[0])])    #outputs = np.array([cuda_dnn.imagesFromBlob(outputs)[0]]) #*1
        rows = outputs.shape[1]

        boxes = []
        scores = []
        class_ids = []

        for i in range(rows):
            classes_scores = outputs[0][i][4:]
            (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
            if maxScore >= 0.25:
                box = [
                    outputs[0][i][0] - (0.5 * outputs[0][i][2]), outputs[0][i][1] - (0.5 * outputs[0][i][3]),
                    outputs[0][i][2], outputs[0][i][3]]
                boxes.append(box)
                scores.append(maxScore)
                class_ids.append(maxClassIndex)
 


        # Publish the detections as ROS String message
        if scores:
            max_score_index = np.argmax(scores)
            max_score = scores[max_score_index]
            max_box = boxes[max_score_index]
            max_class_id = class_ids[max_score_index]

            result_msg = String()
            result_msg.data = f"Max Score: {max_score}\nBounding Box: {max_box}\nClass ID: {max_class_id}"
            self.publisher.publish(result_msg)



def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
