import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('person_detector')

        # ROS image subscriber
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Load the SSD MobileNet v2 model
        model_path = '/home/sagar/Developer/Robotics/Getting-Started-with-ROS2-A-Tutorial-Series/workspace/ros2_ws/src/rpi_robo_imageprocessing/rpi_robo_imageprocessing/frozen_inference_graph.pb'  # Replace with your model path
        config_path = '/home/sagar/Developer/Robotics/Getting-Started-with-ROS2-A-Tutorial-Series/workspace/ros2_ws/src/rpi_robo_imageprocessing/rpi_robo_imageprocessing/graph.pbtxt'  # Replace with your config path
        self.net = cv2.dnn.readNetFromTensorflow(model_path, config_path)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Prepare the image for the SSD MobileNet model
            blob = cv2.dnn.blobFromImage(cv_image, 0.007843, (300, 300), 127.5)

            # Set the blob as input to the model
            self.net.setInput(blob)

            # Perform object detection
            detections = self.net.forward()

            h, w = cv_image.shape[:2]

            # Loop over the detections
            for i in range(detections.shape[2]):
                confidence = detections[0, 0, i, 2]

                # Only consider detections for class ID 1 (person) and confidence > 0.5
                class_id = int(detections[0, 0, i, 1])
                if confidence > 0.5 and class_id == 1:  # Class ID 1 is 'person'
                    # Get bounding box coordinates
                    box = detections[0, 0, i, 3:7] * [w, h, w, h]
                    (startX, startY, endX, endY) = box.astype('int')

                    # Draw bounding box and label
                    label = f"Person: {confidence:.2f}"
                    cv2.rectangle(cv_image, (startX, startY), (endX, endY), (0, 255, 0), 2)
                    cv2.putText(cv_image, label, (startX, startY - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Display the output frame
            cv2.imshow("Person Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
