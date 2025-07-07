import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
from vision_msgs.msg import Point2D
from vision_msgs.msg import Pose2D
from geometry_msgs.msg import PoseWithCovariance

import cv2
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 1)
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)

    def image_cb(self, msg):
        # Convertir l'image ROS en OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # --- Ici tu ajoutes ta détection d'objet (exemple: détection de rectangles rouges) ---
        # Pour l'exemple, on crée une fausse bbox au centre
        h, w, _ = cv_img.shape
        bbox = BoundingBox2D(center=Point2D(x=w//2, y=h//2), size_x=50, size_y=50)
        detection = Detection2D(bbox=bbox)
        detection.results.append(ObjectHypothesisWithPose(id=1, score=0.99, pose=PoseWithCovariance())) # id=1: classe fictive

        # Création du message Detection2DArray
        det_array = Detection2DArray()
        det_array.header = msg.header
        det_array.detections.append(detection)

        # Publier les détections
        self.detection_pub.publish(det_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()