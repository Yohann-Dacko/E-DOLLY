import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import ball_tracker.process_image as proc

class DetectBall(Node):
    def __init__(self):
        super().__init__('detect_ball')
        self.get_logger().info('Initialisation de la détection de balle...')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, "/image_in", self.image_callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.bbox_pub = self.create_publisher(Float32MultiArray, "/detected_ball", 1)

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            keypoints, annotated_image = proc.find_circles(image)

            # Publier l'image annotée
            image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            image_msg.header = msg.header
            self.image_out_pub.publish(image_msg)

            # Trouver la plus grande bbox
            if keypoints:
                largest_bbox = max(keypoints, key=lambda kp: abs((kp[2] - kp[0]) * (kp[3] - kp[1])))
                bbox_msg = Float32MultiArray(data=largest_bbox)
                self.bbox_pub.publish(bbox_msg)

        except Exception as e:
            self.get_logger().error(f"Erreur dans image_callback : {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DetectBall()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            proc.wait_on_gui()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
