import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import tf2_ros
import numpy as np
import cv2
import tf_transformations

class LidarCameraProjectionNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_projection')
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_info = None
        self.latest_image = None
        self.bbox = None  # [norm_x_min, norm_y_min, norm_x_max, norm_y_max]

        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_cb, 1)
        self.create_subscription(Image, "/image_out", self.image_cb, 1)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 1)
        self.create_subscription(Float32MultiArray, '/detected_ball', self.bbox_cb, 1)

        self.bbox_points_pub = self.create_publisher(Float32MultiArray, "/bbox_points", 1)
        self.image_out_projected = self.create_publisher(Image, "/image_out_projected", 1)

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def image_cb(self, msg):
        self.latest_image = msg

    def bbox_cb(self, msg):
        if len(msg.data) == 4:
            self.bbox = msg.data
        else:
            self.bbox = None

    def lidar_cb(self, msg):
        if self.camera_info is None or self.latest_image is None or self.bbox is None:
            return

        # 1. Convert LaserScan to 3D points in Lidar frame
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)
        valid = np.isfinite(ranges)
        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])
        zs = np.zeros_like(xs)
        points_lidar = np.vstack((xs, ys, zs)).T  # shape (N, 3)

        # 2. Transform points to camera frame for projection
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame=self.camera_info.header.frame_id,
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            t = trans.transform.translation
            q = trans.transform.rotation
            rot = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            trans_vec = np.array([t.x, t.y, t.z])
            points_cam = (rot @ points_lidar.T).T + trans_vec
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        # 3. Projection sur l'image
        K = np.array(self.camera_info.k).reshape(3, 3)
        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        rows, cols = cv_img.shape[:2]
        norm_x_min, norm_y_min, norm_x_max, norm_y_max = self.bbox
        x_min = int((norm_x_min * 0.5 + 0.5) * cols)
        y_min = int((norm_y_min * 0.5 + 0.5) * rows)
        x_max = int((norm_x_max * 0.5 + 0.5) * cols)
        y_max = int((norm_y_max * 0.5 + 0.5) * rows)

        points_in_bbox = []
        for idx, pt_cam in enumerate(points_cam):
            X, Y, Z = pt_cam
            if Z <= 0.1:
                continue
            uv = K @ np.array([X, Y, Z])
            u = int(uv[0] / uv[2])
            v = int(uv[1] / uv[2])
            if 0 <= u < cols and 0 <= v < rows:
                if x_min <= u <= x_max and y_min <= v <= y_max:
                    # Ajoute le point BRUT du lidar (dans son repère d'origine)
                    points_in_bbox.append(points_lidar[idx])
                    color = (0, 255, 0)  # Vert si dans la bbox
                else:
                    color = (0, 0, 255)  # Rouge sinon
                cv2.circle(cv_img, (u, v), 2, color, -1)

        # Dessine la bbox détectée
        cv2.rectangle(cv_img, (x_min, y_min), (x_max, y_max), (255, 255, 0), 2)

        # Publier l'image annotée sur /image_out_projected
        img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        img_msg.header = self.latest_image.header
        self.image_out_projected.publish(img_msg)

        # Publier bbox + points bruts sur un unique Float32MultiArray
        if points_in_bbox:
            arr = [x_min, y_min, x_max, y_max]
            for pt in points_in_bbox:
                arr.extend([float(pt[0]), float(pt[1]), float(pt[2])])
            msg_out = Float32MultiArray()
            msg_out.data = arr
            self.bbox_points_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()