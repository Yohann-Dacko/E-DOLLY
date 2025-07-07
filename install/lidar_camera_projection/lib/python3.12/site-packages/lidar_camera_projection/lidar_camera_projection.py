import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo
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

        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_cb, 1)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 1)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 1)

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def image_cb(self, msg):
        self.latest_image = msg

    def lidar_cb(self, msg):
        if self.camera_info is None or self.latest_image is None:
            return

        # 1. Convert LaserScan to 3D points in Lidar frame
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)
        valid = np.isfinite(ranges)
        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])
        zs = np.zeros_like(xs)
        points_lidar = np.vstack((xs, ys, zs)).T

        # 2. Transformer les points dans la frame caméra
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
        points_proj = []
        for pt in points_cam:
            X, Y, Z = pt
            if Z <= 0.1:
                continue
            uv = K @ np.array([X, Y, Z])
            u = int(uv[0] / uv[2])
            v = int(uv[1] / uv[2])
            points_proj.append((u, v))

        # 4. Dessiner les points sur l'image
        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        for (u, v) in points_proj:
            if 0 <= u < cv_img.shape[1] and 0 <= v < cv_img.shape[0]:
                cv2.circle(cv_img, (u, v), 2, (0, 0, 255), -1)

        # 5. Afficher l'image
        cv2.imshow("Lidar projected on camera", cv_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()