import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import numpy as np

class Object3DConstructor(Node):
    def __init__(self):
        super().__init__('object_3d_constructor')
        self.get_logger().info("Node Object3DConstructor started")

        self.declare_parameter('alpha', 0.2)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('map_frame', 'map')
        self.camera_info = None
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(Float32MultiArray, '/bbox_points', self.bbox_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/object_3d_marker', 10)

    def bbox_callback(self, msg):
        data = msg.data
        if len(data) < 7 or (len(data) - 4) % 3 != 0:
            self.get_logger().warn("Invalid bbox message format")
            return

        points = np.array(data[4:], dtype=np.float32).reshape(-1, 3)
        if len(points) == 0:
            self.get_logger().warn("Empty bounding box points")
            return

        # Bounding box dimensions
        x_min, x_max = points[:, 0].min(), points[:, 0].max()
        y_min, y_max = points[:, 1].min(), points[:, 1].max()
        z_min = points[:, 2].min()

        width  = x_max - x_min
        height = y_max - y_min
        depth  = self.alpha  # Constant depth

        center = np.array([
            (x_min + x_max) / 2,
            (y_min + y_max) / 2,
            z_min + depth / 2
        ])

        # Transform to map frame
        point_cam = PointStamped()
        point_cam.header.frame_id = self.camera_frame
        point_cam.header.stamp = self.get_clock().now().to_msg()
        point_cam.point.x, point_cam.point.y, point_cam.point.z = center

        try:
            point_map = self.tf_buffer.transform(point_cam, self.map_frame, timeout=Duration(seconds=0.5))
            frame_id = self.map_frame
            position = point_map.point
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            frame_id = self.camera_frame
            position = point_cam.point

        # Marker setup
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "object_3d"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = width
        marker.scale.y = height
        marker.scale.z = depth
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.lifetime.sec = 1

        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published 3D marker at {position.x:.2f}, {position.y:.2f}, {position.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = Object3DConstructor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
