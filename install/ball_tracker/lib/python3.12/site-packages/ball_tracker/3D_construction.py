import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import CameraInfo, PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
import numpy as np
import tf2_ros
import tf2_geometry_msgs  # noqa
import sensor_msgs_py.point_cloud2 as pc2
import math
from tf_transformations import quaternion_from_euler, quaternion_matrix

class ClosestPointMarkerNode(Node):
    def __init__(self):
        super().__init__('closest_point_marker')

        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('map_frame', 'map')
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        self.create_subscription(Odometry, '/odom', self.odom_cb, 1)
        self.marker_pub = self.create_publisher(Marker, '/closest_point_marker', 1)
        self.pc_pub = self.create_publisher(PointCloud2, '/obstacle_points', 1)
        self.create_subscription(Float32MultiArray, '/bbox_points', self.bbox_points_cb, 1)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_cb, 1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.K = None
        self.marker_id = 0
        self.robot_pos = None

    def camera_info_cb(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3, 3)
        self.get_logger().info("Matrice K reçue et enregistrée.")

    def odom_cb(self, msg):
        self.robot_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]

    def bbox_points_cb(self, msg):
        if self.K is None:
            self.get_logger().warn("Matrice K non disponible, attendre un message /camera/camera_info")
            return
        if self.robot_pos is None:
            self.get_logger().warn("Position du robot non encore reçue.")
            return

        data = msg.data
        if len(data) < 7 or (len(data) - 4) % 3 != 0:
            self.get_logger().warn("Format de message invalide reçu sur /bbox_points")
            return

        bbox = data[0:4]
        points = np.array(data[4:], dtype=np.float32).reshape(-1, 3)
        if len(points) == 0:
            self.get_logger().warn("Aucun point reçu dans la bounding box")
            return

        distances = np.linalg.norm(points, axis=1)
        idx_min = np.argmin(distances)
        point_min = points[idx_min]

        x_min, y_min, x_max, y_max = bbox
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]
        depth = distances[idx_min]

        X1 = (x_min - cx) * depth / fx
        Y1 = (y_min - cy) * depth / fy
        X2 = (x_max - cx) * depth / fx
        Y2 = (y_max - cy) * depth / fy

        self.get_logger().info(f"Point le plus proche (camera frame): {point_min} (distance = {depth:.2f} m)")

        pt = PointStamped()
        pt.header.frame_id = "laser_frame"
        pt.header.stamp.sec = 0
        pt.header.stamp.nanosec = 0
        pt.point.x = float(point_min[0])
        pt.point.y = float(point_min[1])
        pt.point.z = float(point_min[2])

        try:
            pt_map = self.tf_buffer.transform(pt, self.map_frame, timeout=Duration(seconds=0.5))
            point_map = [pt_map.point.x, pt_map.point.y, pt_map.point.z]
            frame_id = self.map_frame
            self.get_logger().info(f"Point projeté dans {self.map_frame} : {point_map}")
        except Exception as e:
            self.get_logger().warn(f"TF transform vers '{self.map_frame}' échouée : {e}")
            point_map = point_min
            frame_id = self.camera_frame

        # Orientation du marker face au robot
        vec = np.array(self.robot_pos) - np.array(point_map)
        yaw = math.atan2(vec[1], vec[0]) + math.pi/2  # Ajustement si besoin
        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)

        # Création du marker
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "closest_point"
        marker.id = self.marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(point_map[0])
        marker.pose.position.y = float(point_map[1])
        marker.pose.position.z = float(0.5 / 2)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = float(abs(X2 - X1))
        marker.scale.y = float(0.5)
        marker.scale.z = float(abs(Y2 - Y1))
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime.sec = 3

        self.marker_pub.publish(marker)
        self.marker_id =0

        # Convertir le marker en nuage de points et publier
        points_3d = self.marker_to_points(marker, resolution=0.05)
        cloud_msg = pc2.create_cloud_xyz32(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=marker.header.frame_id),
            points=points_3d
        )
        self.pc_pub.publish(cloud_msg)

    def marker_to_points(self, marker, resolution=0.05):
        cx = marker.pose.position.x
        cy = marker.pose.position.y
        cz = marker.pose.position.z
        sx = marker.scale.x
        sy = marker.scale.y
        sz = marker.scale.z

        x_vals = np.arange(-sx / 2, sx / 2, resolution)
        y_vals = np.arange(-sy / 2, sy / 2, resolution)
        z_vals = np.arange(-sz / 2, sz / 2, resolution)

        quat = [
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w
        ]
        transform = quaternion_matrix(quat)[:3, :3]

        points = []
        for x in x_vals:
            for y in y_vals:
                for z in z_vals:
                    local_point = np.array([x, y, z])
                    rotated_point = transform @ local_point
                    world_point = rotated_point + np.array([cx, cy, cz])
                    points.append(tuple(world_point))
        return points

def main(args=None):
    rclpy.init(args=args)
    node = ClosestPointMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()