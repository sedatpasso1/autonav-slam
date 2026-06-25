"""
AutoNav-SLAM: KISS-ICP tabanlı ROS2 LiDAR-IMU SLAM düğümü.

Subscribes:
  - /lidar/points  (sensor_msgs/PointCloud2)
  - /imu/data      (sensor_msgs/Imu)

Publishes:
  - /slam/odometry (nav_msgs/Odometry)
  - /slam/map      (sensor_msgs/PointCloud2)  -- accumulated map
  - TF: map -> odom -> base_link
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import sensor_msgs_py.point_cloud2 as pc2
from kiss_icp.pipeline import OdometryPipeline
from kiss_icp.config import KISSConfig


class AutoNavSLAM(Node):
    def __init__(self) -> None:
        super().__init__("autonav_slam")

        # ── Parametreler ──────────────────────────────────────────────
        self.declare_parameter("lidar_topic", "/lidar/points")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("voxel_size", 0.3)
        self.declare_parameter("max_range", 100.0)
        self.declare_parameter("min_range", 1.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        lidar_topic  = self.get_parameter("lidar_topic").value
        imu_topic    = self.get_parameter("imu_topic").value
        voxel_size   = self.get_parameter("voxel_size").value
        max_range    = self.get_parameter("max_range").value
        min_range    = self.get_parameter("min_range").value
        self.map_frame  = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # ── KISS-ICP pipeline ─────────────────────────────────────────
        cfg = KISSConfig()
        cfg.data.deskew       = True       # IMU ile motion deskew
        cfg.data.max_range    = max_range
        cfg.data.min_range    = min_range
        cfg.mapping.voxel_size = voxel_size
        self.pipeline = OdometryPipeline(config=cfg)

        # Akümüle harita (basit: son N scan)
        self._map_clouds: list[np.ndarray] = []
        self._MAP_MAX_SCANS = 500

        # ── TF broadcaster ───────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── QoS ──────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscribers ───────────────────────────────────────────────
        self.sub_lidar = self.create_subscription(
            PointCloud2, lidar_topic, self._lidar_cb, sensor_qos
        )
        self.sub_imu = self.create_subscription(
            Imu, imu_topic, self._imu_cb, sensor_qos
        )

        # ── Publishers ────────────────────────────────────────────────
        self.pub_odom = self.create_publisher(Odometry, "/slam/odometry", 10)
        self.pub_map  = self.create_publisher(PointCloud2, "/slam/map", 1)

        # IMU buffer (deskewing için)
        self._imu_buf: list[tuple[float, np.ndarray]] = []

        self.get_logger().info(
            f"AutoNav-SLAM baslatildi | lidar={lidar_topic} imu={imu_topic} "
            f"voxel={voxel_size}m"
        )

    # ── IMU callback ─────────────────────────────────────────────────
    def _imu_cb(self, msg: Imu) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ang = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        # Son 0.5 saniyelik IMU verisini tut
        self._imu_buf.append((t, ang))
        cutoff = t - 0.5
        self._imu_buf = [(s, v) for s, v in self._imu_buf if s >= cutoff]

    # ── LiDAR callback ───────────────────────────────────────────────
    def _lidar_cb(self, msg: PointCloud2) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # PointCloud2 → numpy (x,y,z)
        pts = np.array([
            [p[0], p[1], p[2]]
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ], dtype=np.float64)

        if len(pts) < 10:
            return

        # IMU angular velocity (deskewing için ortalama)
        if self._imu_buf:
            ang_vel = np.mean([v for _, v in self._imu_buf], axis=0)
        else:
            ang_vel = None

        # KISS-ICP adımı
        pose, _ = self.pipeline.register_frame(pts, ang_vel)

        # Pose → odometry mesajı
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id  = self.base_frame

        t_vec = pose[:3, 3]
        odom.pose.pose.position.x = float(t_vec[0])
        odom.pose.pose.position.y = float(t_vec[1])
        odom.pose.pose.position.z = float(t_vec[2])

        q = _rotation_matrix_to_quaternion(pose[:3, :3])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.pub_odom.publish(odom)

        # TF: map → base_link
        tf = TransformStamped()
        tf.header.stamp    = msg.header.stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id  = self.base_frame
        tf.transform.translation.x = float(t_vec[0])
        tf.transform.translation.y = float(t_vec[1])
        tf.transform.translation.z = float(t_vec[2])
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf)

        # Harita birikimi
        pts_global = (pose[:3, :3] @ pts.T).T + t_vec
        self._map_clouds.append(pts_global)
        if len(self._map_clouds) > self._MAP_MAX_SCANS:
            self._map_clouds.pop(0)

        # Her 10 scan'de bir haritayı yayınla
        if len(self._map_clouds) % 10 == 0:
            self._publish_map(msg.header.stamp)

    def _publish_map(self, stamp) -> None:
        all_pts = np.vstack(self._map_clouds).astype(np.float32)
        map_msg = pc2.create_cloud_xyz32(
            header=rclpy.impl.rcutils_logger.RcutilsLogger,
            points=all_pts.tolist(),
        )
        map_msg.header.stamp    = stamp
        map_msg.header.frame_id = self.map_frame
        self.pub_map.publish(map_msg)


# ── Yardımcı: rotasyon matrisi → quaternion ──────────────────────────
def _rotation_matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoNavSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
