"""
AutoNav-SLAM node birim testleri.
ROS2 Humble + pytest gerektirir.

Kullanım:
    pytest tests/test_slam_node.py -v
    veya Docker:
    docker compose --profile test up unit_test
"""
from __future__ import annotations

import sys
import time
import threading
import unittest

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

# slam_node'u doğrudan import et (install/setup.bash source'landıktan sonra)
from autonav_slam.slam_node import AutoNavSLAM


# ── Yardımcı fabrika fonksiyonları ────────────────────────────────────

def _make_pointcloud(n: int = 200, stamp=None) -> PointCloud2:
    """N noktalı sahte LiDAR PointCloud2 oluşturur (rastgele ±20m)."""
    rng = np.random.default_rng(seed=42)
    pts = rng.uniform(-20.0, 20.0, (n, 3)).tolist()
    h = Header()
    if stamp is not None:
        h.stamp = stamp
    h.frame_id = "lidar"
    return pc2.create_cloud_xyz32(h, pts)


def _make_imu(stamp=None) -> Imu:
    """Küçük sabit angular_velocity ile sahte IMU mesajı."""
    msg = Imu()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = "imu"
    msg.angular_velocity.x = 0.01
    msg.angular_velocity.y = 0.0
    msg.angular_velocity.z = 0.0
    msg.linear_acceleration.z = 9.81
    return msg


# ── Test sınıfı ────────────────────────────────────────────────────────

class TestAutoNavSLAM(unittest.TestCase):
    """AutoNav-SLAM node birim testleri."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init(args=None)

        # Test altındaki node
        cls.slam = AutoNavSLAM()

        # Yardımcı node: publisher + subscriber
        cls.helper: Node = rclpy.create_node("slam_test_helper")

        cls.received_odom:  list[Odometry]    = []
        cls.received_pose:  list[PoseStamped] = []
        cls.received_map:   list[PointCloud2] = []

        cls.helper.create_subscription(
            Odometry,    "/slam/odometry", lambda m: cls.received_odom.append(m),  10)
        cls.helper.create_subscription(
            PoseStamped, "/slam/pose",     lambda m: cls.received_pose.append(m),  10)
        cls.helper.create_subscription(
            PointCloud2, "/slam/map",      lambda m: cls.received_map.append(m),   10)

        cls.pub_lidar = cls.helper.create_publisher(PointCloud2, "/lidar/points", 10)
        cls.pub_imu   = cls.helper.create_publisher(Imu,         "/imu/data",     10)

        # Her iki node'u arka plan thread'inde döndür
        cls._stop = threading.Event()

        def _spin() -> None:
            while not cls._stop.is_set():
                rclpy.spin_once(cls.slam,   timeout_sec=0.05)
                rclpy.spin_once(cls.helper, timeout_sec=0.05)

        cls._spin_thread = threading.Thread(target=_spin, daemon=True)
        cls._spin_thread.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls._stop.set()
        cls._spin_thread.join(timeout=2.0)
        cls.slam.destroy_node()
        cls.helper.destroy_node()
        rclpy.shutdown()

    # ── Yardımcı ─────────────────────────────────────────────────────

    def _publish_scan(self, n_pts: int = 200) -> None:
        stamp = self.slam.get_clock().now().to_msg()
        self.pub_imu.publish(_make_imu(stamp))
        time.sleep(0.01)
        self.pub_lidar.publish(_make_pointcloud(n_pts, stamp))

    def _wait_for(self, collection: list, initial_len: int,
                  timeout: float = 4.0) -> bool:
        deadline = time.time() + timeout
        while len(collection) <= initial_len and time.time() < deadline:
            time.sleep(0.05)
        return len(collection) > initial_len

    # ── Test: temel başlatma ─────────────────────────────────────────

    def test_01_node_name(self) -> None:
        self.assertEqual(self.slam.get_name(), "autonav_slam")

    def test_02_publishers_exist(self) -> None:
        names = [t[0] for t in self.slam.get_publisher_names_and_types_by_node(
            "autonav_slam", "")]
        self.assertIn("/slam/odometry", names)
        self.assertIn("/slam/pose",     names)
        self.assertIn("/slam/map",      names)

    # ── Test: /slam/odometry ─────────────────────────────────────────

    def test_03_odometry_published(self) -> None:
        initial = len(self.received_odom)
        for _ in range(3):
            self._publish_scan(200)
            time.sleep(0.1)
        ok = self._wait_for(self.received_odom, initial)
        self.assertTrue(ok, "/slam/odometry mesajı alınamadı")

    def test_04_odometry_frame(self) -> None:
        self.assertGreater(len(self.received_odom), 0)
        self.assertEqual(self.received_odom[-1].header.frame_id, "map")
        self.assertEqual(self.received_odom[-1].child_frame_id,  "base_link")

    def test_05_odometry_quaternion_normalized(self) -> None:
        self.assertGreater(len(self.received_odom), 0)
        q = self.received_odom[-1].pose.pose.orientation
        norm = (q.x**2 + q.y**2 + q.z**2 + q.w**2) ** 0.5
        self.assertAlmostEqual(norm, 1.0, places=2)

    # ── Test: /slam/pose ─────────────────────────────────────────────

    def test_06_pose_published(self) -> None:
        initial = len(self.received_pose)
        for _ in range(3):
            self._publish_scan(200)
            time.sleep(0.1)
        ok = self._wait_for(self.received_pose, initial)
        self.assertTrue(ok, "/slam/pose mesajı alınamadı")

    def test_07_pose_matches_odometry(self) -> None:
        self.assertGreater(len(self.received_pose), 0)
        self.assertGreater(len(self.received_odom), 0)
        pose = self.received_pose[-1].pose
        odom_pose = self.received_odom[-1].pose.pose
        self.assertAlmostEqual(pose.position.x, odom_pose.position.x, places=3)
        self.assertAlmostEqual(pose.position.y, odom_pose.position.y, places=3)

    # ── Test: /slam/map ──────────────────────────────────────────────

    def test_08_map_published_after_10_scans(self) -> None:
        initial = len(self.received_map)
        for _ in range(12):
            self._publish_scan(150)
            time.sleep(0.05)
        ok = self._wait_for(self.received_map, initial, timeout=6.0)
        self.assertTrue(ok, "/slam/map 12 scan sonra alınamadı")

    def test_09_map_frame_id(self) -> None:
        self.assertGreater(len(self.received_map), 0)
        self.assertEqual(self.received_map[-1].header.frame_id, "map")

    # ── Test: IMU buffer ─────────────────────────────────────────────

    def test_10_imu_buffer_not_negative(self) -> None:
        imu = _make_imu()
        self.pub_imu.publish(imu)
        time.sleep(0.2)
        self.assertGreaterEqual(len(self.slam._imu_buf), 0)

    def test_11_imu_buffer_window(self) -> None:
        """IMU buffer 0.5s pencereyi koruyor mu?"""
        for _ in range(20):
            imu = _make_imu(self.slam.get_clock().now().to_msg())
            self.pub_imu.publish(imu)
            time.sleep(0.02)
        time.sleep(0.2)
        # 0.5s pencerede en fazla 25 öğe olmalı (~50Hz)
        self.assertLessEqual(len(self.slam._imu_buf), 30)

    # ── Test: harita boyutu sınırı ───────────────────────────────────

    def test_12_map_scan_limit(self) -> None:
        """_map_clouds listesi MAX_SCANS sınırını aşmamalı."""
        for _ in range(10):
            self._publish_scan(80)
            time.sleep(0.03)
        self.assertLessEqual(
            len(self.slam._map_clouds),
            self.slam._MAP_MAX_SCANS,
            "_map_clouds MAX_SCANS sınırını aştı",
        )

    # ── Test: az noktalı cloud atlanıyor ─────────────────────────────

    def test_13_sparse_cloud_ignored(self) -> None:
        """< 10 noktalı cloud işlenmemeli (odom sayısı değişmemeli)."""
        # Önceki testlerden kalan pending callback'lerin bitmesini bekle
        time.sleep(0.5)
        initial = len(self.received_odom)
        # 5 noktalı cloud gönder — node bunu atlamalı
        stamp = self.slam.get_clock().now().to_msg()
        self.pub_lidar.publish(_make_pointcloud(5, stamp))
        time.sleep(0.3)
        self.assertEqual(len(self.received_odom), initial,
                         "5 noktalı cloud işlendi, atlanmalıydı")


if __name__ == "__main__":
    unittest.main()
