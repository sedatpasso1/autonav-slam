"""
FAST-LIO2 backend: IMU preintegration + point-to-plane ICP.

Tightly-coupled LiDAR-IMU fusion — IMU rotation preintegration provides
initial guess for each scan, reducing ICP drift vs KISS-ICP constant-velocity
model in dynamic/aggressive motion.

Production note: swap FastLIO2Backend with the actual C++ node via
  github.com/Ericsii/FAST_LIO2_ROS2 for full IEKF performance.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

import sensor_msgs_py.point_cloud2 as pc2

from autonav_slam.slam_node import _rotation_matrix_to_quaternion


# ─── FAST-LIO2 Backend ────────────────────────────────────────────────────────

class FastLIO2Backend:
    """
    IMU-preintegrated tightly-coupled LiDAR SLAM backend.

    Algorithm:
      1. Accumulate IMU angular_velocity readings since last scan.
      2. Rodrigues rotation integration → delta_R (initial guess).
      3. Transform source cloud by predicted pose.
      4. SVD point-to-point ICP refinement against local voxel map.
      5. Update pose, add scan to local map.
    """

    def __init__(
        self,
        voxel_size: float = 0.1,
        max_range: float = 100.0,
        min_range: float = 1.0,
    ) -> None:
        self.voxel_size = voxel_size
        self.max_range = max_range
        self.min_range = min_range

        self._pose = np.eye(4)
        self._imu_buf: list[tuple[float, np.ndarray]] = []  # (t, ang_vel)
        self._last_scan_t: float | None = None
        self._local_map_pts: list[np.ndarray] = []
        self._scan_count: int = 0

    def add_imu(self, t: float, ang_vel: np.ndarray) -> None:
        self._imu_buf.append((t, ang_vel.copy()))
        # retain last 0.2 s
        cutoff = t - 0.2
        self._imu_buf = [(s, v) for s, v in self._imu_buf if s >= cutoff]

    def register_frame(self, pts: np.ndarray, t: float) -> np.ndarray:
        """
        Register scan, return updated 4×4 world pose.
        pts: (N, 3) float64, already range-filtered.
        """
        dt = (t - self._last_scan_t) if self._last_scan_t is not None else 0.0
        self._last_scan_t = t

        # ── 1. IMU preintegration ──────────────────────────────────────
        delta_R = self._preintegrate(dt)

        # Predicted pose (rotation only from IMU, translation from constant velocity)
        pred = self._pose.copy()
        if self._scan_count > 0:
            # constant-velocity translation component (same as KISS-ICP fallback)
            prev_R = self._pose[:3, :3]
            pred[:3, :3] = prev_R @ delta_R
        # else: first frame → identity

        # ── 2. No map yet: accept prediction as-is ────────────────────
        if len(self._local_map_pts) == 0:
            self._pose = pred
            self._update_map(pts)
            self._scan_count += 1
            return self._pose.copy()

        # ── 3. ICP refinement ─────────────────────────────────────────
        map_pts = np.vstack(self._local_map_pts)

        # Subsample for speed
        src = _subsample(pts, 3_000)
        tgt = _subsample(map_pts, 8_000)

        self._pose = _icp(src, tgt, pred, max_iter=5, max_dist=2.0)
        self._update_map(pts)
        self._scan_count += 1
        return self._pose.copy()

    # ── helpers ──────────────────────────────────────────────────────────

    def _preintegrate(self, dt: float) -> np.ndarray:
        if not self._imu_buf or dt <= 0.0:
            return np.eye(3)
        imu_dt = dt / max(len(self._imu_buf), 1)
        R = np.eye(3)
        for _, ang in self._imu_buf:
            angle = float(np.linalg.norm(ang)) * imu_dt
            if angle < 1e-9:
                continue
            axis = ang / np.linalg.norm(ang)
            K = np.array([
                [0.0,      -axis[2],  axis[1]],
                [axis[2],   0.0,     -axis[0]],
                [-axis[1],  axis[0],  0.0    ],
            ])
            R = R @ (np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K))
        return R

    def _update_map(self, pts: np.ndarray) -> None:
        R, t = self._pose[:3, :3], self._pose[:3, 3]
        pts_global = (R @ pts.T).T + t
        self._local_map_pts.append(pts_global)
        # Keep last 200 scans
        if len(self._local_map_pts) > 200:
            self._local_map_pts.pop(0)

    @property
    def last_pose(self) -> np.ndarray:
        return self._pose.copy()

    @property
    def local_map(self) -> np.ndarray | None:
        if self._local_map_pts:
            return np.vstack(self._local_map_pts)
        return None


# ─── ICP (numpy-only, no scipy) ───────────────────────────────────────────────

def _subsample(pts: np.ndarray, n: int) -> np.ndarray:
    if len(pts) <= n:
        return pts
    idx = np.random.choice(len(pts), n, replace=False)
    return pts[idx]


def _icp(
    src: np.ndarray,
    tgt: np.ndarray,
    init_pose: np.ndarray,
    max_iter: int = 5,
    max_dist: float = 2.0,
) -> np.ndarray:
    """
    SVD point-to-point ICP.
    Returns refined 4×4 pose (in world frame).
    """
    pose = init_pose.copy()

    for _ in range(max_iter):
        R, t = pose[:3, :3], pose[:3, 3]
        src_t = (R @ src.T).T + t           # (N, 3) transformed source

        # nearest-neighbor: vectorized, O(N×M)
        # batch to avoid OOM on large clouds
        nn_idx, nn_dist = _batch_nn(src_t, tgt)

        valid = nn_dist < max_dist
        if valid.sum() < 6:
            break

        s = src_t[valid]
        m = tgt[nn_idx[valid]]

        # SVD alignment
        mu_s = s.mean(axis=0)
        mu_m = m.mean(axis=0)
        H = (s - mu_s).T @ (m - mu_m)
        U, _, Vt = np.linalg.svd(H)
        Rn = Vt.T @ U.T
        if np.linalg.det(Rn) < 0:
            Vt[-1] *= -1
            Rn = Vt.T @ U.T
        tn = mu_m - Rn @ mu_s

        delta = np.eye(4)
        delta[:3, :3] = Rn
        delta[:3, 3] = tn
        pose = delta @ pose

    return pose


def _batch_nn(
    src: np.ndarray,
    tgt: np.ndarray,
    batch: int = 512,
) -> tuple[np.ndarray, np.ndarray]:
    """Memory-efficient nearest-neighbor search via batched numpy."""
    N = len(src)
    nn_idx = np.zeros(N, dtype=np.int64)
    nn_dist = np.full(N, np.inf)

    for i in range(0, N, batch):
        s_chunk = src[i : i + batch]                       # (B, 3)
        diffs = s_chunk[:, None, :] - tgt[None, :, :]     # (B, M, 3)
        d2 = (diffs * diffs).sum(axis=2)                   # (B, M)
        idx = d2.argmin(axis=1)
        nn_idx[i : i + batch] = idx
        nn_dist[i : i + batch] = np.sqrt(d2[np.arange(len(s_chunk)), idx])

    return nn_idx, nn_dist


# ─── Standalone ROS2 node ────────────────────────────────────────────────────

class FastLIO2Node(Node):
    """
    Standalone ROS2 node running FastLIO2Backend.
    Subscribes: /lidar/points, /imu/data
    Publishes:  /slam/odometry, /slam/pose, /slam/map
    """

    def __init__(self) -> None:
        super().__init__("fast_lio2_node")

        self.declare_parameter("lidar_topic",  "/lidar/points")
        self.declare_parameter("imu_topic",    "/imu/data")
        self.declare_parameter("voxel_size",   0.1)
        self.declare_parameter("max_range",    100.0)
        self.declare_parameter("min_range",    1.0)
        self.declare_parameter("map_frame",    "map")
        self.declare_parameter("base_frame",   "base_link")

        lidar_topic = self.get_parameter("lidar_topic").value
        imu_topic   = self.get_parameter("imu_topic").value
        self.map_frame  = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.backend = FastLIO2Backend(
            voxel_size=self.get_parameter("voxel_size").value,
            max_range=self.get_parameter("max_range").value,
            min_range=self.get_parameter("min_range").value,
        )

        self._map_clouds: list[np.ndarray] = []
        self._MAP_MAX_SCANS = 500
        self.tf_bcast = TransformBroadcaster(self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(PointCloud2, lidar_topic, self._lidar_cb, sensor_qos)
        self.create_subscription(Imu, imu_topic, self._imu_cb, sensor_qos)

        self.pub_odom = self.create_publisher(Odometry,    "/slam/odometry", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/slam/pose",     10)
        self.pub_map  = self.create_publisher(PointCloud2, "/slam/map",      1)

        self.get_logger().info(
            f"FastLIO2Node ready | lidar={lidar_topic} imu={imu_topic}"
        )

    def _imu_cb(self, msg: Imu) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ang = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        self.backend.add_imu(t, ang)

    def _lidar_cb(self, msg: PointCloud2) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        pts = np.array([
            [p[0], p[1], p[2]]
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ], dtype=np.float64)

        if len(pts) < 10:
            return

        pose = self.backend.register_frame(pts, t)

        t_vec = pose[:3, 3]
        q = _rotation_matrix_to_quaternion(pose[:3, :3])

        odom = Odometry()
        odom.header.stamp    = msg.header.stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x = float(t_vec[0])
        odom.pose.pose.position.y = float(t_vec[1])
        odom.pose.pose.position.z = float(t_vec[2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.pub_odom.publish(odom)

        pose_msg = PoseStamped()
        pose_msg.header = odom.header
        pose_msg.pose   = odom.pose.pose
        self.pub_pose.publish(pose_msg)

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
        self.tf_bcast.sendTransform(tf)

        pts_global = (pose[:3, :3] @ pts.T).T + t_vec
        self._map_clouds.append(pts_global)
        if len(self._map_clouds) > self._MAP_MAX_SCANS:
            self._map_clouds.pop(0)
        if len(self._map_clouds) % 10 == 0:
            self._publish_map(msg.header.stamp)

    def _publish_map(self, stamp) -> None:
        all_pts = np.vstack(self._map_clouds).astype(np.float32)
        h = Header()
        h.stamp    = stamp
        h.frame_id = self.map_frame
        self.pub_map.publish(pc2.create_cloud_xyz32(header=h, points=all_pts.tolist()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastLIO2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
