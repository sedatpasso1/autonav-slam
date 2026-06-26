#!/usr/bin/env python3
"""
KISS-ICP vs FAST-LIO2 backend karşılaştırma.

Sentetik dairesel yörünge üzerinde iki backend'i çalıştırır:
  - KISS-ICP: sabit hız modeli ile ICP
  - FAST-LIO2: IMU preintegrasyon + ICP

Çıktı: data/backend_comparison.json
Kullanım:
  python3 scripts/compare_backends.py
  veya Docker:
  docker compose --profile compare up backend_compare
"""
from __future__ import annotations

import json
import sys
import time
import math
import os

import numpy as np

# ROS bağımlılıkları olmadan çalışabilmek için sys.path'e src ekle
_ws = os.path.join(os.path.dirname(__file__), "..", "install", "autonav_slam",
                   "local", "lib", "python3.10", "dist-packages")
if os.path.isdir(_ws):
    sys.path.insert(0, _ws)

_src = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, _src)


# ── Sentetik veri üreticisi ───────────────────────────────────────────────────

def _make_scan(center: np.ndarray, n_pts: int = 500, noise: float = 0.05,
               rng: np.random.Generator | None = None) -> np.ndarray:
    """Verilen merkez etrafında sahte LiDAR scan üretir."""
    if rng is None:
        rng = np.random.default_rng(0)
    pts = rng.uniform(-15.0, 15.0, (n_pts, 3)).astype(np.float64)
    pts += rng.normal(0, noise, pts.shape)
    return pts


def _make_imu(ang_vel: np.ndarray) -> tuple[float, np.ndarray]:
    return time.time(), ang_vel.copy()


def generate_sequence(
    n_scans: int = 60,
    radius: float = 10.0,
    angular_speed: float = 0.1,   # rad/scan
    scan_hz: float = 10.0,
    imu_hz: float = 100.0,
) -> list[dict]:
    """
    Dairesel hareket senaryosu üretir.
    Döndürür: [{"pts": ndarray, "gt_pose": ndarray, "imu": [(t, ang), ...]}]
    """
    rng = np.random.default_rng(42)
    sequence = []
    dt_scan = 1.0 / scan_hz
    dt_imu  = 1.0 / imu_hz
    n_imu_per_scan = int(imu_hz / scan_hz)

    for i in range(n_scans):
        theta = i * angular_speed
        # Gerçek konum: daire üzerinde
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        gt = np.eye(4)
        gt[0, 3] = x
        gt[1, 3] = y
        gt[0, 0] =  math.cos(theta)
        gt[0, 1] = -math.sin(theta)
        gt[1, 0] =  math.sin(theta)
        gt[1, 1] =  math.cos(theta)

        # LiDAR scan (yerel çerçevede)
        pts = _make_scan(np.zeros(3), n_pts=400, rng=rng)

        # IMU verileri (sabit yaw angular velocity)
        t0 = i * dt_scan
        imu_data = []
        for j in range(n_imu_per_scan):
            t_imu = t0 + j * dt_imu
            ang = np.array([0.0, 0.0, angular_speed / dt_scan])
            imu_data.append((t_imu, ang))

        sequence.append({
            "pts": pts,
            "gt_pose": gt,
            "stamp": t0,
            "imu": imu_data,
        })

    return sequence


# ── ATE / RTE hesabı ─────────────────────────────────────────────────────────

def compute_ate_rmse(estimated: list[np.ndarray], ground_truth: list[np.ndarray]) -> float:
    """Absolute Trajectory Error RMSE (çeviri, metre)."""
    errs = []
    for est, gt in zip(estimated, ground_truth):
        diff = est[:3, 3] - gt[:3, 3]
        errs.append(float(np.linalg.norm(diff)))
    return float(np.sqrt(np.mean(np.square(errs))))


def compute_rte_rmse(estimated: list[np.ndarray], ground_truth: list[np.ndarray],
                     step: int = 1) -> float:
    """Relative Trajectory Error RMSE (çeviri, metre) over `step` frames."""
    errs = []
    for i in range(len(estimated) - step):
        rel_est = np.linalg.inv(estimated[i]) @ estimated[i + step]
        rel_gt  = np.linalg.inv(ground_truth[i]) @ ground_truth[i + step]
        diff_t  = rel_est[:3, 3] - rel_gt[:3, 3]
        errs.append(float(np.linalg.norm(diff_t)))
    return float(np.sqrt(np.mean(np.square(errs)))) if errs else 0.0


# ── Backend çalıştırıcı ───────────────────────────────────────────────────────

def run_kiss_icp(sequence: list[dict]) -> tuple[list[np.ndarray], float]:
    from kiss_icp.kiss_icp import KissICP
    from kiss_icp.config import KISSConfig

    cfg = KISSConfig()
    cfg.data.deskew        = False
    cfg.data.max_range     = 100.0
    cfg.data.min_range     = 1.0
    cfg.mapping.voxel_size = 0.3
    pipeline = KissICP(config=cfg)

    poses = []
    t0 = time.perf_counter()
    for frame in sequence:
        pipeline.register_frame(frame["pts"], timestamps=np.array([]))
        poses.append(pipeline.last_pose.copy())
    elapsed = time.perf_counter() - t0
    return poses, elapsed


def run_fast_lio2(sequence: list[dict]) -> tuple[list[np.ndarray], float]:
    from autonav_slam.fast_lio2_node import FastLIO2Backend

    backend = FastLIO2Backend(voxel_size=0.1, max_range=100.0, min_range=1.0)

    poses = []
    t0 = time.perf_counter()
    for frame in sequence:
        for t_imu, ang in frame["imu"]:
            backend.add_imu(t_imu, ang)
        pose = backend.register_frame(frame["pts"], frame["stamp"])
        poses.append(pose.copy())
    elapsed = time.perf_counter() - t0
    return poses, elapsed


# ── Ana ───────────────────────────────────────────────────────────────────────

def main() -> None:
    print("AutoNav-SLAM backend karşılaştırması başlıyor...")
    print("Sentetik daire yörüngesi (60 scan, r=10m) üretiliyor...")

    sequence = generate_sequence(n_scans=60)
    gt_poses = [f["gt_pose"] for f in sequence]

    print("KISS-ICP çalıştırılıyor...")
    kiss_poses, kiss_time = run_kiss_icp(sequence)
    kiss_ate = compute_ate_rmse(kiss_poses, gt_poses)
    kiss_rte = compute_rte_rmse(kiss_poses, gt_poses)

    print("FAST-LIO2 (Python) çalıştırılıyor...")
    fl2_poses, fl2_time  = run_fast_lio2(sequence)
    fl2_ate = compute_ate_rmse(fl2_poses, gt_poses)
    fl2_rte = compute_rte_rmse(fl2_poses, gt_poses)

    n = len(sequence)
    result = {
        "n_scans": n,
        "backends": {
            "kiss_icp": {
                "ate_rmse_m":          round(kiss_ate, 4),
                "rte_rmse_m":          round(kiss_rte, 4),
                "total_time_s":        round(kiss_time, 3),
                "avg_ms_per_scan":     round(kiss_time / n * 1000, 2),
            },
            "fast_lio2": {
                "ate_rmse_m":          round(fl2_ate, 4),
                "rte_rmse_m":          round(fl2_rte, 4),
                "total_time_s":        round(fl2_time, 3),
                "avg_ms_per_scan":     round(fl2_time / n * 1000, 2),
            },
        },
        "improvement": {
            "ate_reduction_pct": round((1 - fl2_ate / kiss_ate) * 100, 1) if kiss_ate else 0,
            "rte_reduction_pct": round((1 - fl2_rte / kiss_rte) * 100, 1) if kiss_rte else 0,
        },
    }

    os.makedirs("data", exist_ok=True)
    out_path = "data/backend_comparison.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    print("\n── Sonuçlar ──────────────────────────────────────────────")
    print(f"{'Metrik':<30} {'KISS-ICP':>12} {'FAST-LIO2':>12}")
    print("-" * 56)
    ki = result["backends"]["kiss_icp"]
    fl = result["backends"]["fast_lio2"]
    print(f"{'ATE RMSE (m)':<30} {ki['ate_rmse_m']:>12.4f} {fl['ate_rmse_m']:>12.4f}")
    print(f"{'RTE RMSE (m)':<30} {ki['rte_rmse_m']:>12.4f} {fl['rte_rmse_m']:>12.4f}")
    print(f"{'Ortalama süre (ms/scan)':<30} {ki['avg_ms_per_scan']:>12.2f} {fl['avg_ms_per_scan']:>12.2f}")
    print(f"\nATE iyileşmesi: {result['improvement']['ate_reduction_pct']}%")
    print(f"RTE iyileşmesi: {result['improvement']['rte_reduction_pct']}%")
    print(f"\nRapor kaydedildi: {out_path}")


if __name__ == "__main__":
    main()
