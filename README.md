# AutoNav-SLAM

ROS2 Humble üzerinde KISS-ICP tabanlı gerçek zamanlı LiDAR-IMU SLAM modülü.  
Otonom araç navigasyonu için geliştirildi — **Quensoft / ner-AI** projesi.

## Özellikler

- **KISS-ICP** odometry — minimal parametre, yüksek robustness
- LiDAR + IMU sensör füzyonu (motion deskewing)
- Gerçek zamanlı 3D harita birikimi
- ROS2 Humble uyumlu (Python)
- RViz2 görselleştirme

## Algoritma Karşılaştırması

| Algoritma | Loop Closure | Hesaplama | Parametre | Öneri |
|-----------|-------------|-----------|-----------|-------|
| KISS-ICP  | Hayır | Düşük | Az | ✅ Bu proje |
| LIO-SAM   | Evet | Orta | Orta | Üretim için |
| FAST-LIO2 | Hayır | Çok düşük | Az | Edge/gömülü |

## Kurulum

```bash
# ROS2 Humble gerekli
cd ~/ros2_ws/src
git clone https://github.com/quensoft/autonav-slam
cd ~/ros2_ws
pip install kiss-icp
colcon build --packages-select autonav_slam
source install/setup.bash
```

## Kullanım

```bash
# Varsayılan parametrelerle başlat
ros2 launch autonav_slam slam.launch.py

# Özel topic ve voxel size
ros2 launch autonav_slam slam.launch.py \
  lidar_topic:=/os_cloud_node/points \
  imu_topic:=/imu/data \
  voxel_size:=0.5

# RViz olmadan
ros2 launch autonav_slam slam.launch.py use_rviz:=false
```

## Test Verisi (MulRan Dataset)

```bash
# İlk 100 scan indir (~500MB)
python scripts/download_mulran.py --sequence KAIST01 --out ./data

# Tam dataset
python scripts/download_mulran.py --sequence KAIST01 --out ./data --full
```

## Yayınlanan Topic'ler

| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/slam/odometry` | `nav_msgs/Odometry` | Anlık konum ve yön |
| `/slam/map` | `sensor_msgs/PointCloud2` | Akümüle 3D nokta bulutu |
| TF `map → base_link` | — | Koordinat dönüşümü |

## Sonraki Adımlar

- [ ] EKF ile IMU entegrasyonu (loop closure için)
- [ ] LIO-SAM adaptöre geçiş (üretim ortamı)
- [ ] KAIST/MulRan benchmark değerlendirmesi
- [ ] Savunma ve lojistik sektörü deployment adaptörleri

## Lisans

MIT — Quensoft
